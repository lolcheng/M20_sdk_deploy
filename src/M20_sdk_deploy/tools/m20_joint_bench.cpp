#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <poll.h>
#include <unistd.h>

#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "sensor_msgs/msg/imu.hpp"

#include "dds_types.h"
#include "m20_joint_layout.hpp"
#include "m20_pro_urdf_limits.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using JointData = drdds::msg::JointsData;
using JointCommand = drdds::msg::JointsDataCmd;
using Imu = sensor_msgs::msg::Imu;
using JointValues = std::array<float, m20::joint_names.size()>;

constexpr auto period = 5ms;
constexpr auto feedback_timeout = 100ms;
constexpr auto writer_quiet_time = 1s;
constexpr auto arm_timeout = 3s;
constexpr auto release_duration = 250ms;
constexpr float radians_per_degree = 0.01745329252F;
constexpr float position_margin = 5.0F * radians_per_degree;
constexpr float max_wheel_goal = 2.0F;
constexpr float max_leg_feedback_speed = 0.8F;
constexpr float max_wheel_feedback_speed = 2.5F;
constexpr float leg_step = 5.0F * radians_per_degree * 0.005F;
constexpr float wheel_step = 0.005F;
constexpr float leg_kp = 200.0F;
constexpr float leg_kd = 2.5F;
constexpr float damping_kd = 1.0F;

constexpr char help[] =
    "status | limits | arm | stop | disarm | quit\n"
    "pos <joint> <deg>     set an absolute M20 Pro URDF angle (e.g. pos fl_hipx 25)\n"
    "jog <joint> <deg>     add to the current target (e.g. jog fl_hipx +20)\n"
    "Leg targets are clamped 5 deg inside the URDF limits.\n"
    "wheel <leg> <rad/s>   hold speed until changed or stopped (e.g. wheel fr 0.3)\n"
    "Legs: fl, fr, hl, hr; leg joints: hipx, hipy, knee. Type stop before selecting another motor.\n";

volatile std::sig_atomic_t interrupted = 0;

void on_signal(int) { interrupted = 1; }

enum class Mode { Observe, Hardware, Simulation, Test };
enum class Phase { Idle, Pairing, Armed, Releasing };

const char *topic_prefix(Mode mode) {
    if (mode == Mode::Simulation) return "/m20_joint_bench_sim";
    if (mode == Mode::Test) return "/m20_joint_bench_test";
    return "";
}

const char *mode_name(Mode mode) {
    switch (mode) {
        case Mode::Hardware: return "Hardware bench";
        case Mode::Simulation: return "MuJoCo bench";
        case Mode::Test: return "Isolated test";
        case Mode::Observe: return "Read-only";
    }
    return "Bench";
}

constexpr bool is_wheel(std::size_t index) { return index % 4 == 3; }

constexpr auto ordered_urdf_limits() {
    std::array<m20::UrdfLimit, m20::joint_names.size()> limits{};
    for (std::size_t index = 0; index < limits.size(); ++index) {
        for (const auto &limit : m20::pro_urdf_limits) {
            if (limit.name == std::string_view(m20::joint_names[index])) limits[index] = limit;
        }
    }
    return limits;
}

constexpr auto joint_limits = ordered_urdf_limits();

constexpr bool valid_joint_limits() {
    for (std::size_t index = 0; index < joint_limits.size(); ++index) {
        const auto &limit = joint_limits[index];
        if (is_wheel(index) ? !limit.name.empty() : limit.upper - limit.lower <= 2.0F * position_margin) {
            return false;
        }
    }
    return true;
}

static_assert(valid_joint_limits(), "M20 Pro URDF must provide usable limits for all 12 leg joints");

template <typename Stamp>
int64_t nanoseconds(const Stamp &stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

template <typename Vector>
bool finite_vector(const Vector &value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double parse_value(std::string value, const std::string &unit) {
    if (value.size() > unit.size() && value.compare(value.size() - unit.size(), unit.size(), unit) == 0) {
        value.resize(value.size() - unit.size());
    }
    std::size_t parsed = 0;
    const double number = std::stod(value, &parsed);
    if (parsed != value.size() || !std::isfinite(number)) {
        throw std::invalid_argument("Expected a finite number.");
    }
    return number;
}

std::size_t joint_index(const std::string &name) {
    const auto match = std::find(m20::joint_names.begin(), m20::joint_names.end(), name);
    if (match == m20::joint_names.end()) {
        throw std::invalid_argument("Unknown motor. Type help for names.");
    }
    return static_cast<std::size_t>(match - m20::joint_names.begin());
}

struct FeedbackClock {
    void update(int64_t stamp, bool sample_valid) {
        valid = sample_valid && stamp > 0;
        if (valid && stamp != last_stamp) {
            advances = stamp > last_stamp && last_stamp > 0;
            received = Clock::now();
            last_stamp = stamp;
        }
    }

    bool ready(Clock::time_point now) const {
        return valid && advances && now - received <= feedback_timeout;
    }

    int64_t last_stamp = 0;
    Clock::time_point received{};
    bool advances = false;
    bool valid = false;
};

class JointBench : public rclcpp::Node {
public:
    explicit JointBench(Mode mode)
        : Node("m20_joint_bench"), mode_(mode),
          prefix_(topic_prefix(mode)),
          cmd_topic_(prefix_ + "/JOINTS_CMD"), started_(Clock::now()), foreign_at_(started_) {
        const auto sensors = rclcpp::SensorDataQoS().keep_last(1);
        joints_sub_ = create_subscription<JointData>(prefix_ + "/JOINTS_DATA", sensors,
            [this](JointData::ConstSharedPtr message) { read_joints(*message); });
        imu_sub_ = create_subscription<Imu>(prefix_ + "/IMU", sensors,
            [this](Imu::ConstSharedPtr message) { read_imu(*message); });
        command_monitor_ = create_subscription<JointCommand>(cmd_topic_, sensors,
            [this](JointCommand::ConstSharedPtr, const rclcpp::MessageInfo &info) {
                bool own = false;
                const auto &gid = info.get_rmw_message_info().publisher_gid;
                if (!own_gid_ || rmw_compare_gids_equal(&gid, &*own_gid_, &own) != RMW_RET_OK || !own) {
                    foreign_at_ = Clock::now();
                    if (phase_ != Phase::Idle) {
                        drop_output("Another command stream was observed; output stopped.");
                    }
                }
            });
        timer_ = create_wall_timer(period, [this] { tick(); });
    }

    void request_quit() {
        if (!closing_) {
            closing_ = true;
            release("Exiting");
        }
    }

    bool done() const { return closing_ && phase_ == Phase::Idle; }
    bool closing() const { return closing_; }

    void command(const std::string &line) {
        std::istringstream input(line);
        std::vector<std::string> words;
        for (std::string word; input >> word;) {
            words.push_back(word);
        }
        if (words.empty()) {
            return;
        }
        const auto &verb = words[0];
        if (words.size() == 1) {
            if (verb == "help") std::cout << help;
            else if (verb == "status") print_status();
            else if (verb == "limits") print_limits();
            else if (verb == "arm") arm();
            else if (verb == "stop") stop_motion();
            else if (verb == "disarm") release("Disarmed");
            else if (verb == "quit") request_quit();
            else throw std::invalid_argument("Unknown command. Type help.");
            return;
        }
        if (words.size() != 3 || (verb != "jog" && verb != "pos" && verb != "wheel")) {
            throw std::invalid_argument("Expected pos/jog <joint> <deg> or wheel <leg> <rad/s>.");
        }
        if (phase_ != Phase::Armed) {
            throw std::runtime_error("Not armed. Wait for 'Armed' before requesting movement.");
        }
        const bool wheel = verb == "wheel";
        const auto index = joint_index(wheel ? words[1] + "_wheel" : words[1]);
        if (wheel != is_wheel(index)) {
            throw std::invalid_argument("Use position commands for leg joints only.");
        }
        if (selected_ && *selected_ != index) {
            throw std::invalid_argument("Use stop before selecting another motor.");
        }
        const double value = parse_value(words[2], wheel ? "rad/s" : "deg");
        if (wheel) {
            if (std::abs(value) > max_wheel_goal) {
                throw std::invalid_argument("Wheel goal must stay within +/-2 rad/s.");
            }
            wheel_goal_ = static_cast<float>(value);
            std::cout << m20::joint_names[index] << " -> " << wheel_goal_
                      << " rad/s (held until changed or stopped)\n";
        } else {
            const double requested = value * radians_per_degree + (verb == "jog" ? goal_[index] : 0.0);
            const auto &limit = joint_limits[index];
            const double lower = limit.lower + position_margin;
            const double upper = limit.upper - position_margin;
            const double target = std::clamp(requested, lower, upper);
            if (target != requested) {
                std::cout << "Warning: " << m20::joint_names[index] << " requested "
                          << requested / radians_per_degree << " deg; clamped to " << target / radians_per_degree
                          << " deg (allowed " << lower / radians_per_degree << " to "
                          << upper / radians_per_degree << " deg).\n";
            }
            goal_[index] = static_cast<float>(target);
            std::cout << m20::joint_names[index] << " -> " << target / radians_per_degree << " deg (URDF)\n";
        }
        selected_ = index;
    }

private:
    void read_joints(const JointData &message) {
        bool valid = true;
        for (std::size_t index = 0; index < position_.size(); ++index) {
            const auto &joint = message.data.joints_data[index];
            valid = valid && std::isfinite(joint.position) && std::isfinite(joint.velocity) &&
                    joint.status_word == 1;
        }
        if (valid) {
            for (std::size_t index = 0; index < position_.size(); ++index) {
                position_[index] = message.data.joints_data[index].position;
                velocity_[index] = message.data.joints_data[index].velocity;
            }
        }
        joint_clock_.update(nanoseconds(message.header.stamp), valid);
    }

    void read_imu(const Imu &message) {
        const bool valid = message.angular_velocity_covariance[0] != -1.0 &&
            message.linear_acceleration_covariance[0] != -1.0 &&
            finite_vector(message.angular_velocity) && finite_vector(message.linear_acceleration);
        imu_clock_.update(nanoseconds(message.header.stamp), valid);
    }

    bool feedback_ready(Clock::time_point now) const {
        return joint_clock_.ready(now) && imu_clock_.ready(now);
    }

    const char *joint_outside_urdf_limits() const {
        for (std::size_t index = 0; index < position_.size(); ++index) {
            if (!is_wheel(index) && (position_[index] < joint_limits[index].lower ||
                                     position_[index] > joint_limits[index].upper)) {
                return m20::joint_names[index];
            }
        }
        return nullptr;
    }

    void arm() {
        const auto now = Clock::now();
        if (mode_ == Mode::Observe) throw std::runtime_error("Read-only mode: no motor commands are sent.");
        if (phase_ != Phase::Idle) throw std::runtime_error("Already arming, armed or releasing.");
        if (!feedback_ready(now)) throw std::runtime_error("Need fresh joint/IMU feedback and normal driver status.");
        if (const auto *joint = joint_outside_urdf_limits()) {
            throw std::runtime_error(std::string(joint) + " position is outside the URDF mechanical limits.");
        }
        if (now - foreign_at_ < writer_quiet_time) throw std::runtime_error("Command stream is not quiet yet.");
        publisher_ = create_publisher<JointCommand>(cmd_topic_, rclcpp::QoS(1).reliable());
        own_gid_ = publisher_->get_gid();
        pairing_until_ = now + arm_timeout;
        phase_ = Phase::Pairing;
        std::cout << "Arming: waiting for a command receiver and checking feedback.\n";
    }

    void stop_motion() {
        if (phase_ != Phase::Armed) throw std::runtime_error("Not armed.");
        wheel_goal_ = 0.0F;
        wheel_sent_ = 0.0F;
        selected_.reset();
        goal_ = sent_ = position_;
        std::cout << "Wheel targets zero; holding current leg angles.\n";
    }

    void release(const std::string &reason) {
        if (phase_ == Phase::Pairing) {
            drop_output(reason);
        } else if (phase_ == Phase::Armed) {
            phase_ = Phase::Releasing;
            release_until_ = Clock::now() + release_duration;
            std::cout << reason << ": sending brief damping, then stopping output.\n" << std::flush;
        }
    }

    void drop_output(const std::string &reason) {
        publisher_.reset();
        phase_ = Phase::Idle;
        std::cout << reason << '\n' << std::flush;
    }

    void tick() {
        if (phase_ == Phase::Idle) return;
        const auto now = Clock::now();
        if (phase_ == Phase::Pairing) {
            if (!feedback_ready(now) || now >= pairing_until_) {
                drop_output("Arm cancelled: feedback or command receiver unavailable.");
                return;
            }
            if (publisher_->get_subscription_count() <= 1) return;
            if (std::any_of(velocity_.begin(), velocity_.end(), [](float speed) { return std::abs(speed) > 0.2F; })) {
                drop_output("Arm cancelled: motors are not stationary.");
                return;
            }
            if (const auto *joint = joint_outside_urdf_limits()) {
                drop_output(std::string("Arm cancelled: ") + joint + " position is outside the URDF mechanical limits.");
                return;
            }
            goal_ = sent_ = position_;
            selected_.reset();
            wheel_goal_ = wheel_sent_ = 0.0F;
            phase_ = Phase::Armed;
            std::cout << "Armed: holding current leg angles; wheel targets zero.\n" << std::flush;
        }
        if (publisher_->get_subscription_count() <= 1) {
            drop_output("Command receiver lost; output stopped.");
            return;
        }
        if (phase_ == Phase::Armed) {
            if (!feedback_ready(now)) {
                release("Joint or IMU feedback is stale or invalid");
            } else if (const auto *joint = joint_outside_urdf_limits()) {
                release(std::string(joint) + " position is outside the URDF mechanical limits");
            } else {
                for (std::size_t index = 0; index < position_.size(); ++index) {
                    const float speed_limit = is_wheel(index) ? max_wheel_feedback_speed : max_leg_feedback_speed;
                    if (std::abs(velocity_[index]) > speed_limit) {
                        release(std::string(m20::joint_names[index]) + " exceeded the measured speed limit");
                        break;
                    }
                }
            }
        }
        if (phase_ == Phase::Releasing && now >= release_until_) {
            drop_output("Output stopped.");
            return;
        }
        publish_frame();
    }

    void publish_frame() {
        const bool armed = phase_ == Phase::Armed;
        JointCommand frame;
        frame.header.stamp = get_clock()->now();
        wheel_sent_ += std::clamp(wheel_goal_ - wheel_sent_, -wheel_step, wheel_step);
        for (std::size_t index = 0; index < sent_.size(); ++index) {
            auto &joint = frame.data.joints_data[index];
            const bool wheel = is_wheel(index);
            sent_[index] += std::clamp(goal_[index] - sent_[index], -leg_step, leg_step);
            joint.position = armed && !wheel ? sent_[index] : position_[index];
            joint.velocity = armed && wheel && selected_ == index ? wheel_sent_ : 0.0F;
            joint.kp = armed && !wheel ? leg_kp : 0.0F;
            joint.kd = armed && !wheel ? leg_kd : damping_kd;
            joint.control_word = dds::kIndexMotorControl;
        }
        publisher_->publish(frame);
    }

    void print_limits() const {
        std::cout << "Leg joint         URDF min/max (deg)       allowed target (deg)\n"
                  << std::fixed << std::setprecision(3);
        for (std::size_t index = 0; index < joint_limits.size(); ++index) {
            if (is_wheel(index)) continue;
            const auto &limit = joint_limits[index];
            std::cout << std::left << std::setw(16) << m20::joint_names[index]
                      << std::right << std::setw(10) << limit.lower / radians_per_degree
                      << std::setw(10) << limit.upper / radians_per_degree << "    "
                      << std::setw(10) << (limit.lower + position_margin) / radians_per_degree
                      << std::setw(10) << (limit.upper - position_margin) / radians_per_degree << '\n';
        }
        std::cout << "Wheel targets: -2 to +2 rad/s\n";
    }

    void print_status() {
        const auto now = Clock::now();
        const auto names = std::array<const char *, 4>{"idle", "arming", "armed", "releasing"};
        std::cout << names[static_cast<std::size_t>(phase_)]
                  << " | joints=" << (joint_clock_.ready(now) ? "live" : "not ready")
                  << " | IMU=" << (imu_clock_.ready(now) ? "live" : "not ready")
                  << " | other command traffic=" << (now - foreign_at_ < writer_quiet_time ? "recent/observing" : "quiet")
                  << " | discoverable command publishers=" << count_publishers(cmd_topic_) << '\n';
        if (!joint_clock_.ready(now)) return;
        std::cout << "motor             angle(deg)   velocity(rad/s)      goal (deg or rad/s)\n";
        for (std::size_t index = 0; index < position_.size(); ++index) {
            const bool wheel = is_wheel(index);
            std::cout << std::fixed << std::setprecision(3) << std::left << std::setw(15) << m20::joint_names[index]
                      << std::right << std::setw(13) << position_[index] / radians_per_degree
                      << std::setw(17) << velocity_[index];
            if (phase_ == Phase::Armed) {
                const float goal = wheel ? (selected_ == index ? wheel_goal_ : 0.0F) : goal_[index] / radians_per_degree;
                std::cout << std::setw(10) << goal << (wheel ? " rad/s" : " deg");
            }
            std::cout << '\n';
        }
    }

    Mode mode_;
    Phase phase_ = Phase::Idle;
    const std::string prefix_, cmd_topic_;
    const Clock::time_point started_;
    Clock::time_point foreign_at_, pairing_until_, release_until_;
    FeedbackClock joint_clock_, imu_clock_;
    JointValues position_{}, velocity_{}, goal_{}, sent_{};
    std::optional<std::size_t> selected_;
    std::optional<rmw_gid_t> own_gid_;
    float wheel_goal_ = 0.0F;
    float wheel_sent_ = 0.0F;
    bool closing_ = false;
    rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
    rclcpp::Subscription<JointData>::SharedPtr joints_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<JointCommand>::SharedPtr command_monitor_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void read_console(JointBench &node, std::string &pending, bool tty) {
    if (pending.find('\n') == std::string::npos) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 0);
        if (ready < 0 && errno != EINTR) node.request_quit();
        if (ready <= 0) return;
        if (input.revents & (POLLIN | POLLHUP)) {
            char buffer[256];
            const auto received = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (received == 0 || (received < 0 && errno != EINTR && errno != EAGAIN)) node.request_quit();
            if (received > 0) pending.append(buffer, static_cast<std::size_t>(received));
        } else if (input.revents & (POLLERR | POLLNVAL)) {
            node.request_quit();
        }
    }
    const auto end = pending.find('\n');
    if (pending.size() > 1024) {
        std::cerr << "Terminal input too long; quitting.\n";
        node.request_quit();
    }
    if (end == std::string::npos || node.closing()) return;
    try {
        node.command(pending.substr(0, end));
    } catch (const std::exception &error) {
        std::cout << "Rejected: " << error.what() << '\n';
    }
    pending.erase(0, end + 1);
    if (tty && !node.closing()) std::cout << "m20> ";
    std::cout << std::flush;
}

}

int main(int argc, char **argv) {
    Mode mode = Mode::Observe;
    if (argc > 1) {
        const std::string option = argv[1];
        if (argc == 2 && option == "--hardware") mode = Mode::Hardware;
        else if (argc == 2 && option == "--sim") mode = Mode::Simulation;
        else if (argc == 2 && option == "--test") mode = Mode::Test;
        else {
            std::cout << "Usage: m20_joint_bench [--sim [--headless] | --hardware | --test]\n" << help;
            return argc == 2 && (option == "-h" || option == "--help") ? 0 : 2;
        }
    }
    if (mode == Mode::Hardware && (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO))) {
        std::cerr << "Hardware mode requires an interactive terminal (use ssh -t if needed).\n";
        return 2;
    }
    rclcpp::init(0, nullptr);
    rclcpp::uninstall_signal_handlers();
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGHUP, on_signal);
    std::signal(SIGPIPE, on_signal);
    int result = 0;
    try {
        auto node = std::make_shared<JointBench>(mode);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        const bool tty = isatty(STDIN_FILENO);
        std::cout << mode_name(mode) << ": disarmed. Type status or help.\n";
        if (tty) std::cout << "m20> ";
        std::cout << std::flush;
        std::string pending;
        while (rclcpp::ok() && !node->done()) {
            if (interrupted) node->request_quit();
            executor.spin_once(period);
            if (!node->closing()) read_console(*node, pending, tty);
        }
    } catch (const std::exception &error) {
        std::cerr << "Bench exited: " << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
