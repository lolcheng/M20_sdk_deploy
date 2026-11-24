/**
 * @file dds_interface.hpp
 * @brief hardware interface for dds
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#pragma once


#include "common_types.h"
#include "robot_interface.h"
#include <mutex>
#include "dds_types.h"

#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/battery_data.hpp"

using namespace dds;

// 下发关节命令时用的方向和零位
struct JointConfig {
    float dir;     // 电机方向相对 SDK 的符号，同向为 1，反向为 -1
    float offset;  // 零位偏移（rad）
};


// DDS 硬件接口，发布关节命令并缓存反馈
class DdsInterface : public RobotInterface{
protected:
    mutable std::mutex data_mutex_;
    double ri_ts_{0.}, imu_ts_{0.};   // 关节和 IMU 消息的时间戳（秒）
    // 机身角速度（rad/s）、横滚/俯仰/偏航角（rad）、加速度（m/s²）
    Vec3f omega_body_{Vec3f::Zero()}, rpy_{Vec3f::Zero()}, acc_{Vec3f::Zero()};
    VecXf joint_pos_, joint_vel_, joint_tau_;               // 各关节位置（rad）、速度（rad/s）、力矩（N·m）
    VecXf motor_temperture_, driver_temperture_;            // 各关节的电机温度和驱动器温度

    float *pos_offset_;        // 各关节的零位偏移（度）
    float *joint_dir_;         // 各关节的方向系数
    bool *data_updated_;      // 各关节的反馈更新标志
    uint16_t *control_word;   // 清故障时按顺序发送的四个控制字

    JointConfig *joint_config_;              // 下发命令用的关节配置
    std::vector<uint16_t> driver_status_;    // 各关节驱动器上报的状态字
    std::vector<uint16_t> joint_data_id_;    // 各关节的数据 ID

    BatteryInfo_dds battery_info_[2];       // 电池信息
    std::vector<uint16_t> battery_data_;    // 对外返回的电池数据，第 0 项为剩余电量百分比
    double time_stamp_;                     // 机型接口启动时记录的本地时间戳（ms）
    float remain_bat;                       // 电池的电量百分比

    // ROS 2 话题
    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr joint_cmd_pub_;     // /JOINTS_CMD，发送关节命令
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr joint_data_sub_;    // /JOINTS_DATA，接收关节反馈
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_data_sub_;        // /IMU_DATA，接收 IMU 反馈
    rclcpp::Subscription<drdds::msg::BatteryData>::SharedPtr health_data_sub_; // /BATTERY_DATA，接收电池反馈


    // 所有关节的更新标志都置位时返回 true
    bool IsDataUpdatedFinished() {
        bool res = data_updated_[0];
        for (int i = 0; i < dof_num_; ++i) {
            res = res & data_updated_[i];
        }
        return res;
    }

    // IMU 时间戳大于 0，且姿态角、角速度和加速度都是有限值时返回 true
    bool IsImuDataReady() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return imu_ts_ > 0. && rpy_.allFinite() && omega_body_.allFinite() && acc_.allFinite();
    }

    // 等待关节反馈，再用当前位置修正零位偏移
    void ResetPositionOffset() {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));
        usleep(100 * 1000);
        VecXf last_joint_pos = this->GetJointPosition();     // 上一轮读取的关节位置
        VecXf current_joint_pos = this->GetJointPosition();  // 本轮读取的关节位置
        int cnt = 0;                                         // 等待反馈的轮次

        // 处理待执行的订阅回调，再读取关节位置
        while (!IsDataUpdatedFinished()) {
            ++cnt;
            this->RefreshRobotData();                       // 尝试更新机器人状态
            rclcpp::spin_some(this->get_node());
            current_joint_pos = this->GetJointPosition();

            // 位置有变化也记为已更新
            for (int i = 0; i < dof_num_; ++i) {
                if (!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i)) {
                    data_updated_[i] = true;
                }
            }
            last_joint_pos = current_joint_pos;
            usleep(1000);
            // 等待达到第 10000 轮时打印更新失败提示，继续等反馈
            if (cnt == 10000) {
                std::cout << data_updated_ << std::endl;
                std::cout << "joint data update is not finished\n";
            }
        }
        // TODO：限制 hipy 的角度
        for (int i = 1; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -210. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 90. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        // 更新关节轴向和零位偏移
        for (int i = 0; i < dof_num_; ++i) {
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

public:
    int run_cnt_ = 0;  // 收到的关节反馈数

    /**
     * @brief 初始化反馈缓存和 DDS 话题，再发送驱动器复位命令
     * @param robot_name 机器人名称
     * @param dof_num 关节自由度
     */
    DdsInterface(const std::string &robot_name, const int &dof_num) : RobotInterface(robot_name, dof_num) {
        std::cout << robot_name << " is using Ecan Hardware Interface" << std::endl;
        // 各关节的运动和温度反馈
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperture_ = VecXf::Zero(dof_num_);
        driver_temperture_ = VecXf::Zero(dof_num_);

        // 各关节的配置和更新标志，方向与零位
        pos_offset_ = new float[dof_num_];
        joint_dir_ = new float[dof_num_];
        data_updated_ = new bool[dof_num_];
        joint_config_ = new JointConfig[dof_num_];
        control_word = new uint16_t[4];

        // 状态字，关节数据和电池数据
        driver_status_.resize(dof_num_);
        joint_data_id_.resize(dof_num_);
        battery_data_.resize(BATTERY_DATA_SIZE);

        // ROS2：关节和电池话题的队列深度为 10，IMU 使用传感器 QoS
        joint_cmd_pub_ = node_->create_publisher<drdds::msg::JointsDataCmd>("/JOINTS_CMD", 10);
        joint_data_sub_ = node_->create_subscription<drdds::msg::JointsData>("/JOINTS_DATA", 10,
                                    std::bind(&DdsInterface::Handler, this,std::placeholders::_1));
        imu_data_sub_ = node_->create_subscription<drdds::msg::ImuData>("/IMU_DATA", rclcpp::SensorDataQoS(),
                                    std::bind(&DdsInterface::HandlerIMU, this, std::placeholders::_1));
        health_data_sub_ = node_->create_subscription<drdds::msg::BatteryData>("/BATTERY_DATA", 10,
                                    std::bind(&DdsInterface::HandlerHealth, this, std::placeholders::_1));
        
        sleep(1);

        // 保存驱动器复位控制字
        control_word[0] = kIndexDisable;
        control_word[1] = kIndexErrorReset;
        control_word[2] = kIndexEnable;
        control_word[3] = kIndexGetStatusWord;
        ResetJointError();
    }

    virtual ~DdsInterface() {
        delete[] pos_offset_;
        delete[] joint_dir_;
        delete[] data_updated_;
        delete[] joint_config_;
    }

    // 启动时的零位处理
    virtual void Start() {
    }

    // 停止
    virtual void Stop() {
    }

    // 取最近一帧关节反馈的消息时间戳（秒）
    virtual double GetInterfaceTimeStamp() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return ri_ts_;
    }

    // 取各关节在 SDK 坐标下的位置（rad）
    virtual VecXf GetJointPosition() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return joint_pos_;
    }

    // 取各关节在 SDK 坐标下的速度（rad/s）
    virtual VecXf GetJointVelocity() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return joint_vel_;
    }

    // 取各关节在 SDK 坐标下的力矩（N·m）
    virtual VecXf GetJointTorque() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return joint_tau_;
    }

    // 取横滚、俯仰和偏航角（rad）
    virtual Vec3f GetImuRpy() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return rpy_;
    }

    // 取 IMU 的三轴加速度（m/s²）
    virtual Vec3f GetImuAcc() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return acc_;
    }

    // 取机身坐标系下的三轴角速度（rad/s）
    virtual Vec3f GetImuOmega() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return omega_body_;
    }

    // 接触力用四个零值占位
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }

    // 重置关节故障：依次发送失能、清故障、使能和读取状态字的命令
    virtual void ResetJointError() {
        auto msg = drdds::msg::JointsDataCmd();  // 下发给驱动器的命令消息

        // 每步给所有关节发同一个控制字，位置、速度、力矩和增益都置零
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < dof_num_; ++i) {
                msg.data.joints_data[i].position = 0;
                msg.data.joints_data[i].velocity = 0;
                msg.data.joints_data[i].torque = 0;
                msg.data.joints_data[i].kp = 0;
                msg.data.joints_data[i].kd = 0;
                msg.data.joints_data[i].control_word = control_word[j];
            }
            std::cout << "ResetJointError publish!! \n" << std::endl;
            joint_cmd_pub_->publish(msg);
            usleep(10 * 1000);
        }
    }

    /**
     * @brief 发布关节控制命令
     * @param input 关节指令矩阵，每行一个关节
     *              五列依次为位置增益 kp、目标位置（rad）、速度增益 kd、
     *              目标速度（rad/s）、前馈力矩（N·m）
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        joint_cmd_ = input;
        auto msg = drdds::msg::JointsDataCmd();  // 下发给驱动器的命令消息

        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position =
                    (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
            msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
            msg.data.joints_data[i].kp = input(i, 0);
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        std::cout << "SetJointCommand publish!! \n" << std::endl;
        joint_cmd_pub_->publish(msg);
    }

    // 取各关节的电机温度
    virtual VecXf GetMotorTemperture() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return motor_temperture_;
    }

    // 取各关节的驱动器温度
    virtual VecXf GetDriverTemperture() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return driver_temperture_;
    }

    // 取最近一帧 IMU 反馈的消息时间戳（秒）
    virtual double GetImuTimestamp() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return imu_ts_;
    }

    // 取电池数据，第 0 项为第一块电池的剩余电量百分比
    virtual std::vector<uint16_t> GetBatteryData() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return battery_data_;
    }

    // 按关节顺序返回驱动器状态字
    virtual std::vector<uint16_t> GetDriverStatusWord() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return driver_status_;
    }

    // 按关节顺序返回反馈数据 ID
    virtual std::vector<uint16_t> GetJointDataID() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return joint_data_id_;
    }

    // 反馈数据由订阅回调更新
    virtual void RefreshRobotData() {
    }

    /**
     * @brief 更新关节反馈
     * @param msg 收到的关节反馈，含各轴的位置、速度、力矩、温度及状态字
     */
    virtual void Handler(const drdds::msg::JointsData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ++run_cnt_;
        // 将电机反馈换算到 SDK 的关节方向和零位
        for (int i = 0; i < dof_num_; ++i) {
            joint_pos_(i) = msg->data.joints_data[i].position * joint_dir_[i] + pos_offset_[i] / 180. * M_PI;
            joint_vel_(i) = msg->data.joints_data[i].velocity * joint_dir_[i];
            joint_tau_(i) = msg->data.joints_data[i].torque * joint_dir_[i];
            motor_temperture_(i) = float(msg->data.joints_data[i].motion_temp);
            driver_temperture_(i) = float(msg->data.joints_data[i].driver_temp);
            driver_status_[i] = msg->data.joints_data[i].status_word;
            joint_data_id_[i] = uint16_t(run_cnt_);  // 同一帧的关节共用一个数据 ID
        }
        ri_ts_ = rclcpp::Time(msg->header.stamp).seconds();
        // 位置和速度都是有限值时，才把该关节标为已更新
        for (int i = 0; i < dof_num_; ++i)
            data_updated_[i] = std::isfinite(joint_pos_(i)) && std::isfinite(joint_vel_(i));
    }

    /**
     * @brief 更新 IMU 反馈
     * @param msg 收到的 IMU 消息，含姿态角、加速度和角速度
     */
    void HandlerIMU(const drdds::msg::ImuData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 姿态角由度转为弧度，加速度和角速度直接保存
        rpy_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));
        acc_ << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        omega_body_ << msg->data.omega_x, msg->data.omega_y, msg->data.omega_z;
        imu_ts_ = rclcpp::Time(msg->header.stamp).seconds();
    }

    /**
     * @brief TODO：更新第一块电池的剩余电量
     * @param msg 收到的电池反馈，data[0] 对应第一块电池
     */
    void HandlerHealth(const drdds::msg::BatteryData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        battery_data_[0] = uint8_t(msg->data[0].battery_level);
        remain_bat = battery_data_[0] / 100;
        battery_info_[0].battery_level = msg->data[0].battery_level;
    }
};
