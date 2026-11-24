#pragma once

#include "robot_interface.h"
#include "dds_interface.hpp"
#include "m20_joint_layout.hpp"

// M20 的 DDS 硬件接口，负责关节零位修正和指令下发
class M20Interface : public DdsInterface {
protected:
    /**
     * @brief 等关节和 IMU 反馈就绪，再修正关节零位
     * @throws std::runtime_error 等待反馈超时
     */
    void ResetPositionOffset() {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);  // 反馈等待期限
        // 循环处理订阅回调，等所有关节和 IMU 都有有效反馈
        while (rclcpp::ok() && (!IsDataUpdatedFinished() || !IsImuDataReady())) {
            rclcpp::spin_some(this->get_node());
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("No valid joint/IMU data received within 10 seconds");
            usleep(1000);
        }
        if (!rclcpp::ok()) throw std::runtime_error("Robot startup interrupted");
        VecXf current_joint_pos = this->GetJointPosition();  // SDK 坐标下的关节反馈角度（rad）

        // TODO：hipy 超出 ±140° 时，零位偏移加减一圈
        for (int i = 1; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        // TODO：knee 超出 ±164° 时，零位偏移加减一圈
        for (int i = 2; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }

        // 更新下发用的方向和零位偏移
        for (int i = 0; i < dof_num_; ++i) {
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

public:
    /**
     * @brief 初始化 M20 的 16 路关节配置
     * @param robot_name 机器人名称，传给 DDS 基类
     */
    M20Interface(const std::string &robot_name) : DdsInterface(robot_name, 16) {
        battery_data_.resize(2 * BATTERY_DATA_SIZE);  // 电池数据

        // 初始零位偏移（度），每行一条腿，顺序为左前、右前、左后、右后
        // 每行依次为hipx,hip,knee,wheel
        float init_pos_offset[16] = {-25, -131, 160, 0.,
                                     25, -131, 160, 0,
                                     -25, 131, -160, 0,
                                     25, 131, -160, 0};
        // 按关节表设置方向系数
        for (int i = 0; i < dof_num_; ++i) {
            pos_offset_[i] = init_pos_offset[i];
            joint_dir_[i] = m20::joint_direction[i];
            data_updated_[i] = false;
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

    ~M20Interface() {
    }

    // 记录启动时间（ms），再修正零位。
    virtual void Start() {
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    // 停止接口
    virtual void Stop() {
    }

    /**
     * @brief 发布 M20 的关节控制命令
     * @param input 16×5 的关节指令矩阵，每行一个关节，行顺序见 m20::joint_names
     *              五列依次为位置增益 kp、目标位置（rad）、速度增益 kd、
     *              目标速度（rad/s）、前馈力矩（N·m）
     * @note 前馈力矩超过限值时，由底层固件或硬件截断
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        auto msg = drdds::msg::JointsDataCmd();  // 发往 /JOINTS_CMD 的整帧指令

        // 目标位置先减零位偏移再换方向，速度和力矩只换方向
        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position =
                    (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
            msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
            msg.data.joints_data[i].kp = input(i, 0);
            // 每组第 4 路是轮电机，位置增益置零，按目标速度和前馈力矩控制
            if (i % 4 == 3) {
                msg.data.joints_data[i].kp = 0;
            }
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        joint_cmd_pub_->publish(msg);

    }

};
