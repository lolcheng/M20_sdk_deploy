#pragma once

#include "common_types.h"
#include <atomic>
#include "rclcpp/rclcpp.hpp"
using namespace types;

namespace interface{
// 单组电池数据的字段数，字段顺序见 GetBatteryData。
const int BATTERY_DATA_SIZE=8;

// 统一关节和 IMU 的读取方式，具体机型负责启动、停止和下发指令。
class RobotInterface{
private:
public:
    /**
     * @brief 保存机型信息，初始化关节指令和 ROS 2 节点
     * @param robot_name 机器人名称
     * @param dof_num 受控关节的自由度数，用来确定指令矩阵的行数
     */
    RobotInterface(const std::string& robot_name, int dof_num):robot_name_(robot_name), dof_num_(dof_num){
        start_flag_ = true;
        joint_cmd_ = Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(dof_num_, 5);  // 初始指令全为 0
        node_ = std::make_shared<rclcpp::Node>("rl_deploy");

    }
    virtual ~RobotInterface(){};

    std::string robot_name_;  // 机器人名称
    const int dof_num_;  // 受控关节的自由度数
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;  // 关节指令缓存，列顺序见 SetJointCommand
    std::atomic<bool> start_flag_;  // 原子启动标志，构造时为 true
    rclcpp::Node::SharedPtr node_;  // 接口收发 ROS 2 消息用的 rl_deploy 节点

    // 返回 ROS 2 节点的共享指针，供调用方处理回调或创建话题。
    rclcpp::Node::SharedPtr get_node() {
        return node_;
    }
    /**
     * @brief 启动机器人接口，准备读取状态和发送控制指令
     */
    virtual void Start() = 0;

    /**
     * @brief 由具体机型处理停止控制时的操作
     */
    virtual void Stop() = 0;

    /**
     * @brief 读取关节反馈的时间戳
     * @return 最近一帧关节反馈的时间戳（秒）
     */
    virtual double GetInterfaceTimeStamp() = 0;

    /**
     * @brief 读取关节位置
     * @return 按关节顺序排列的角度（rad）
     */
    virtual VecXf GetJointPosition() = 0;

    /**
     * @brief 读取关节速度
     * @return 按关节顺序排列的角速度（rad/s）
     */
    virtual VecXf GetJointVelocity() = 0;

    /**
     * @brief 读取关节力矩
     * @return 按关节顺序排列的力矩（N·m）
     */
    virtual VecXf GetJointTorque() = 0;

    /**
     * @brief 读取机身姿态角
     * @return 依次为横滚角、俯仰角和偏航角（rad）
     */
    virtual Vec3f GetImuRpy() = 0;

    /**
     * @brief 读取机身加速度
     * @return 按 X、Y、Z 排列的加速度（m/s²）
     */
    virtual Vec3f GetImuAcc() = 0;

    /**
     * @brief 读取机身角速度
     * @return 机身坐标系下按 X、Y、Z 排列的角速度（rad/s）
     */
    virtual Vec3f GetImuOmega() = 0;

    /**
     * @brief 设置关节指令
     * @param input 每行对应一个关节，共 dof_num_ 行、5 列。五列依次为位置增益 kp、
     *              目标位置（rad）、速度增益 kd、目标速度（rad/s）、前馈力矩（N·m）。
     * @details 力矩 = kp × (目标位置 - 当前位置) + kd × (目标速度 - 当前速度) + 前馈力矩。
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;

    // 返回缓存的关节指令，行列格式与 SetJointCommand 的输入相同。
    virtual MatXf GetJointCommand(){
        return joint_cmd_;
    }

    /**
     * @brief 读取接触力
     * @return 接触力向量，通道顺序由具体机型约定
     * @todo 有力传感器的机型接入传感器反馈
     */
    virtual VecXf GetContactForce() = 0;


    /**
     * @brief 读取各关节的电机温度
     * @return 按关节顺序排列的温度，基类每项返回 30
     */
    virtual VecXf GetMotorTemperture(){
        return 30.*VecXf::Ones(dof_num_);
    }

    /**
     * @brief 读取各关节的驱动器温度
     * @return 按关节顺序排列的温度，基类每项返回 0
     */
    virtual VecXf GetDriverTemperture(){
        return VecXf::Zero(dof_num_);
    }

    /**
     * @brief 读取电池数据
     * @return 每组依次为电量百分比（0～100）、电压（mV）、电流（mA）、
     *         循环次数和 4 路温度（℃）。基类返回一组全 0。
     */
    virtual std::vector<uint16_t> GetBatteryData(){
        return std::vector<uint16_t>(BATTERY_DATA_SIZE, 0);
    }

    /**
     * @brief 读取 IMU 反馈的时间戳
     * @return 最近一帧 IMU 反馈的时间戳（秒），基类返回 0
     */
    virtual double GetImuTimestamp(){
        return 0.;
    }

    /**
     * @brief 读取各关节驱动器的状态字
     * @return 按关节顺序排列的状态字，基类每项返回 1，表示正常
     */
    virtual std::vector<uint16_t> GetDriverStatusWord(){
        return std::vector<uint16_t>(dof_num_, 1);
    }

    /**
     * @brief 读取各关节的反馈数据编号
     * @return 按关节顺序排列的数据编号，基类每项返回 0
     */
    virtual std::vector<uint16_t> GetJointDataID(){
        return std::vector<uint16_t>(dof_num_, 0);
    }

    // 供派生类按需主动刷新反馈，基类默认留空。
    virtual void RefreshRobotData(){
    }
};


};
