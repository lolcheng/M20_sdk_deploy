
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <csignal>
#include <pthread.h>


namespace types {
// 基础向量与矩阵
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec4f = Eigen::Vector4f;
using Vec4d = Eigen::Vector4d;
using VecXf = Eigen::VectorXf;
using VecXd = Eigen::VectorXd;

using Mat3f = Eigen::Matrix3f;
using Mat3d = Eigen::Matrix3d;
using MatXf = Eigen::MatrixXf;
using MatXd = Eigen::MatrixXd;

const float gravity = 9.815;

// 机器人状态
struct RobotBasicState {
    Vec3f base_rpy;             // 基座rpy
    Vec4f base_quat;            // 基座四元数
    Mat3f base_rot_mat;         // 基座旋转矩阵
    Vec3f base_omega;           // 基座角速度
    Vec3f base_acc;             // 基座加速度
    MatXf flt_base_acc_mat;     // TODO：浮动基座加速度矩阵，20*3
    VecXf joint_pos;            // 关节位置
    VecXf joint_vel;            // 关节速度
    VecXf joint_tau;            // 关节力矩

    RobotBasicState(int dof = 16) {
        base_rpy.setZero();
        base_quat.setZero();
        base_rot_mat.setIdentity();
        base_omega.setZero();
        base_acc.setZero();
        flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        joint_pos = VecXf::Zero(dof);
        joint_vel = VecXf::Zero(dof);
        joint_tau = VecXf::Zero(dof);
    }
};

// 机器人动作
struct RobotAction {
    VecXf goal_joint_pos;   // 目标关节位置
    VecXf goal_joint_vel;   // 目标关节速度
    VecXf kp;               // Kp
    VecXf kd;               // Kd
    VecXf tau_ff;           // 前馈力矩

    MatXf ConvertToMat() {  // 以矩阵形式发布动作
        MatXf res(goal_joint_pos.rows(), 5);
        res.col(0) = kp;
        res.col(1) = goal_joint_pos;
        res.col(2) = kd;
        res.col(3) = goal_joint_vel;
        res.col(4) = tau_ff;
        return res;
    }
};

// 用户命令
struct UserCommand {
    double time_stamp;              // 时间戳
    uint8_t safe_control_mode;      // TODO：安全控制模式：0 normal, 1 stand, 2 sit, 3 joint damping
    uint8_t target_mode;            // TODO：目标模式
    float forward_vel_scale = 0;    // 前进速度缩放
    float side_vel_scale = 0;       // 侧向速度缩放
    float turnning_vel_scale = 0;   // 转向速度缩放
    float reserved_scale;           // 预留缩放
};

// 网络信息
enum FromType {
    inside = 0, // 内部发送
    user,       // 用户发送
};
struct NetInfo {
    std::string ip; // IP地址
    int port;       // 端口
    FromType fromType;
};

// 网络数据
struct DataInfo : public NetInfo {  // 数据信息
    const char* buffer;     // TODO：数据缓冲区
    ssize_t len;            // 数据长度
};
struct CmdInfo : public NetInfo {   // 命令信息
    int type;               // 命令类型
    int command;            // 命令
};
};  // namespace types
