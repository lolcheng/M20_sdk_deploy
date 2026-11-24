#pragma once

#include "common_types.h"

namespace types{
    enum RobotName{     // 机器人名称
        M20 = 11,
    };

    enum RobotType{     // 机器人类型
        QuadrupedWheel = 1,
    };

    enum RobotMotionState{      // 机器人运动状态
        WaitingForStand = 0,    // 等待站立
        StandingUp      = 1,    // 站立中
        JointDamping    = 2,    // 关节阻尼
        LieDown         = 4,    // 躺下
        RLControlMode   = 6,    // RL控制
    };

    enum StateName{             // 状态名称
        kInvalid      = -1,     // 无效状态
        kIdle         = 0,      // 空闲状态
        kStandUp      = 1,      // 站立状态
        kJointDamping = 2,      // 关节阻尼状态
        kLieDown      = 4,      // 躺下状态
        kRLControl    = 6,      // RL控制状态
    };

    enum RemoteCommandType{     // 远控命令方式
        kKeyBoard = 0,          // 键盘
        kGamepad = 1,           // 控制器
        kRos2 = 2,              // ROS2
    };
    
    enum KeyCode {              // 键盘按键码
        L1,                     // 左前
        L2,                     // 左后
        R1,                     // 右前
        R2,                     // 右后
        UNKNOWN                 // 未知
    };

    // 获取当前工作目录
    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};
