/**
 * @file motion_state_feedback.hpp
 * @brief motion state feedback 
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once
#include "common_types.h"

// 运动状态反馈
class MotionStateFeedback
{
private:
    uint8_t current_state_;     // 当前运动状态
    uint8_t current_gait_;      // 当前运动步态
    uint8_t last_state_;        // 上一运动状态
    uint8_t last_gait_;         // 上一运动步态

    Vec3f current_vel_;         // 当前速度
    Vec3f cmd_vel_;             // 命令速度
    Vec3f max_vel_;             // 最大速度

    union{
        struct{
            uint32_t joint_pos_limit : 1;   // TODO：关节位置限制
            uint32_t posture_limit : 1;     // TODO：姿态限制
            uint32_t reserved : 30;         // 保留位
        }error_code_bit_;                   // 错误码位域
        uint32_t error_code_;               // 错误码
    };
public:
    MotionStateFeedback(/* args */){
        memset(this, 0, sizeof(MotionStateFeedback));
        std::cout << "Motion State Feedback" << std::endl;
    }
    ~MotionStateFeedback(){

    }

    // 更新当前运动状态
    void UpdateCurrentState(int state){
        if(uint8_t(state) != current_state_){
            last_state_ = current_state_;
            current_state_ = uint8_t(state);
        }
    }

    // 更新当前运动步态
    void UpdateCurrentGait(uint8_t gait){
        if(gait != current_gait_){
            last_gait_ = current_gait_;
            current_gait_ = gait;
        }
    }

    // 清除运动错误码
    void ClearMotionError(){error_code_ = 0;}
    // 设置关节位置限制错误码
    void SetJointPosLimitError(){
        error_code_bit_.joint_pos_limit = 1;
    }
    // 设置姿态限制错误码
    void SetPostureLimitError(){
        error_code_bit_.posture_limit = 1;
    }

    // 外部调用函数
    inline uint8_t GetCurrentState(){return current_state_;}
    inline uint8_t GetCurrentGait(){return current_gait_;}
    inline uint8_t GetLastState(){return last_state_;}
    inline uint8_t GetLastGait(){return last_gait_;}
};
