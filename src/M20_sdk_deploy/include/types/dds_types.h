#include "common_types.h"

namespace dds{
    // 电池信息
    typedef struct{
        uint16_t voltage;             // 总电压，单位10mV
        int16_t  current;             // 总电流，单位10mA
        uint16_t remaining_capacity;  // 剩余容量，单位10mAH
        uint16_t nominal_capacity;    // 标称容量，单位10mAH
        uint16_t cycles;              // 循环次数
        uint16_t production_date;     // 生产日期
        uint16_t balanced_low;        // TODO：均衡低
        uint16_t balanced_high;       // TODO：均衡高
        uint16_t protected_state;     // 保护状态

        uint8_t software_version;  // 软件版本
        uint8_t battery_level;     // 剩余容量百分比
        uint8_t mos_state;         // Mos状态
        uint8_t battery_quantity;  // 电池串数
        uint8_t battery_ntc;       // NTC个数

        float battery_temperature[4];   // 电池温度，单位摄氏度
    }BatteryInfo_dds;

    // 关节信息
    typedef struct {
        float pos, vel, tor, kp, kd;    // 关节位置、速度、力矩、Kp、Kd
        float motor_temperature, driver_temperature;  // 电机温度、驱动器温度
        uint16_t status_word, control_word, data_id;  // 状态字、控制字、数据ID
    } JointData_dds;

    // IMU信息
    struct ImuData_dds{
        float roll, pitch, yaw;             // rpy
        float omega_x, omega_y, omega_z;    // 角速度
        float acc_x, acc_y, acc_z;          // 加速度
        uint32_t timestamp;                 // 时间戳
    };

    // 命令索引
    enum CommandIndex{
        kIndexDisable = 1,                  // 失能
        kIndexEnable = 2,                   // 使能
        kIndexCalibration = 3,              // 校准
        //    kIndexOutEncoderCalibration = 4,
        kIndexMotorControl = 4,             // 电机控制
        kIndexMotorReset = 5,               // 电机复位
        kIndexSetHome = 6,                  // 设置零点
        kIndexSetGear = 7,                  // 设置齿轮比
        kIndexSetID = 8,                    // 设置ID
        kIndexSetCanTimeout = 9,            // 设置CAN超时
        kIndexSetBandwidth = 10,            // 设置带宽
        kIndexSetLimitCurrent = 11,         // 设置限制电流
        kIndexSetUnderVoltage = 12,         // 设置欠压保护
        kIndexSetOverVoltage = 13,          // 设置过压保护
        kIndexSetMotorTemperature = 14,     // 设置电机温度上限
        kIndexSetDriverTemperature = 15,    // 设置驱动器温度上限
        kIndexSaveConfig = 16,              // 保存配置
        kIndexErrorReset = 17,              // 错误复位
        kIndexWriteAppBackStart = 18,       // TODO：写入应用程序启动
        kIndexWriteAppBack = 19,            // TODO：写入应用程序
        kIndexCheckAppBack = 20,            // TODO：检查应用程序
        kIndexDfuStart = 21,                // TODO：DFU启动
        kIndexGetVersion = 22,              // 获取版本
        kIndexGetStatusWord = 23,           // 获取状态字
        kIndexGetConfig = 24,               // TODO：获取配置
        kIndexOutEncoderCalibration = 25,   // TODO：输出编码器校准
        kIndexGetOutEncoder = 26,           // TODO：获取输出编码器
        kIndexCalibReport = 31              // TODO：校准报告
    };
}
    