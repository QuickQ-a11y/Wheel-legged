#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "PID.h"

#include <stdint.h>

/**
 * @brief 单个云台轴的反馈口径、标定和控制参数。
 *
 * 角度和角速度反馈都取自 IMU（板子装在云台头部，直接测世界系姿态）；
 * 电机编码器只用于机械限位、YAW 投影角和 Watch 诊断，不进控制环。
 * 方向反了只改这里的 *_scale 符号，不动 gimbal_control.c。
 */
typedef struct
{
    /* 反馈口径 */
    uint8_t gyro_index;  /* 速度环取 filteredGyroRadps 哪一轴：1=Y(pitch)、2=Z(yaw)。 */
    /*
     * IMU 角度与角速度反馈相对本轴正方向的符号，+1 或 -1。
     *
     * 角度和角速度共用一个符号不是简化：task_imu.c 的整车系转换里
     * pitchRad 和 filteredGyroRadps[1] 是一起取反的，yaw 侧两者都不取反，
     * 所以两者极性天生绑定。拆成两个字段填成异号时，角度环把 d_angle
     * 当阻尼项用（derivative = -feedbackRate），阻尼会直接变成正反馈。
     */
    float imu_scale;

    /* 目标生成：杆量是角速度语义，积分成角度目标 */
    float stick_rate;    /* 满杆对应的角速度，rad/s；带符号，决定推杆方向与转向的对应。 */
    float follow_limit;  /* 目标相对反馈的最大超前量，rad；限住积分绕死，必须为正。 */
    float angle_min;     /* 角度目标下限，rad。min >= max 表示该轴不限位。 */
    float angle_max;

    /* 输出 */
    float T_limit;       /* 力矩限幅，N*m。 */

    /* 电机编码器口径 */
    float motor_scale;   /* 关节角正方向相对电机角正方向的符号。 */
    float motor_ratio;   /* 关节角/电机角传动比；位置、速度和力矩统一消费 scale*ratio。 */
    float motor_zero;    /* 机械零位对应的电机位置，rad。 */
    float joint_min;     /* 关节角机械限位下限，rad。min >= max 表示不限位。 */
    float joint_max;

    algorithm_pid_config_t angle_pid; /* 外环：角度误差 -> 目标角速度，rad -> rad/s。 */
    algorithm_pid_config_t speed_pid; /* 内环：角速度误差 -> 力矩，rad/s -> N*m。 */
} Gimbal_Axis_Config_t;

/** @brief 输出安全门。任一为 0 则对应通道恒零。 */
typedef struct
{
    uint8_t dm_flag;   /* 云台 DM 力矩输出使能。 */
    uint8_t dji_flag;  /* 发射机构 DJI 电流输出使能。 */
    /*
     * 板间下发使能。与上面两个不同：它不通任何电机，只往 FDCAN3 发遥控快照和
     * 云台朝向。关掉用于单独调试云台，或者排查总线负载时把它摘掉。
     * 底盘侧还有 Chassis_Config.follow.enable_flag 作为第二道开关。
     */
    uint8_t board_flag;
} Gimbal_Output_Config_t;

typedef struct
{
    Gimbal_Axis_Config_t yaw;
    Gimbal_Axis_Config_t pitch;
    Gimbal_Output_Config_t output;
    /*
     * 使能瞬间 PITCH 回水平的速率，rad/s。放顶层而不是逐轴配置：只有 PITCH 有
     * 绝对基准（地平面，IMU 直接给）能回中，YAW 没有，放进 Gimbal_Axis_Config_t
     * 会多出一个永远用不到的字段。
     */
    float pitch_home_rate;
    float default_dt;  /* 任务 tick 差越界时回退的周期，单位 s。 */
} Gimbal_Config_t;

extern const Gimbal_Config_t Gimbal_Config;

#ifdef __cplusplus
}
#endif

#endif
