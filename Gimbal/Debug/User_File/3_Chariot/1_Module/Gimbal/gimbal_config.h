#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

/** @brief 单个云台轴的机械与电气标定。 */
typedef struct
{
    float scale;      /* 关节角正方向相对电机角正方向的符号，取 +1 或 -1。 */
    float ratio;      /* 关节角/电机角传动比；位置、速度、力矩统一消费 scale*ratio。 */
    float zero_rad;   /* 机械零位对应的电机位置，单位 rad。 */
    float angle_min;  /* 关节角下限，单位 rad。 */
    float angle_max;  /* 关节角上限，单位 rad。 */
    float T_limit;    /* 该轴力矩限幅，单位 N*m。 */
} Gimbal_Axis_Config_t;

/** @brief 输出安全门。任一为 0 则对应通道恒零。 */
typedef struct
{
    uint8_t dm_flag;   /* 云台 DM 力矩输出使能。 */
    uint8_t dji_flag;  /* 发射机构 DJI 电流输出使能。 */
} Gimbal_Output_Config_t;

typedef struct
{
    Gimbal_Axis_Config_t yaw;
    Gimbal_Axis_Config_t pitch;
    Gimbal_Output_Config_t output;
    float default_dt;  /* 任务 tick 差越界时回退的周期，单位 s。 */
} Gimbal_Config_t;

extern const Gimbal_Config_t Gimbal_Config;

#ifdef __cplusplus
}
#endif

#endif
