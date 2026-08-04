#ifndef CHASSIS_VMC_H
#define CHASSIS_VMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"

#include <stdint.h>

typedef struct
{
    float phi1;                  /* 后主动杆角，绕车体+Y增大，rad。 */
    float phi2;                  /* phi1侧从动杆角，rad。 */
    float phi3;                  /* phi4侧从动杆角，rad。 */
    float phi4;                  /* 前主动杆角，绕车体+Y增大，rad。 */
    float L0;                    /* 虚拟腿长，m。 */
    float phi0;                  /* 虚拟腿角主值；向下为pi/2，rad。 */
    float phi0_total;            /* 连续展开的虚拟腿角，rad。 */
    float d_L0;                  /* 虚拟腿伸缩速度，m/s。 */
    float d_phi0;                /* 虚拟腿摆角速度，rad/s。 */
    float theta;                 /* LQR 腿摆角，rad。 */
    float d_theta;               /* LQR 腿摆角速度，rad/s。 */
    float target_L0;             /* 当前控制目标腿长，m。 */
    float target_phi0;           /* 当前连续目标腿角，rad。 */
    float F0;                    /* 虚拟腿轴向力，N。 */
    float Tp;                    /* 虚拟腿摆矩，N*m。 */
    float K_L0_fit;              /* 限幅后实际拟合腿长，m。 */
    uint8_t valid_flag;          /* 本轮五连杆状态有数学定义。 */
} Chassis_Leg_t;

/* VMC虚拟支撑力和摆力矩映射后的两个主动关节力矩。 */
typedef struct
{
    float T1;                    /* phi1电机坐标力矩，N*m。 */
    float T4;                    /* phi4电机坐标力矩，N*m。 */
} VMC_Torque_t;

/* 由两个主动关节反馈力矩反解得到的虚拟腿广义力。 */
typedef struct
{
    float F0;
    float Tp;
} VMC_Force_t;

/* 五连杆逆运动学给出的连续等价主动关节目标角。 */
typedef struct
{
    float phi1;
    float phi4;
} VMC_Joint_Target_t;

/**
 * @brief 由phi1和phi4关节反馈计算五连杆虚拟腿状态。
 *
 * 只更新运动学字段；控制目标和广义力由上层拥有，不在此函数清零。
 */
void VMC_State_Calc(const Chassis_Leg_Config_t *config,
                    float phi1_motor_position_rad,
                    float phi4_motor_position_rad,
                    float phi1_motor_speed_radps,
                    float phi4_motor_speed_radps,
                    Chassis_Leg_t *leg);

/**
 * @brief 由目标腿长和腿角计算当前装配分支下的phi1和phi4目标角。
 *
 * 返回 0 表示目标超出五连杆工作空间或输入无效，返回 1 表示逆解有效。
 */
uint8_t VMC_Inverse_Calc(const Chassis_Leg_Config_t *config,
                         const Chassis_Leg_t *current_leg,
                         float target_L0,
                         float target_phi0,
                         VMC_Joint_Target_t *target);

/**
 * @brief 将虚拟支撑力和腿摆力矩映射为phi1和phi4电机力矩。
 *
 * 映射采用虚功关系；返回0时输出保持零，返回1时力矩有效。
 */
uint8_t VMC_Torque_Calc(const Chassis_Leg_Config_t *config,
                        const Chassis_Leg_t *leg,
                        float F0,
                        float Tp,
                        VMC_Torque_t *torque);

/**
 * @brief 将phi1和phi4电机反馈力矩反解为虚拟支撑力和腿摆力矩。
 */
uint8_t VMC_Force_Calc(const Chassis_Leg_Config_t *config,
                       const Chassis_Leg_t *leg,
                       float phi1_motor_torque_nm,
                       float phi4_motor_torque_nm,
                       VMC_Force_t *force);

#ifdef __cplusplus
}
#endif

#endif
