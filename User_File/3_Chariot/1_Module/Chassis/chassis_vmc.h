#ifndef CHASSIS_VMC_H
#define CHASSIS_VMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"

#include <stdint.h>

typedef struct
{
    float phi1_rad;              /* 前主动杆几何角，单位 rad。 */
    float phi2_rad;              /* 前从动杆几何角，单位 rad。 */
    float phi3_rad;              /* 后从动杆几何角，单位 rad。 */
    float phi4_rad;              /* 后主动杆几何角，单位 rad。 */
    float length_m;              /* 髋轴中心到轮轴C点的虚拟腿长，单位 m。 */
    float phi0_rad;              /* atan2 主值，范围 [-pi, pi]。 */
    float phi0_total_rad;        /* 底盘状态层连续展开后的虚拟腿角。 */
    float length_speed_mps;      /* 虚拟腿伸缩速度，单位 m/s。 */
    float phi0_speed_radps;      /* 虚拟腿摆角速度，单位 rad/s。 */
} chassis_vmc_state_t;

/* VMC虚拟支撑力和摆力矩映射后的两个主动关节力矩。 */
typedef struct
{
    float front_nm;              /* 前髋电机物理力矩，单位 N*m。 */
    float back_nm;               /* 后髋电机物理力矩，单位 N*m。 */
} chassis_vmc_torque_t;

/* 五连杆逆运动学给出的连续等价主动关节目标角。 */
typedef struct
{
    float phi1_rad;
    float phi4_rad;
} chassis_vmc_joint_target_t;

/**
 * @brief 由前后髋关节反馈计算五连杆虚拟腿状态。
 *
 * 输出包含主动/从动杆角、虚拟腿长和速度；几何无解或奇异时保持全零。
 */
void VMC_CalcState(const chassis_leg_config_t *config,
                    float front_position_rad,
                    float back_position_rad,
                    float front_speed_radps,
                    float back_speed_radps,
                    chassis_vmc_state_t *leg);

/**
 * @brief 由目标腿长和腿角计算当前装配分支下的前后主动关节目标角。
 *
 * 返回 0 表示目标超出五连杆工作空间或输入无效，返回 1 表示逆解有效。
 */
uint8_t VMC_CalcJointTarget(const chassis_leg_config_t *config,
                              const chassis_vmc_state_t *current_leg,
                              float target_length_m,
                              float target_phi0_rad,
                              chassis_vmc_joint_target_t *target);

/**
 * @brief 将虚拟支撑力和腿摆力矩映射为前后髋关节力矩。
 *
 * 映射采用虚功关系，几何奇异或腿长为零时输出保持零。
 */
void VMC_CalcTorque(const chassis_leg_config_t *config,
                     const chassis_vmc_state_t *leg,
                     float support_force_n,
                     float swing_torque_nm,
                     chassis_vmc_torque_t *torque);

#ifdef __cplusplus
}
#endif

#endif
