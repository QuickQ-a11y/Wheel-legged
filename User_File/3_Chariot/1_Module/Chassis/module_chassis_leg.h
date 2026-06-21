#ifndef MODULE_CHASSIS_LEG_H
#define MODULE_CHASSIS_LEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"
#include "module_chassis_model.h"

typedef struct
{
    float phi1Rad;               /* 前侧主动杆几何角，单位 rad。 */
    float phi2Rad;               /* 前侧从动杆几何角，单位 rad。 */
    float phi3Rad;               /* 后侧从动杆几何角，单位 rad。 */
    float phi4Rad;               /* 后侧主动杆几何角，单位 rad。 */
    float legLengthM;            /* 虚拟腿长，单位 m。 */
    float phi0Rad;               /* 虚拟腿相对机体系几何角，单位 rad；控制腿角 theta = 竖直参考角 - phi0 + pitch。 */
    float legLengthVelocityMps;  /* 虚拟腿长变化速度，单位 m/s。 */
    float legSwingVelocityRadps; /* 虚拟腿摆角速度，等于 -d(phi0)/dt；正值对应 theta 增大，单位 rad/s。 */
} module_chassis_leg_state_t;

typedef struct
{
    float frontTorqueNm;         /* 前侧髋关节电机命令力矩，单位 N*m。 */
    float backTorqueNm;          /* 后侧髋关节电机命令力矩，单位 N*m。 */
} module_chassis_leg_joint_torque_t;

/**
 * @brief 根据两髋关节反馈计算五连杆腿部状态。
 */
app_status_t Module_Chassis_Leg_CalculateState(const module_chassis_leg_config_t *config,
                                               float frontPositionRad,
                                               float backPositionRad,
                                               float frontVelocityRadps,
                                               float backVelocityRadps,
                                               module_chassis_leg_state_t *state);

/**
 * @brief 将虚拟支撑力和腿摆力矩映射为两个髋关节力矩。
 *
 * swingTorqueNm 的正方向定义为使控制腿角 theta 增大。
 * 已在 Webots 中确认：theta > 0 时虚拟腿向机身后方倾斜，因此正腿摆力矩会驱动虚拟腿向机身后方摆动。
 */
app_status_t Module_Chassis_Leg_MapVirtualForce(const module_chassis_leg_config_t *config,
                                                const module_chassis_leg_state_t *state,
                                                float supportForceN,
                                                float swingTorqueNm,
                                                module_chassis_leg_joint_torque_t *jointTorque);

#ifdef __cplusplus
}
#endif

#endif
