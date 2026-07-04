#ifndef MODULE_CHASSIS_CONTROLLER_H
#define MODULE_CHASSIS_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module_chassis.h"
#include "module_chassis_leg.h"
#include "module_chassis_model.h"

#include <stdint.h>

typedef struct
{
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT]; /* 左右虚拟腿几何状态快照。 */
    float controlState[MODULE_CHASSIS_CONTROL_STATE_COUNT];         /* 本轮 LQR 状态向量，顺序见 module_chassis_control_state_index_t。 */
    float lqrK[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT]; /* 本轮实际用于反馈计算的 K 矩阵。 */
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT];        /* LQR 广义输出，顺序见 module_chassis_control_output_index_t。 */
    float supportForcesN[MODULE_CHASSIS_LEG_COUNT];                 /* 左右腿虚拟支撑力，单位 N。 */
    float lqrInputLegLengthM[MODULE_CHASSIS_LEG_COUNT];             /* 送入 LQR 拟合前的左右腿长，单位 m。 */
    float lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_COUNT];           /* 限制到拟合范围后的左右腿长，单位 m。 */
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT];    /* 左右轮角速度，单位 rad/s。 */
    float rawForwardVelocityMps;                                    /* 轮速和腿部几何计算得到的原始前进速度，单位 m/s。 */
    float forwardVelocityMps;                                       /* 卡尔曼融合后的当前前进速度，单位 m/s。 */
    float forwardAccelerationMps2;                                  /* IMU 前向运动加速度测量值，单位 m/s^2。 */
    float fusedForwardAccelerationMps2;                             /* 卡尔曼融合后的前向加速度状态，单位 m/s^2。 */
    float forwardPositionM;                                         /* 由融合速度积分得到的前进位移状态，单位 m。 */
    uint8_t isLqrKFitEnabled;                                       /* 本轮是否使用 LQR 腿长拟合 lqrK。 */
    uint8_t isLqrKLengthLimited;                                    /* 本轮 lqrK 腿长输入是否被限制到拟合范围。 */
    uint8_t isStateValid;                                           /* 最近一次控制计算是否完整成功。 */
} module_chassis_controller_debug_t;

extern module_chassis_controller_debug_t chassisControllerDebug;

/**
 * @brief 初始化底盘控制器内部状态。
 */
void Module_Chassis_Controller_Init(const module_chassis_model_config_t *config);

/**
 * @brief 清空底盘运动融合状态。
 *
 * 零力矩、故障或重新初始化时调用，避免恢复控制后沿用旧的速度和位移积分。
 */
void Module_Chassis_Controller_ResetMotionState(const module_chassis_model_config_t *config);

/**
 * @brief 执行一轮底盘控制环。
 *
 * 按反馈状态、支撑力、K 矩阵、LQR 输出、VMC 和电机命令的顺序生成本轮输出。
 * 第一阶段默认不打开非零电机输出。
 */
void Module_Chassis_Controller_RunControlLoop(const module_chassis_model_config_t *config,
                                              const module_chassis_input_t *input,
                                              module_chassis_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
