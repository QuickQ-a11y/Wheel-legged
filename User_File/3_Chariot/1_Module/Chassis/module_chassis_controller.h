#ifndef MODULE_CHASSIS_CONTROLLER_H
#define MODULE_CHASSIS_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"
#include "module_chassis.h"
#include "module_chassis_leg.h"
#include "module_chassis_model.h"

#include <stdint.h>

typedef struct
{
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT];
    float controlState[MODULE_CHASSIS_CONTROL_STATE_COUNT];
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT];
    float supportForcesN[MODULE_CHASSIS_LEG_COUNT];
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT];
    float forwardVelocityMps;
    uint8_t isStateValid;
} module_chassis_controller_debug_t;

/**
 * @brief 初始化底盘控制器内部状态。
 */
void Module_Chassis_Controller_Init(const module_chassis_model_config_t *config);

/**
 * @brief 计算轮腿控制器中间状态和安全受控输出。
 *
 * 第一阶段默认只允许计算，不打开非零电机输出。
 */
app_status_t Module_Chassis_Controller_Update(const module_chassis_model_config_t *config,
                                              const module_chassis_input_t *input,
                                              module_chassis_output_t *output);

/**
 * @brief 读取最近一次控制器调试状态快照。
 */
app_status_t Module_Chassis_Controller_GetDebug(module_chassis_controller_debug_t *debug);

#ifdef __cplusplus
}
#endif

#endif
