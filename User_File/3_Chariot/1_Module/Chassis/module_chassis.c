#include "module_chassis.h"

#include "module_chassis_controller.h"
#include "module_chassis_model.h"

#include <string.h>

static module_chassis_output_t chassisLastOutput;
static const module_chassis_model_config_t *chassisModelConfig;

/**
 * @brief 将输出结构清零并标记为安全输出。
 *
 * 故障和未使能状态下持续调用该函数，避免电机保留上一帧非零命令。
 */
static void Module_Chassis_SetZeroOutput(module_chassis_output_t *output)
{
    if (output == NULL)
    {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->safeOutput = 1U;
}

/**
 * @brief 根据输入状态生成故障标志。
 *
 * 当前检查使能、IMU、两类电机在线状态和 CAN 错误累计值。
 */
static uint32_t Module_Chassis_CheckFaults(const module_chassis_input_t *input)
{
    uint32_t faultFlags = MODULE_CHASSIS_FAULT_NONE;
    uint32_t index;

    if (input->isEnabled == 0U)
    {
        faultFlags |= MODULE_CHASSIS_FAULT_DISABLED;
    }

    if ((input->imu.isInitialized == 0U) ||
        (input->imu.isAttitudeReady == 0U) ||
        (input->imu.errorCode != 0U))
    {
        faultFlags |= MODULE_CHASSIS_FAULT_IMU;
    }

    for (index = 0U; index < APP_CONFIG_DM_MOTOR_COUNT; index++)
    {
        if (input->dmMotors[index].isOnline == 0U)
        {
            faultFlags |= MODULE_CHASSIS_FAULT_DM_MOTOR;
            break;
        }
    }

    for (index = 0U; index < APP_CONFIG_DJI_WHEEL_COUNT; index++)
    {
        if (input->djiWheels[index].isOnline == 0U)
        {
            faultFlags |= MODULE_CHASSIS_FAULT_DJI_MOTOR;
            break;
        }
    }

    if (input->canTxErrorCount > APP_CONFIG_CAN_TX_ERROR_LIMIT)
    {
        faultFlags |= MODULE_CHASSIS_FAULT_CAN;
    }

    return faultFlags;
}

static uint8_t Module_Chassis_IsOutputAllowed(const module_chassis_model_config_t *config)
{
    uint8_t jointOutputAllowed;
    uint8_t wheelOutputAllowed;

    if ((config == NULL) || (APP_CONFIG_CHASSIS_CONTROLLER_OUTPUT_ENABLE == 0U))
    {
        return 0U;
    }

    jointOutputAllowed =
        ((config->output.jointTorqueOutputEnabled != 0U) &&
         (config->output.jointTorqueLimitNm > 0.0f)) ? 1U : 0U;
    wheelOutputAllowed =
        ((config->output.wheelCurrentOutputEnabled != 0U) &&
         (config->wheel.torqueLimitNm > 0.0f) &&
         (config->wheel.torqueToCurrentRaw != 0.0f) &&
         (config->wheel.currentLimitRaw > 0)) ? 1U : 0U;

    return ((jointOutputAllowed != 0U) || (wheelOutputAllowed != 0U)) ? 1U : 0U;
}

void Module_Chassis_Init(void)
{
    chassisModelConfig = Module_Chassis_Model_GetDefaultConfig();
    Module_Chassis_Controller_Init(chassisModelConfig);
    Module_Chassis_SetZeroOutput(&chassisLastOutput);
    chassisLastOutput.faultFlags = MODULE_CHASSIS_FAULT_DISABLED;
}

app_status_t Module_Chassis_Update(const module_chassis_input_t *input,
                                module_chassis_output_t *output)
{
    uint32_t faultFlags;

    if ((input == NULL) || (output == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    Module_Chassis_SetZeroOutput(output);
    faultFlags = Module_Chassis_CheckFaults(input);
    output->faultFlags = faultFlags;

    if ((faultFlags == MODULE_CHASSIS_FAULT_NONE) && (chassisModelConfig != NULL))
    {
        app_status_t controllerStatus =
            Module_Chassis_Controller_Update(chassisModelConfig, input, output);

        if (controllerStatus != APP_STATUS_OK)
        {
            Module_Chassis_SetZeroOutput(output);
            output->faultFlags = MODULE_CHASSIS_FAULT_CONTROLLER;
        }
    }

    /*
     * 控制链已经参与计算，但默认编译配置和模型配置均不允许非零输出。
     * 后续实机调试必须同时打开编译宏、模型输出开关和限幅参数。
     */
    if (Module_Chassis_IsOutputAllowed(chassisModelConfig) == 0U)
    {
        output->safeOutput = 1U;
    }
    else
    {
        output->safeOutput = (output->faultFlags == MODULE_CHASSIS_FAULT_NONE) ? 0U : 1U;
    }

    chassisLastOutput = *output;

    return APP_STATUS_OK;
}
