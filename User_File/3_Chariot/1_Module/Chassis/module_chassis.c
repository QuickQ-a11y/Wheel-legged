#include "module_chassis.h"

#include "module_chassis_controller.h"
#include "module_chassis_model.h"

#include <string.h>

static const module_chassis_model_config_t *chassisModelConfig;

/**
 * @brief 将输出结构清零并标记为安全输出。
 *
 * 故障和未使能状态下持续调用该函数，避免电机保留上一帧非零命令。
 */
static void Module_Chassis_SetZeroOutput(module_chassis_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->safeOutput = 1U;
}

void Module_Chassis_Init(void)
{
    chassisModelConfig = Module_Chassis_Model_GetDefaultConfig();
    Module_Chassis_Controller_Init(chassisModelConfig);
}

void Module_Chassis_ResetMotionState(void)
{
    if (chassisModelConfig != NULL)
    {
        Module_Chassis_Controller_ResetMotionState(chassisModelConfig);
    }
}

void Module_Chassis_RunControl(const module_chassis_input_t *input,
                               module_chassis_output_t *output)
{
    uint32_t faultFlags;
    uint8_t jointOutputAllowed;
    uint8_t wheelOutputAllowed;
    uint32_t index;

    Module_Chassis_SetZeroOutput(output);

    /*
     * 故障位直接在主流程里生成，避免再包一层 Check 函数。
     * 设备异常时不会进入控制器，输出继续保持安全零输出。
     */
    faultFlags = MODULE_CHASSIS_FAULT_NONE;
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
    output->faultFlags = faultFlags;

    if ((faultFlags == MODULE_CHASSIS_FAULT_NONE) && (chassisModelConfig != NULL))
    {
        Module_Chassis_Controller_RunControlLoop(chassisModelConfig, input, output);
    }
    else if (chassisModelConfig != NULL)
    {
        Module_Chassis_Controller_ResetMotionState(chassisModelConfig);
    }

    /*
     * 控制链已经参与计算，但默认编译配置和模型配置均不允许非零输出。
     * 后续实机调试必须同时打开编译宏、模型输出开关和限幅参数。
     */
    jointOutputAllowed =
        ((chassisModelConfig != NULL) &&
         (APP_CONFIG_CHASSIS_CONTROLLER_OUTPUT_ENABLE != 0U) &&
         (chassisModelConfig->output.jointTorqueOutputEnabled != 0U) &&
         (chassisModelConfig->output.jointTorqueLimitNm > 0.0f)) ? 1U : 0U;
    wheelOutputAllowed =
        ((chassisModelConfig != NULL) &&
         (APP_CONFIG_CHASSIS_CONTROLLER_OUTPUT_ENABLE != 0U) &&
         (chassisModelConfig->output.wheelCurrentOutputEnabled != 0U) &&
         (chassisModelConfig->wheel.torqueLimitNm > 0.0f) &&
         (chassisModelConfig->wheel.torqueToCurrentRaw != 0.0f) &&
         (chassisModelConfig->wheel.currentLimitRaw > 0)) ? 1U : 0U;

    if ((jointOutputAllowed == 0U) && (wheelOutputAllowed == 0U))
    {
        output->safeOutput = 1U;
    }
    else
    {
        output->safeOutput = (output->faultFlags == MODULE_CHASSIS_FAULT_NONE) ? 0U : 1U;
    }
}
