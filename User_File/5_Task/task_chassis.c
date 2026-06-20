#include "task_chassis.h"

#include "app_config.h"
#include "task_can.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "task_imu.h"
#include "module_chassis.h"
#include "task_can_dispatch.h"

#include "cmsis_os2.h"

#include <string.h>

static osThreadId_t chassisTaskHandle;
static uint8_t chassisEnable;
static volatile uint32_t chassisFaultFlags;

static const osThreadAttr_t chassisTaskAttributes = {
    .name = "ChassisTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 将 IMU 任务状态转换为底盘模块输入。
 */
static void Chassis_Task_FillImuInput(const task_imu_state_t *imuState,
                                      module_chassis_input_t *chassisInput)
{
    if ((imuState == NULL) || (chassisInput == NULL))
    {
        return;
    }

    chassisInput->imu.isInitialized = imuState->isInitialized;
    chassisInput->imu.isAttitudeReady = imuState->isAttitudeReady;
    chassisInput->imu.errorCode = imuState->lastErrorCode;
    chassisInput->imu.rollRad = imuState->rollRad;
    chassisInput->imu.pitchRad = imuState->pitchRad;
    chassisInput->imu.yawRad = imuState->yawRad;
    chassisInput->imu.gyroRadps[0] =
        imuState->bmi088Data.gyroRadps[0] - imuState->gyroBiasRadps[0];
    chassisInput->imu.gyroRadps[1] =
        imuState->bmi088Data.gyroRadps[1] - imuState->gyroBiasRadps[1];
    chassisInput->imu.gyroRadps[2] =
        imuState->bmi088Data.gyroRadps[2] - imuState->gyroBiasRadps[2];
    chassisInput->imu.dtSec = imuState->dtSec;
}

/**
 * @brief 将电机设备层状态转换为底盘模块输入。
 */
static void Chassis_Task_FillMotorInput(uint32_t nowTick,
                                        module_chassis_input_t *chassisInput)
{
    uint32_t index;

    if (chassisInput == NULL)
    {
        return;
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t dmState = {0};

        if (Motor_DM_GetState((motor_dm_index_t)index, &dmState) == APP_STATUS_OK)
        {
            chassisInput->dmMotors[index].isOnline =
                Motor_DM_IsOnline((motor_dm_index_t)index, nowTick);
            chassisInput->dmMotors[index].positionRad = dmState.positionRad;
            chassisInput->dmMotors[index].velocityRadps = dmState.velocityRadps;
            chassisInput->dmMotors[index].torqueNm = dmState.torqueNm;
        }
    }

    for (index = 0U; index < APP_CONFIG_DJI_WHEEL_COUNT; index++)
    {
        const motor_dji_t *wheel = &chassisDjiWheels[index];

        chassisInput->djiWheels[index].isOnline = Motor_DJI_IsOnline(wheel, nowTick);
        chassisInput->djiWheels[index].speedRpm = wheel->speedRpm;
        chassisInput->djiWheels[index].currentRaw = wheel->currentRaw;
    }
}

/**
 * @brief 收集底盘模块输入。
 */
static app_status_t Chassis_Task_BuildInput(module_chassis_input_t *chassisInput)
{
    task_imu_state_t imuState = {0};
    uint32_t nowTick = HAL_GetTick();

    if (chassisInput == NULL)
    {
        return APP_STATUS_INVALID_PARAM;
    }

    memset(chassisInput, 0, sizeof(*chassisInput));
    chassisInput->isEnabled = chassisEnable;
    chassisInput->canTxErrorCount = CAN_Task_GetTxErrorCount();

    if (IMU_Task_GetState(&imuState) == APP_STATUS_OK)
    {
        Chassis_Task_FillImuInput(&imuState, chassisInput);
    }

    Chassis_Task_FillMotorInput(nowTick, chassisInput);

    return APP_STATUS_OK;
}

/**
 * @brief 将底盘模块输出下发到电机设备层和 CAN 发送缓存。
 */
static void Chassis_Task_ApplyOutput(const module_chassis_output_t *chassisOutput)
{
    uint32_t index;

    if (chassisOutput == NULL)
    {
        return;
    }

    Motor_DM_SetSafe(chassisOutput->safeOutput);
    if (chassisOutput->safeOutput != 0U)
    {
        Motor_DM_ZeroAll();
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_command_t command = {
            .positionRad = chassisOutput->dmCommands[index].positionRad,
            .velocityRadps = chassisOutput->dmCommands[index].velocityRadps,
            .kp = chassisOutput->dmCommands[index].kp,
            .kd = chassisOutput->dmCommands[index].kd,
            .torqueNm = chassisOutput->dmCommands[index].torqueNm,
        };

        (void)Motor_DM_SetCommand((motor_dm_index_t)index, &command);
    }

    (void)CAN_Task_SetDjiCurrent(chassisOutput->djiCurrents);
    Motor_DM_UpdateTxFrames();
    (void)CAN_Task_RequestTx();
}

static void ChassisTask(void *argument)
{
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    Motor_DM_Init();
    Motor_DM_SetEnable(0U);
    Module_Chassis_Init();

    for (;;)
    {
        module_chassis_input_t chassisInput = {0};
        module_chassis_output_t chassisOutput = {0};

        if (Chassis_Task_BuildInput(&chassisInput) != APP_STATUS_OK)
        {
            memset(&chassisOutput, 0, sizeof(chassisOutput));
            chassisOutput.safeOutput = 1U;
            chassisOutput.faultFlags = MODULE_CHASSIS_FAULT_DISABLED;
        }
        else
        {
            (void)Module_Chassis_Update(&chassisInput, &chassisOutput);
        }

        chassisFaultFlags = chassisOutput.faultFlags;
        Chassis_Task_ApplyOutput(&chassisOutput);

        wakeTick += APP_CONFIG_CHASSIS_TASK_PERIOD_TICKS;
        (void)osDelayUntil(wakeTick);
    }
}

void Chassis_Task_Init(void)
{
    chassisEnable = 0U;
    chassisFaultFlags = MODULE_CHASSIS_FAULT_DISABLED;
    chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTaskAttributes);
}

void Chassis_Task_SetEnable(uint8_t enable)
{
    chassisEnable = (enable != 0U) ? 1U : 0U;
    Motor_DM_SetEnable(chassisEnable);
}

uint8_t Chassis_Task_IsEnabled(void)
{
    return chassisEnable;
}

uint32_t Chassis_Task_GetFaultFlags(void)
{
    return chassisFaultFlags;
}

void Chassis_Task_NotifyImuReady(void)
{
    /*
     * 底盘任务当前使用固定周期调度。
     * 保留该接口是为了兼容旧调用链，后续如改为 IMU 触发可在此处设置任务标志。
     */
    (void)chassisTaskHandle;
}
