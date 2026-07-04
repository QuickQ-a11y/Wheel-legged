#include "chassis_task.h"

#include "app_config.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "module_chassis.h"
#include "task_can.h"
#include "task_can_dispatch.h"
#include "task_imu.h"

#include "cmsis_os2.h"

#include <string.h>

typedef enum
{
    CHASSIS_STATE_STANDING = 0,       /* 正常站立和平衡控制。 */
    CHASSIS_STATE_ZERO_FORCE,         /* 零力矩状态，所有输出保持安全零值。 */
    CHASSIS_STATE_FALLEN,             /* 倒地状态框架，后续接入起身前腿部调整。 */
    CHASSIS_STATE_FALLING_TO_STAND,   /* 重新站立框架，后续接入小板凳准备姿态。 */
    CHASSIS_STATE_BENCH,              /* 小板凳调试状态，后续验证腿和轮。 */
} chassis_state_t;

static osThreadId_t chassisTaskHandle;
static uint8_t chassisEnable;
static chassis_mode_t chassisMode;
static chassis_state_t chassisState;

static const osThreadAttr_t chassisTaskAttributes = {
    .name = "ChassisTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 将输出结构清零并标记为安全输出。
 *
 * 未实现模式、未使能和故障兜底都使用这一路径，避免电机保留上一帧命令。
 */
static void ChassisTask_SetZeroOutput(module_chassis_output_t *chassisOutput)
{
    memset(chassisOutput, 0, sizeof(*chassisOutput));
    chassisOutput->safeOutput = 1U;
    chassisOutput->faultFlags = MODULE_CHASSIS_FAULT_NONE;
    Module_Chassis_ResetMotionState();
}

/**
 * @brief 将 IMU 任务状态转换为底盘模块输入。
 */
static void ChassisTask_FillImuInput(const task_imu_state_t *imuState,
                                     module_chassis_input_t *chassisInput)
{
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
    memcpy(chassisInput->imu.motionAccMps2,
           imuState->motionAccMps2,
           sizeof(chassisInput->imu.motionAccMps2));
    chassisInput->imu.dtSec = imuState->dtSec;
}

/**
 * @brief 将电机设备层状态转换为底盘模块输入。
 */
static void ChassisTask_FillMotorInput(uint32_t nowTick,
                                       module_chassis_input_t *chassisInput)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t dmState = {0};

        Motor_DM_GetState((motor_dm_index_t)index, &dmState);
        chassisInput->dmMotors[index].isOnline =
            Motor_DM_IsOnline((motor_dm_index_t)index, nowTick);
        chassisInput->dmMotors[index].positionRad = dmState.positionRad;
        chassisInput->dmMotors[index].velocityRadps = dmState.velocityRadps;
        chassisInput->dmMotors[index].torqueNm = dmState.torqueNm;
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
 * @brief 收集底盘模块输入，对应 SPR 的 chassis_feedback_update 前半段。
 *
 * 当前工程的腿部几何、状态向量和 VMC 中间量仍在底盘模块中计算。
 * 任务层只负责把设备层反馈整理成模块输入。
 */
static void ChassisTask_ReadFeedback(module_chassis_input_t *chassisInput)
{
    task_imu_state_t imuState = {0};
    uint32_t nowTick = HAL_GetTick();

    memset(chassisInput, 0, sizeof(*chassisInput));
    chassisInput->isEnabled = chassisEnable;
    chassisInput->canTxErrorCount = CAN_Task_GetTxErrorCount();

    IMU_Task_GetState(&imuState);
    ChassisTask_FillImuInput(&imuState, chassisInput);
    ChassisTask_FillMotorInput(nowTick, chassisInput);
}

/**
 * @brief 将底盘模块输出下发到电机设备层和 CAN 发送缓存。
 */
static void ChassisTask_ApplyOutput(const module_chassis_output_t *chassisOutput)
{
    uint32_t index;

    Motor_DM_SetSafe(chassisOutput->safeOutput);
    if (chassisOutput->safeOutput != 0U)
    {
        Motor_DM_ZeroAll();
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_command_t command = {
            .torqueNm = chassisOutput->dmCommands[index].torqueNm,
        };

        Motor_DM_SetCommand((motor_dm_index_t)index, &command);
    }

    CAN_Task_SetDjiCurrent(chassisOutput->djiCurrents);
    Motor_DM_UpdateTxFrames();
    CAN_Task_RequestTx();
}

/**
 * @brief 设置外层底盘模式，对应 SPR 的 chassis_set_mode。
 *
 * 目前遥控/上层输入尚未接入，外层模式由 ChassisTask_SetMode 写入。
 * 后续接入遥控时，只在这里把遥控值映射成 chassisMode。
 */
static void ChassisTask_SelectMode(void)
{
    /* 当前没有遥控源，保留阶段入口以固定 SPR 风格主流程。 */

}

/**
 * @brief 设置本轮控制状态，对应 SPR 的 chassis_set_contorl。
 *
 * 第一阶段只根据外层模式选择内层状态；速度、腿长、yaw 和 roll 目标
 * 后续在这里接入遥控或上层控制输入。
 */
static void ChassisTask_SetControl(void)
{
    if (chassisEnable == 0U)
    {
        chassisState = CHASSIS_STATE_ZERO_FORCE;
        return;
    }

    switch (chassisMode)
    {
    case CHASSIS_MODE_ZERO_FORCE:
        chassisState = CHASSIS_STATE_ZERO_FORCE;
        break;

    case CHASSIS_MODE_FOLLOW:
    case CHASSIS_MODE_TOP:
        if ((chassisState == CHASSIS_STATE_ZERO_FORCE) ||
            (chassisState == CHASSIS_STATE_FALLEN) ||
            (chassisState == CHASSIS_STATE_BENCH))
        {
            chassisState = CHASSIS_STATE_STANDING;
        }
        break;

    case CHASSIS_MODE_SELF_SAVE:
        if ((chassisState != CHASSIS_STATE_FALLEN) &&
            (chassisState != CHASSIS_STATE_FALLING_TO_STAND))
        {
            chassisState = CHASSIS_STATE_FALLEN;
        }
        break;

    case CHASSIS_MODE_BENCH:
        chassisState = CHASSIS_STATE_BENCH;
        break;

    default:
        chassisState = CHASSIS_STATE_ZERO_FORCE;
        break;
    }
}

/**
 * @brief 正常站立状态，当前进入现有 LQR/VMC 平衡链路。
 */
static void ChassisTask_StayStanding(const module_chassis_input_t *chassisInput,
                                     module_chassis_output_t *chassisOutput)
{
    Module_Chassis_RunControl(chassisInput, chassisOutput);
}

/**
 * @brief 零力矩状态，持续覆盖安全零输出。
 */
static void ChassisTask_StayZeroForce(module_chassis_output_t *chassisOutput)
{
    ChassisTask_SetZeroOutput(chassisOutput);
}

/**
 * @brief 倒地状态框架，当前不输出非零力矩。
 *
 * 真正起身前需要补充倒地判据、目标腿姿态和关节 PID 参数。
 */
static void ChassisTask_StayFallen(module_chassis_output_t *chassisOutput)
{
    ChassisTask_SetZeroOutput(chassisOutput);
}

/**
 * @brief 重新站立状态框架，当前仍保持安全零输出。
 */
static void ChassisTask_StayFallingToStand(module_chassis_output_t *chassisOutput)
{
    ChassisTask_SetZeroOutput(chassisOutput);
}

/**
 * @brief 小板凳调试状态框架，当前不输出非零力矩。
 */
static void ChassisTask_StayBench(module_chassis_output_t *chassisOutput)
{
    ChassisTask_SetZeroOutput(chassisOutput);
}

/**
 * @brief 按内层状态执行一轮控制，对应 SPR 的 chassis_control_loop。
 */
static void ChassisTask_ControlLoop(const module_chassis_input_t *chassisInput,
                                    module_chassis_output_t *chassisOutput)
{
    if (chassisEnable == 0U)
    {
        ChassisTask_SetZeroOutput(chassisOutput);
        chassisOutput->faultFlags = MODULE_CHASSIS_FAULT_DISABLED;
        return;
    }

    switch (chassisState)
    {
    case CHASSIS_STATE_STANDING:
        ChassisTask_StayStanding(chassisInput, chassisOutput);
        break;

    case CHASSIS_STATE_ZERO_FORCE:
        ChassisTask_StayZeroForce(chassisOutput);
        break;

    case CHASSIS_STATE_FALLEN:
        ChassisTask_StayFallen(chassisOutput);
        break;

    case CHASSIS_STATE_FALLING_TO_STAND:
        ChassisTask_StayFallingToStand(chassisOutput);
        break;

    case CHASSIS_STATE_BENCH:
        ChassisTask_StayBench(chassisOutput);
        break;

    default:
        ChassisTask_StayZeroForce(chassisOutput);
        break;
    }
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

        ChassisTask_SelectMode();
        ChassisTask_ReadFeedback(&chassisInput);
        ChassisTask_SetControl();
        ChassisTask_ControlLoop(&chassisInput, &chassisOutput);

        ChassisTask_ApplyOutput(&chassisOutput);

        wakeTick += APP_CONFIG_CHASSIS_TASK_PERIOD_TICKS;
        (void)osDelayUntil(wakeTick);
    }
}

void ChassisTask_Init(void)
{
    chassisEnable = 0U;
    chassisMode = CHASSIS_MODE_ZERO_FORCE;
    chassisState = CHASSIS_STATE_ZERO_FORCE;
    chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTaskAttributes);
}

void ChassisTask_SetEnable(uint8_t enable)
{
    chassisEnable = (enable != 0U) ? 1U : 0U;
    Motor_DM_SetEnable(chassisEnable);
}

void ChassisTask_SetMode(chassis_mode_t mode)
{
    switch (mode)
    {
    case CHASSIS_MODE_ZERO_FORCE:
    case CHASSIS_MODE_FOLLOW:
    case CHASSIS_MODE_TOP:
    case CHASSIS_MODE_SELF_SAVE:
    case CHASSIS_MODE_BENCH:
        chassisMode = mode;
        break;

    default:
        chassisMode = CHASSIS_MODE_ZERO_FORCE;
        break;
    }
}

void ChassisTask_NotifyImuReady(void)
{
    /*
     * 底盘任务当前使用固定周期调度。
     * 保留该接口是为了兼容旧调用链，后续如改为 IMU 触发可在此处设置任务标志。
     */
    (void)chassisTaskHandle;
}
