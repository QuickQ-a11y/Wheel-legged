#include "gimbal_control.h"

#include "Limit.h"
#include "task_can.h"

#include <string.h>

Gimbal_t Gimbal;

/**
 * @brief 把一台 DM 的反馈折算成关节量。
 *
 * 位置、速度、力矩统一消费 scale*ratio；设备层不维护安装方向。
 */
static void Axis_Update(Gimbal_Axis_t *axis,
                        const Gimbal_Axis_Config_t *config,
                        motor_dm_index_t index,
                        uint32_t nowTick)
{
    motor_dm_state_t motor;
    float gain = config->scale * config->ratio;

    Motor_DM_GetState(index, &motor);

    axis->angle = (motor.positionRad - config->zero_rad) * gain;
    axis->d_angle = motor.velocityRadps * gain;
    axis->T_fb = motor.torqueNm * gain;
    axis->online = Motor_DM_IsOnline(index, nowTick);
}

void Gimbal_Init(void)
{
    memset(&Gimbal, 0, sizeof(Gimbal));

    Gimbal.state = GIMBAL_ZERO_FORCE;
    Gimbal.fault = GIMBAL_FAULT_NONE;
    Gimbal.dt = Gimbal_Config.default_dt;
    /* 上电默认封锁输出，由每周期的安全门重新判定。 */
    Gimbal.safe_flag = 1U;

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
}

void Gimbal_Feedback_Update(void)
{
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;

    IMU_Task_GetState(&Gimbal.imu);

    Axis_Update(&Gimbal.yaw, &Gimbal_Config.yaw, MOTOR_DM_YAW, nowTick);
    Axis_Update(&Gimbal.pitch, &Gimbal_Config.pitch, MOTOR_DM_PITCH, nowTick);

    Gimbal.fault = GIMBAL_FAULT_NONE;

    if ((Gimbal.imu.isInitialized == 0U) || (Gimbal.imu.isAttitudeReady == 0U))
    {
        Gimbal.fault |= GIMBAL_FAULT_IMU;
    }
    if ((Gimbal.yaw.online == 0U) || (Gimbal.pitch.online == 0U))
    {
        Gimbal.fault |= GIMBAL_FAULT_DM;
    }
    for (index = 0U; index < APP_DJI_COUNT; index++)
    {
        if (Motor_DJI_IsOnline((motor_dji_index_t)index, nowTick) == 0U)
        {
            Gimbal.fault |= GIMBAL_FAULT_DJI;
        }
    }
    if (CAN_Task_GetTxErrorCount() > APP_CAN_TX_ERROR_MAX)
    {
        Gimbal.fault |= GIMBAL_FAULT_CAN;
    }
    if ((Remote.online == 0U) || (Remote.rightSwitch == REMOTE_SWITCH_DOWN))
    {
        Gimbal.fault |= GIMBAL_FAULT_REMOTE;
    }
}

void Gimbal_State_Update(void)
{
    switch (Remote.modeRequest)
    {
    case REMOTE_MODE_MANUAL:
        Gimbal.state = GIMBAL_MANUAL;
        break;

    case REMOTE_MODE_VISION:
        Gimbal.state = GIMBAL_VISION;
        break;

    case REMOTE_MODE_IDENT:
        Gimbal.state = GIMBAL_IDENT;
        break;

    case REMOTE_MODE_ZERO_FORCE:
        Gimbal.state = GIMBAL_ZERO_FORCE;
        break;

    case REMOTE_MODE_NONE:
    default:
        break;
    }
}

/**
 * @brief 计算本周期的力矩与电流请求。
 *
 * 控制律尚未接入，本轮所有请求量恒为零；请求量与最终命令分开保存，
 * 故障时只清最终命令，Watch 仍能看到控制器想输出什么。
 */
void Gimbal_Control(void)
{
    Gimbal.yaw.T_req = 0.0f;
    Gimbal.pitch.T_req = 0.0f;
    memset(Gimbal.I_dji_req, 0, sizeof(Gimbal.I_dji_req));
}

void Gimbal_Command_Send(void)
{
    motor_dm_command_t command;

    Gimbal.safe_flag = 1U;
    if ((APP_GIMBAL_OUTPUT_ENABLE != 0U) &&
        (Gimbal.fault == GIMBAL_FAULT_NONE))
    {
        Gimbal.safe_flag = 0U;
    }

    if ((Gimbal.safe_flag == 0U) && (Gimbal_Config.output.dm_flag != 0U))
    {
        Gimbal.yaw.T = Algorithm_LimitSymmetric(Gimbal.yaw.T_req,
                                                Gimbal_Config.yaw.T_limit);
        Gimbal.pitch.T = Algorithm_LimitSymmetric(Gimbal.pitch.T_req,
                                                  Gimbal_Config.pitch.T_limit);
    }
    else
    {
        Gimbal.yaw.T = 0.0f;
        Gimbal.pitch.T = 0.0f;
    }

    if ((Gimbal.safe_flag == 0U) && (Gimbal_Config.output.dji_flag != 0U))
    {
        memcpy(Gimbal.I_dji, Gimbal.I_dji_req, sizeof(Gimbal.I_dji));
    }
    else
    {
        memset(Gimbal.I_dji, 0, sizeof(Gimbal.I_dji));
    }

    /* 力矩按关节量下发回电机量，与反馈折算互为逆运算。 */
    command.torqueNm = Gimbal.yaw.T / (Gimbal_Config.yaw.scale * Gimbal_Config.yaw.ratio);
    Motor_DM_SetCommand(MOTOR_DM_YAW, &command);
    command.torqueNm = Gimbal.pitch.T /
                       (Gimbal_Config.pitch.scale * Gimbal_Config.pitch.ratio);
    Motor_DM_SetCommand(MOTOR_DM_PITCH, &command);

    Motor_DM_SetSafe(Gimbal.safe_flag);
    Motor_DM_SetEnable((Gimbal.safe_flag == 0U) ? 1U : 0U);
    Motor_DM_UpdateTxFrames();

    CAN_Task_SetDjiCurrent(Gimbal.I_dji);
    CAN_Task_RequestTx();
}
