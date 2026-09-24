#include "shooter_control.h"

#include "Limit.h"
#include "remote_input.h"

#include "stm32h7xx_hal.h"

#include <math.h>
#include <string.h>

Shooter_t Shooter;

/** @brief 某条总线上的电机是否被本轮使能，未使能的不计入故障。 */
static uint8_t Shooter_BusEnabled(motor_dji_index_t index)
{
    return (Motor_DJI_BusOf(index) == APP_CAN_BUS_FDCAN3)
               ? Shooter_Config.output.trigger_flag
               : Shooter_Config.output.fric_flag;
}

void Shooter_Init(void)
{
    uint32_t index;

    memset(&Shooter, 0, sizeof(Shooter));
    Shooter.dt = Shooter_Config.default_dt;
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        Algorithm_PID_Init(&Shooter.fric_pid[index]);
    }
    Motor_DJI_SetSafe(1U);
    Motor_DJI_ClearCommands();
}

void Shooter_Feedback_Update(void)
{
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;

    Shooter.fault = SHOOTER_FAULT_NONE;

    for (index = 0U; index < APP_DJI_COUNT; index++)
    {
        Motor_DJI_GetState((motor_dji_index_t)index, &Shooter.motor[index]);

        /*
         * 只有本轮真要驱动的那条总线才把在线状态计入故障。分步装机时
         * 未装的电机恒离线，无条件计入会让 fault 永远非零、发射机构永远不出力。
         */
        if (Shooter_BusEnabled((motor_dji_index_t)index) == 0U)
        {
            continue;
        }
        if (Motor_DJI_IsOnline((motor_dji_index_t)index, nowTick) == 0U)
        {
            Shooter.fault |= SHOOTER_FAULT_MOTOR;
        }
        /*
         * C610 在反馈里直接报堵转、三相未接、位置传感器丢失。拨盘卡弹就是
         * MOTOR_DJI_ERR_STALL，这是白送的诊断，不接进故障太可惜。
         */
        if (Shooter.motor[index].errorCode != MOTOR_DJI_ERR_NONE)
        {
            Shooter.fault |= SHOOTER_FAULT_ESC;
        }
    }

    /* VrB 派生量，本步只产出不消费，供 Watch 核对旋钮解析。 */
    Shooter.fric_request = (Remote.knobB > APP_RC_FRIC_ON) ? 1U : 0U;
    Shooter.trigger_request = (Remote.knobB >= APP_RC_TRIGGER_ON) ? 1U : 0U;
}

void Shooter_Control(void)
{
    uint32_t index;
    uint8_t readyCount = 0U;

    memset(Shooter.I_req, 0, sizeof(Shooter.I_req));
    Shooter.fric_ready_flag = 0U;

    if (Shooter.enable_flag == 0U)
    {
        /* 总开关关掉时积分必须清，否则重新使能会带着一坨陈旧积分猛冲。 */
        for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
        {
            Algorithm_PID_Init(&Shooter.fric_pid[index]);
        }
        return;
    }

    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        float output = 0.0f;
        float speedRpm = (float)Shooter.motor[index].speedRpm;

        /*
         * ⚠ feedbackRate 传 0.0f 配合 kd = 0，把这个函数用成纯 PI。
         * 它的 derivative 项是 -feedbackRate，本意给"反馈速度可信"的位置环用；
         * 速度环的导数是加速度，我们没有可信来源，所以不给。不是漏传参数。
         */
        Algorithm_PID_UpdateByFeedbackRate(&Shooter_Config.fric_speed_pid,
                                           &Shooter.fric_pid[index],
                                           Shooter.fric_target_rpm,
                                           speedRpm,
                                           0.0f,
                                           Shooter.dt,
                                           &output);
        Shooter.I_req[index] = (int16_t)output;

        if (fabsf(Shooter.fric_target_rpm - speedRpm) <=
            Shooter_Config.fric_ready_rpm)
        {
            readyCount++;
        }
    }

    if ((readyCount == APP_DJI_FRIC_COUNT) &&
        (Shooter.fric_target_rpm != 0.0f))
    {
        Shooter.fric_ready_flag = 1U;
    }

    /* 拨盘本步没有控制环，直接透传手动电流。限幅由设备层统一做。 */
    Shooter.I_req[MOTOR_DJI_TRIGGER] = Shooter.trigger_I_manual;
}

void Shooter_Command_Send(void)
{
    uint32_t index;

    Shooter.safe_flag =
        ((APP_GIMBAL_OUTPUT_ENABLE != 0U) &&
         (Shooter.fault == SHOOTER_FAULT_NONE) &&
         (Shooter.enable_flag != 0U))
            ? 0U
            : 1U;

    for (index = 0U; index < APP_DJI_COUNT; index++)
    {
        /*
         * 未使能的总线上那台电机的最终命令也压成 0：它的帧本来就不发，
         * 但让 Shooter.I 如实反映"实际下发了什么"，Watch 上才不会误导。
         */
        Shooter.I[index] =
            ((Shooter.safe_flag == 0U) &&
             (Shooter_BusEnabled((motor_dji_index_t)index) != 0U))
                ? Shooter.I_req[index]
                : 0;
        Motor_DJI_SetCommand((motor_dji_index_t)index, Shooter.I[index]);
    }

    Motor_DJI_SetSafe(Shooter.safe_flag);
    Motor_DJI_UpdateTxFrames(Shooter_Config.output.fric_flag,
                             Shooter_Config.output.trigger_flag);
}
