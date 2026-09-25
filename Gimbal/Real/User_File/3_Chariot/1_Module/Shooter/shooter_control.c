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
    Shooter.fric_target_rpm = Shooter_Config.fric_default_rpm;
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        Algorithm_PID_Init(&Shooter.fric_pid[index]);
    }
    Algorithm_PID_Init(&Shooter.trigger_angle_pid);
    Algorithm_PID_Init(&Shooter.trigger_speed_pid);
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

    /*
     * 拨盘换到【输出轴】口径：一发 = 2π/N 这个关系只在输出轴上成立。
     * ⚠ 与上面摩擦轮那套不同源：摩擦轮减速箱已拆，speedRpm 直接就是轮速；
     * 拨盘减速箱还在，要除 36 才是输出轴。别把两边的数混着比。
     * 角速度是 speedRpm 的纯单位换算，没有状态，所以在用的地方直接算。
     */
    Shooter.trigger_angle = Shooter.motor[MOTOR_DJI_TRIGGER].angle;
    Shooter.trigger_omega =
        (float)Shooter.motor[MOTOR_DJI_TRIGGER].speedRpm *
        (2.0f * APP_PI_F / 60.0f) / APP_DJI_TRIGGER_GEAR_RATIO;

    /* VrB 派生的操作者意图。 */
    Shooter.fric_request = (Remote.knobB > APP_RC_FRIC_ON) ? 1U : 0U;
    Shooter.trigger_request = (Remote.knobB >= APP_RC_TRIGGER_ON) ? 1U : 0U;
}


/** @brief 卡弹判据：电调堵转码【或】电流饱和。⚠ 只在 NORMAL 里调，理由见头文件。 */
static uint8_t Shooter_JamDetected(void)
{
    const motor_dji_state_t *trigger = &Shooter.motor[MOTOR_DJI_TRIGGER];

    return ((trigger->errorCode == MOTOR_DJI_ERR_STALL) ||
            (fabsf((float)trigger->currentRaw) >= Shooter_Config.jam_current))
               ? 1U
               : 0U;
}

/** @brief 切卡弹状态并把停留计时清零。 */
static void Shooter_JamSetState(shooter_jam_state_t next)
{
    Shooter.jam_state = next;
    Shooter.jam_state_s = 0.0f;
}

static void Shooter_Jam_Update(void)
{
    Shooter.jam_state_s += Shooter.dt;

    switch (Shooter.jam_state)
    {
    case SHOOTER_JAM_NORMAL:
        if (Shooter_JamDetected() != 0U)
        {
            Shooter_JamSetState(SHOOTER_JAM_SUSPECT);
        }
        break;

    case SHOOTER_JAM_SUSPECT:
        if (Shooter_JamDetected() == 0U)
        {
            /* 短暂的大电流是弹丸挤压，不是卡弹。 */
            Shooter_JamSetState(SHOOTER_JAM_NORMAL);
        }
        else if (Shooter.jam_state_s >= Shooter_Config.jam_confirm_s)
        {
            /* 一次性动作：目标角往回退一段。退弹期间拨弹逻辑被整个旁路，
               所以这条指令不会被下一轮覆盖。 */
            Shooter.trigger_target -=
                Shooter_Config.jam_back_rad * APP_SHOOT_TRIGGER_DIR;
            Shooter.jam_retry++;
            Shooter_JamSetState(SHOOTER_JAM_BACKING);
        }
        break;

    case SHOOTER_JAM_BACKING:
        if (Shooter.jam_state_s >= Shooter_Config.jam_back_s)
        {
            /* 退够时间还没脱困就升级。参考的两个工程都没有这一层，
               于是可以永远"退一点、再顶、再退"，把电机烧在里面。 */
            Shooter_JamSetState(
                (Shooter.jam_retry >= Shooter_Config.jam_retry_max)
                    ? SHOOTER_JAM_GIVEUP
                    : SHOOTER_JAM_NORMAL);
        }
        break;

    case SHOOTER_JAM_GIVEUP:
        /* 唯一出口：操作者松开 VrB。堵转消失本身解不开，否则会反复自动重试。 */
        if (Shooter.trigger_request == 0U)
        {
            Shooter.jam_retry = 0U;
            Shooter_JamSetState(SHOOTER_JAM_NORMAL);
        }
        break;
    }
}

/**
 * @brief 目标角加一发。
 *
 * ⚠ 从【上一次目标】累加，不是从当前反馈角累加。参考实现
 * （RoboWalker crt_booster.cpp:217 的 Get_Now_Angle() + 2π/7）是后者，
 * 上一发还没转到位时新目标基于半途角，这一发就会短一截——
 * 连发、以及卡弹退弹刚结束时必然踩到。
 */
static void Shooter_Trigger_AddAmmo(void)
{
    Shooter.trigger_target += APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR;
}

/** @brief 回到待发：目标拉回当前角，计时与积分全清。 */
static void Shooter_Trigger_EnterIdle(void)
{
    Shooter.trigger_state = SHOOTER_TRIGGER_IDLE;
    Shooter.trigger_state_s = 0.0f;
    Shooter.burst_s = 0.0f;
    Shooter.jam_retry = 0U;
    Shooter.trigger_target = Shooter.trigger_angle;
}

static void Shooter_Trigger_Update(void)
{
    float burstPeriod = 1.0f / (float)APP_SHOOT_BURST_HZ;

    Shooter.trigger_state_s += Shooter.dt;

    switch (Shooter.trigger_state)
    {
    case SHOOTER_TRIGGER_IDLE:
        /*
         * 待发：目标跟随反馈，不积着没打出去的发数；两个环的积分【每拍都清】。
         *
         * ⚠ 不能只在刚进 IDLE 那一拍清：误差为 0 时积分不会自己衰减，
         * 上一轮留下的一坨积分会让拨盘在待发状态一直带着小电流。
         */
        Shooter.trigger_target = Shooter.trigger_angle;
        Algorithm_PID_Init(&Shooter.trigger_angle_pid);
        Algorithm_PID_Init(&Shooter.trigger_speed_pid);
        if (Shooter.trigger_request != 0U)
        {
            /* 进使能区【当拍】就出这一发，这就是"单发"。 */
            Shooter_Trigger_AddAmmo();
            Shooter.trigger_state = SHOOTER_TRIGGER_HOLD;
            Shooter.trigger_state_s = 0.0f;
        }
        break;

    case SHOOTER_TRIGGER_HOLD:
        if (Shooter.trigger_request == 0U)
        {
            Shooter_Trigger_EnterIdle();
        }
        else if (Shooter.trigger_state_s >=
                 ((float)APP_SHOOT_BURST_DELAY_MS * 0.001f))
        {
            Shooter.burst_s = 0.0f;
            Shooter.trigger_state = SHOOTER_TRIGGER_BURST;
            Shooter.trigger_state_s = 0.0f;
        }
        break;

    case SHOOTER_TRIGGER_BURST:
        if (Shooter.trigger_request == 0U)
        {
            Shooter_Trigger_EnterIdle();
            break;
        }
        Shooter.burst_s += Shooter.dt;
        if (Shooter.burst_s >= burstPeriod)
        {
            /* 减去一个周期而不是清零：节拍不会被 dt 的零头一点点拖慢。 */
            Shooter.burst_s -= burstPeriod;
            Shooter_Trigger_AddAmmo();
        }
        break;
    }
}

/** @brief 拨盘串级环：位置环（纯 P）-> 目标角速度 -> 速度环（PI）-> 电流。 */
static void Shooter_Trigger_Output(void)
{
    float omega = 0.0f;
    float current = 0.0f;

    if (Shooter.jam_state == SHOOTER_JAM_GIVEUP)
    {
        /*
         * 放弃状态只封锁拨盘，摩擦轮照转——脱困后要能立刻接着打。
         * 这里【不用】再给 I_req 写 0：Shooter_Control 开头已经整体 memset 过，
         * 不跑下面的串级环就等于零输出。
         */
        Shooter.trigger_target_omega = 0.0f;
        Algorithm_PID_Init(&Shooter.trigger_angle_pid);
        Algorithm_PID_Init(&Shooter.trigger_speed_pid);
        return;
    }

    /* ⚠ 外环 feedbackRate 传 0.0f 配合 kd = 0：串级的阻尼由内环给，
       外环再加微分会和内环打架。 */
    Algorithm_PID_UpdateByFeedbackRate(&Shooter_Config.trigger_angle_pid,
                                       &Shooter.trigger_angle_pid,
                                       Shooter.trigger_target,
                                       Shooter.trigger_angle,
                                       0.0f,
                                       Shooter.dt,
                                       &omega);
    Shooter.trigger_target_omega = omega;

    Algorithm_PID_UpdateByFeedbackRate(&Shooter_Config.trigger_speed_pid,
                                       &Shooter.trigger_speed_pid,
                                       Shooter.trigger_target_omega,
                                       Shooter.trigger_omega,
                                       0.0f,
                                       Shooter.dt,
                                       &current);
    Shooter.I_req[MOTOR_DJI_TRIGGER] = (int16_t)current;
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
        Algorithm_PID_Init(&Shooter.trigger_angle_pid);
        Algorithm_PID_Init(&Shooter.trigger_speed_pid);
        Shooter_Trigger_EnterIdle();
        Shooter_JamSetState(SHOOTER_JAM_NORMAL);
        return;
    }

    /*
     * 防卡弹在前、且优先：只有 NORMAL / SUSPECT 才跑拨弹逻辑。
     * BACKING / GIVEUP 期间整个旁路，退弹的目标角才不会被下一轮加发覆盖。
     * 这一个 if 就是两层状态机正交的全部机制。
     */
    Shooter_Jam_Update();
    if ((Shooter.jam_state == SHOOTER_JAM_NORMAL) ||
        (Shooter.jam_state == SHOOTER_JAM_SUSPECT))
    {
        Shooter_Trigger_Update();
    }
    Shooter_Trigger_Output();

    if (Shooter.fric_request == 0U)
    {
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
