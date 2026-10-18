#include "shooter_control.h"

#include "Limit.h"
#include "task_remote.h"

#include "stm32h7xx_hal.h"

#include <math.h>
#include <string.h>

Shooter_t Shooter;

/* 摩擦轮下标 -> 转向。两轮面对面夹弹、转向相反，符号见 app_config.h 的 APP_SHOOT_FRIC_1/2_DIR。 */
static const float fricDir[APP_DJI_FRIC_COUNT] = {
    APP_SHOOT_FRIC_1_DIR,
    APP_SHOOT_FRIC_2_DIR,
};

void Shooter_Init(void)
{
    uint32_t index;

    memset(&Shooter, 0, sizeof(Shooter));
    Shooter.dt = Shooter_Config.default_dt;
    Shooter.fric_target_rpm = Shooter_Config.fric_default_rpm;
    Shooter.trigger_I_limit = Shooter_Config.trigger_default_I_limit;
    Shooter.jam_back_flag = Shooter_Config.jam_back_default_flag;
    /* 上电即上锁：不能靠遥控同步那几帧离线碰巧先把它锁上。 */
    Shooter.trigger_lock = 1U;
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        Algorithm_PID_Init(&Shooter.fric_pid[index]);
    }
    Algorithm_PID_Init(&Shooter.trigger_angle_pid);
    Algorithm_PID_Init(&Shooter.trigger_speed_pid);
    Motor_DJI_SetSafe(1U);
    Motor_DJI_ClearCommands();
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    Motor_LK_SetSafe(1U);
    Motor_LK_SetCommand(0);
#endif
}

void Shooter_Feedback_Update(void)
{
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;
    uint8_t triggerOnline;
    uint8_t triggerEsc;

    Remote_Task_GetRemote(&Shooter.remote);
    Shooter.fault = SHOOTER_FAULT_NONE;

    /*
     * 只有本轮真要驱动的那条总线才把在线状态计入故障。分步装机时
     * 未装的电机恒离线，无条件计入会让 fault 永远非零、发射机构永远不出力。
     * 摩擦轮在 FDCAN2、拨盘在 FDCAN3，两条总线各看各的门。
     *
     * 摩擦轮的 C620 不报错误码（反馈 data[7] 是空字节），只判在线；
     * 拨盘的 C610 报堵转、三相未接、位置传感器丢失，见下面拨盘那段。
     */
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        Motor_DJI_GetState((motor_dji_index_t)index, &Shooter.fric_motor[index]);
        Shooter.fric_fault[index] = SHOOTER_FAULT_NONE;
        if (Shooter_Config.output.fric_flag == 0U)
        {
            continue;
        }
        if (Motor_DJI_IsOnline((motor_dji_index_t)index, nowTick) == 0U)
        {
            Shooter.fric_fault[index] |= SHOOTER_FAULT_MOTOR;
        }
        Shooter.fault |= Shooter.fric_fault[index];
    }

    /*
     * 拨盘换到【输出轴】口径：一发 = 2π/N 这个关系只在输出轴上成立。
     * ⚠ 与上面摩擦轮那套不同源：摩擦轮减速箱已拆，speedRpm 直接就是轮速；
     * 拨盘的减速箱还在（M2006 36:1、翎控 10:1），要除减速比才是输出轴。
     * 角速度是反馈转速的纯单位换算，没有状态，所以在用的地方直接算。
     */
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    Motor_LK_GetState(&Shooter.trigger_motor);
    triggerOnline = Motor_LK_IsOnline(nowTick);
    /*
     * 翎控的 0xA1 回复里没有错误码：本轮只判在线，过温欠压等保护由电机自己执行
     * （要读得额外轮询 0x9A，两份文档的位定义对不上，等上机再定）。
     */
    triggerEsc = 0U;
    /* 电机端 dps -> 输出轴 rad/s。 */
    Shooter.trigger_omega = (float)Shooter.trigger_motor.speedDps *
                            (APP_PI_F / 180.0f) / APP_LK_TRIGGER_GEAR_RATIO;
#else
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &Shooter.trigger_motor);
    triggerOnline = Motor_DJI_IsOnline(MOTOR_DJI_TRIGGER, nowTick);
    /*
     * ⚠ 拨盘的堵转码【不算】电调故障，交给卡弹状态机：算故障的话拨盘这一台会被清零，
     * 退弹就没有电流去执行。其余错误码（三相未接、传感器丢失……）照旧停拨盘这一台。
     */
    triggerEsc = ((Shooter.trigger_motor.errorCode != MOTOR_DJI_ERR_NONE) &&
                  (Shooter.trigger_motor.errorCode != MOTOR_DJI_ERR_STALL))
                     ? 1U
                     : 0U;
    /* 转子 rpm -> 输出轴 rad/s。 */
    Shooter.trigger_omega = (float)Shooter.trigger_motor.speedRpm *
                            (2.0f * APP_PI_F / 60.0f) / APP_DJI_TRIGGER_GEAR_RATIO;
#endif
    Shooter.trigger_angle = Shooter.trigger_motor.angle;

    Shooter.trigger_fault = SHOOTER_FAULT_NONE;
    if (Shooter_Config.output.trigger_flag != 0U)
    {
        if (triggerOnline == 0U)
        {
            Shooter.trigger_fault |= SHOOTER_FAULT_MOTOR;
        }
        if (triggerEsc != 0U)
        {
            Shooter.trigger_fault |= SHOOTER_FAULT_ESC;
        }
    }
    Shooter.fault |= Shooter.trigger_fault;

    /*
     * 急停或遥控离线（红线）：两部分都要停。判据与 gimbal_control.c 的
     * Gimbal_State_Update 同源——SwC 在 MID/DOWN 才是运行，UP 和 UNKNOWN 都算急停。
     * online 要单独看：不能指望离线时快照碰巧被清零。
     */
    Shooter.stop_flag =
        ((Shooter.remote.online != 0U) &&
         ((Shooter.remote.sw[REMOTE_SW_C] == REMOTE_SWITCH_MID) ||
          (Shooter.remote.sw[REMOTE_SW_C] == REMOTE_SWITCH_DOWN)))
            ? 0U
            : 1U;

    /* 上膛锁：急停期间锁上，解除后 VrB 退出拨弹区才解开。 */
    if (Shooter.stop_flag != 0U)
    {
        Shooter.trigger_lock = 1U;
    }
    else if (Shooter.remote.knobB < APP_RC_TRIGGER_ON)
    {
        Shooter.trigger_lock = 0U;
    }

    /* VrB 派生的操作者意图；急停时一律不请求，两路都走刹车。 */
    Shooter.fric_request =
        ((Shooter.stop_flag == 0U) && (Shooter.remote.knobB > APP_RC_FRIC_ON)) ? 1U : 0U;
    Shooter.trigger_request =
        ((Shooter.trigger_lock == 0U) && (Shooter.remote.knobB >= APP_RC_TRIGGER_ON)) ? 1U : 0U;
}


/** @brief 卡弹判据：电调堵转码【或】电流饱和。⚠ 只在 NORMAL 里调，理由见头文件。 */
static uint8_t Shooter_JamDetected(void)
{
    /* 饱和阈值跟着力矩限幅走，理由见 Shooter_Config_t.jam_current_ratio。 */
    float jamCurrent = Shooter_Config.jam_current_ratio *
                       Shooter.trigger_I_limit * APP_TRIGGER_RAW_PER_A;
    uint8_t stall = 0U;

#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
    /* 堵转码只有 C610 有（反馈 data[7]）；翎控只靠电流饱和这一条。 */
    stall = (Shooter.trigger_motor.errorCode == MOTOR_DJI_ERR_STALL) ? 1U : 0U;
#endif

    return ((stall != 0U) ||
            (fabsf((float)Shooter.trigger_motor.currentRaw) >= jamCurrent))
               ? 1U
               : 0U;
}

/** @brief 切卡弹状态并把停留计时清零。 */
static void Shooter_JamSetState(shooter_jam_state_t next)
{
    Shooter.jam_state = next;
    Shooter.jam_state_s = 0.0f;
}

/**
 * @brief 在途那一发的目标：从当前目标往回扣整发数，扣到刚好还在反馈角前面——扔掉它后面的积压。
 *
 * 目标从上一次目标累加，连发跟不上或卡住时会领先反馈好几发；只有最近的那一发是真正在转的。
 * 先扣掉到位容差再数：拨盘比某条槽位边界只靠后不到容差，那条边界算已经到了，不能当成在途。
 * 扣的发数下限是 0：已经到位（领先量在容差以内）时会数出 -1，不夹住就凭空多出一发。
 */
static float Shooter_Trigger_InFlightTarget(void)
{
    float lead = (Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR;
    float drop = fmaxf(ceilf((lead - Shooter_Config.trigger_arrive_rad) / APP_SHOOT_RAD_PER_AMMO) - 1.0f,
                       0.0f);

    return Shooter.trigger_target - drop * APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR;
}

/** @brief 回到待发：目标拉回当前角，计时与积分全清。在途那一发就此放弃。 */
static void Shooter_Trigger_EnterIdle(void)
{
    Shooter.trigger_state = SHOOTER_TRIGGER_IDLE;
    Shooter.trigger_state_s = 0.0f;
    Shooter.burst_s = 0.0f;
    Shooter.jam_retry = 0U;
    Shooter.trigger_target = Shooter.trigger_angle;
    Shooter.trigger_last_target = Shooter.trigger_angle;
}

static void Shooter_Jam_Update(void)
{
    Shooter.jam_state_s += Shooter.dt;

    switch (Shooter.jam_state)
    {
    case SHOOTER_JAM_NORMAL:
        /* 重打的那一发打完了：这一轮卡弹算解决，连续计数清零。GIVEUP 只给连续回拨都没打出去的。 */
        if ((Shooter.jam_retry != 0U) &&
            ((Shooter.trigger_resume_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR <=
             Shooter_Config.trigger_arrive_rad))
        {
            Shooter.jam_retry = 0U;
        }
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
            if (Shooter.jam_back_flag == 0U)
            {
                /* 回拨关着（机械组看原始卡弹用）：判到了就停在卡点，不回拨、不重打；松开 VrB 解除。 */
                Shooter_JamSetState(SHOOTER_JAM_GIVEUP);
                break;
            }
            /*
             * 一次性动作：先记下被卡的那一发（回拨完回到这里重打，它后面攒的积压扔掉），
             * 再从【反馈角】往回退一段。回拨期间拨弹逻辑被整个旁路，这条指令不会被下一轮覆盖。
             * ⚠ 不能从目标角往回减：目标从上一次目标累加，卡住时远在反馈前面，
             * 减完仍在前面，拨盘照样往前顶。基准取反馈角同 RoboWalker crt_booster.cpp:76、
             * 上交 rm_controllers 的 block()。
             */
            Shooter.trigger_resume_target = Shooter_Trigger_InFlightTarget();
            Shooter.trigger_target = Shooter.trigger_angle -
                                     Shooter_Config.jam_back_rad * APP_SHOOT_TRIGGER_DIR;
            Shooter.jam_retry++;
            Shooter_JamSetState(SHOOTER_JAM_BACKING);
        }
        break;

    case SHOOTER_JAM_BACKING:
        /* 退到位就走，弹已经松开了；退不动（弹卡死、电调关了输出）由超时兜底。 */
        if ((fabsf(Shooter.trigger_target - Shooter.trigger_angle) <=
             Shooter_Config.trigger_arrive_rad) ||
            (Shooter.jam_state_s >= Shooter_Config.jam_back_s))
        {
            /* 回到被卡的那一发重打：这一发不丢，卡住期间攒的积压也不补打。 */
            Shooter.trigger_target = Shooter.trigger_resume_target;
            /*
             * 连续 jam_retry_max 次回拨都没把那一发打出去，就放弃。参考的四家（DJI、上交、
             * RoboWalker、湖南大学）都没有这一层，可以永远"退一点、再顶、再退"，把电机烧在里面。
             */
            Shooter_JamSetState(
                (Shooter.jam_retry >= Shooter_Config.jam_retry_max)
                    ? SHOOTER_JAM_GIVEUP
                    : SHOOTER_JAM_NORMAL);
        }
        break;

    case SHOOTER_JAM_GIVEUP:
        /*
         * 唯一出口：操作者松开 VrB。堵转消失本身解不开，否则会反复自动重试。
         * 解除时被卡的那一发一起放弃（回待发）：松开在拨弹那边是"收尾、把在途那一发打完"，
         * 不放弃的话拨盘立刻又顶回那颗卡死的弹，判卡弹、回拨、GIVEUP 没完没了。
         */
        if (Shooter.trigger_request == 0U)
        {
            Shooter_Trigger_EnterIdle();
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

/**
 * @brief 松开 VrB：不再加发，在途那一发打完才回待发；它后面攒的积压扔掉。
 *
 * ⚠ 不能像原来那样直接回待发：目标立刻拉回反馈角，拨盘停在槽位中间、弹推到一半，
 * 之后每一发都带着这截相位差。急停和 GIVEUP 松开才是"立刻放弃"，走 EnterIdle。
 */
static void Shooter_Trigger_EnterFinish(void)
{
    Shooter.trigger_target = Shooter_Trigger_InFlightTarget();
    Shooter.trigger_state = SHOOTER_TRIGGER_FINISH;
    Shooter.trigger_state_s = 0.0f;
    Shooter.burst_s = 0.0f;
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
            /*
             * 拨盘还停在上一发终点附近（收尾到容差以内就回了待发）：从终点累加，
             * 否则每松开一次就丢一截相位。急停放弃过、被手转开了，就从当前角起。
             */
            if (fabsf(Shooter.trigger_angle - Shooter.trigger_last_target) <=
                Shooter_Config.trigger_arrive_rad)
            {
                Shooter.trigger_target = Shooter.trigger_last_target;
            }
            /* 进使能区【当拍】就出这一发，这就是"单发"。 */
            Shooter_Trigger_AddAmmo();
            Shooter.trigger_state = SHOOTER_TRIGGER_HOLD;
            Shooter.trigger_state_s = 0.0f;
        }
        break;

    case SHOOTER_TRIGGER_HOLD:
        if (Shooter.trigger_request == 0U)
        {
            Shooter_Trigger_EnterFinish();
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
            Shooter_Trigger_EnterFinish();
            break;
        }
        Shooter.burst_s += Shooter.dt;
        if (Shooter.burst_s >= burstPeriod)
        {
            /* 减去一个周期而不是清零：节拍不会被 dt 的零头一点点拖慢。 */
            Shooter.burst_s -= burstPeriod;
            /*
             * 疑似卡弹期间这一拍跳过、不补：卡弹在确认之前自己松开时，
             * 不会把这段时间攒下的发一口气补打出去。
             */
            if (Shooter.jam_state != SHOOTER_JAM_SUSPECT)
            {
                Shooter_Trigger_AddAmmo();
            }
        }
        break;

    case SHOOTER_TRIGGER_FINISH:
        if (Shooter.trigger_request != 0U)
        {
            /* 收尾时又进使能区：新的一发，从在途那一发的目标再加一发。 */
            Shooter_Trigger_AddAmmo();
            Shooter.trigger_state = SHOOTER_TRIGGER_HOLD;
            Shooter.trigger_state_s = 0.0f;
        }
        else if ((Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR <=
                 Shooter_Config.trigger_arrive_rad)
        {
            float done = Shooter.trigger_target;

            Shooter_Trigger_EnterIdle();
            Shooter.trigger_last_target = done;
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
    /*
     * 力矩限幅：速度环输出之后硬夹，不改 PID 参数，Watch 里改 trigger_I_limit 下一拍就生效。
     * 待发和急停的刹车也走这里，同样受限。
     */
    Shooter.trigger_I_req = (int16_t)Algorithm_LimitSymmetric(
        current, Shooter.trigger_I_limit * APP_TRIGGER_RAW_PER_A);
}

void Shooter_Control(void)
{
    uint32_t index;
    uint8_t readyCount = 0U;
    float fricTarget;

    memset(Shooter.I_req, 0, sizeof(Shooter.I_req));
    Shooter.trigger_I_req = 0;
    Shooter.fric_ready_flag = 0U;

    if (Shooter.stop_flag != 0U)
    {
        /*
         * 急停或遥控离线：两层状态机复位——退弹也不许再动，拨盘回待发。
         * 待发态的串级环照算（请求供 Watch 观察），但拨盘的最终电流在
         * Shooter_Command_Send 里清零，理由见那里。
         */
        Shooter_JamSetState(SHOOTER_JAM_NORMAL);
        Shooter_Trigger_EnterIdle();
    }
    else
    {
        Shooter_Jam_Update();
    }

    /*
     * 防卡弹在前、且优先：只有 NORMAL / SUSPECT 才跑拨弹逻辑。
     * BACKING / GIVEUP 期间整个旁路，退弹的目标角才不会被下一轮加发覆盖。
     * 这一个 if 就是两层状态机正交的全部机制。
     */
    if ((Shooter.jam_state == SHOOTER_JAM_NORMAL) ||
        (Shooter.jam_state == SHOOTER_JAM_SUSPECT))
    {
        Shooter_Trigger_Update();
    }
    Shooter_Trigger_Output();

    /*
     * 摩擦轮的"停"是 PID 刹车：目标转速置 0，积分逐拍清——纯 P，刹车电流 = -kp·转速，
     * 随转速衰减到 0，停稳后不再推。不用固定电流：固定值在轮子停住之后还会继续推，
     * 把它反向拖起来。VrB 回下半程、急停、遥控离线都走这一条。
     * 参考 RoboWalker crt_booster.cpp:187 与湖南大学 shoot_task.c:158，关摩擦轮都是目标置 0。
     *
     * 每台的目标 = 自己的方向 × fricTarget：两轮转向相反，就绪判据也得拿带方向的目标比。
     */
    fricTarget = (Shooter.fric_request != 0U) ? Shooter.fric_target_rpm : 0.0f;
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        float output = 0.0f;
        float speedRpm = (float)Shooter.fric_motor[index].speedRpm;
        float targetRpm = fricDir[index] * fricTarget;

        if (Shooter.fric_request == 0U)
        {
            Algorithm_PID_Init(&Shooter.fric_pid[index]);
        }

        /*
         * ⚠ feedbackRate 传 0.0f 配合 kd = 0，把这个函数用成纯 PI。
         * 它的 derivative 项是 -feedbackRate，本意给"反馈速度可信"的位置环用；
         * 速度环的导数是加速度，我们没有可信来源，所以不给。不是漏传参数。
         */
        Algorithm_PID_UpdateByFeedbackRate(&Shooter_Config.fric_speed_pid,
                                           &Shooter.fric_pid[index],
                                           targetRpm,
                                           speedRpm,
                                           0.0f,
                                           Shooter.dt,
                                           &output);
        Shooter.I_req[index] = (int16_t)output;

        if ((Shooter.fric_request != 0U) &&
            (fabsf(targetRpm - speedRpm) <= Shooter_Config.fric_ready_rpm))
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

    /* 整体安全门只剩编译期总开关；故障按台清零，见下。 */
    Shooter.safe_flag = (APP_GIMBAL_OUTPUT_ENABLE != 0U) ? 0U : 1U;

    /*
     * 按台把门：哪台有故障只清它自己，别的照常出力、照常刹车。
     * 未使能的总线上那台电机的最终命令也压成 0：它的帧本来就不发，
     * 但让 Shooter.I 如实反映"实际下发了什么"，Watch 上才不会误导。
     */
    for (index = 0U; index < APP_DJI_FRIC_COUNT; index++)
    {
        Shooter.I[index] =
            ((Shooter.safe_flag == 0U) && (Shooter_Config.output.fric_flag != 0U) &&
             (Shooter.fric_fault[index] == SHOOTER_FAULT_NONE))
                ? Shooter.I_req[index]
                : 0;
        Motor_DJI_SetCommand((motor_dji_index_t)index, Shooter.I[index]);
    }
    /*
     * ⚠ 拨盘在急停/遥控离线时【不刹车】，最终电流直接清零（摩擦轮仍走 PID 刹车）。
     * 翎控在 0xA1 转矩模式下会一直执行收到的最后一个 iq：云台板一停发（烧录时单片机被
     * 停住、死机），那一帧刹车电流就成了开环的固定电流，把拨盘拖着转——2026-09-30 烧录时
     * 实机遇到拨盘疯转。急停时发 0，先打急停再烧录，最后一帧就是 0。
     * 这只管得了"急停后停发"这一种情况，真正的兜底是电机上位机里的 Lose Input Protection。
     */
    Shooter.trigger_I =
        ((Shooter.safe_flag == 0U) && (Shooter_Config.output.trigger_flag != 0U) &&
         (Shooter.trigger_fault == SHOOTER_FAULT_NONE) && (Shooter.stop_flag == 0U))
            ? Shooter.trigger_I_req
            : 0;

    Motor_DJI_SetSafe(Shooter.safe_flag);
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    Motor_LK_SetCommand(Shooter.trigger_I);
    Motor_LK_SetSafe(Shooter.safe_flag);
    Motor_LK_UpdateTxFrames(Shooter_Config.output.trigger_flag);
    /* 拨盘换成翎控后 FDCAN3 上没有 C610，DJI 的那一帧不发。 */
    Motor_DJI_UpdateTxFrames(Shooter_Config.output.fric_flag, 0U);
#else
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, Shooter.trigger_I);
    Motor_DJI_UpdateTxFrames(Shooter_Config.output.fric_flag,
                             Shooter_Config.output.trigger_flag);
#endif
}
