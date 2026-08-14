#include "chassis_control.h"

#include "Angle.h"
#include "LQR.h"
#include "Limit.h"
#include "PID.h"
#include "chassis_config.h"

#include <math.h>
#include <string.h>

#define CHASSIS_RPM_TO_RADPS 0.10471975512f
#define CHASSIS_OUTPUT_FAULT_MASK                                         \
    (CHASSIS_FAULT_DISABLED | CHASSIS_FAULT_IMU |                       \
     CHASSIS_FAULT_DM_MOTOR | CHASSIS_FAULT_DM_ERROR |                  \
     CHASSIS_FAULT_DJI_MOTOR | CHASSIS_FAULT_CAN |                      \
     CHASSIS_FAULT_KINEMATICS | CHASSIS_FAULT_REMOTE)

Chassis_t Chassis;

/*
 * K矩阵每周期由 leg[].L0 重算后当周期用完，不跨周期携带信息，因此放在文件
 * 作用域而不挂进 Chassis，避免 Watch 树里多出 40 个派生量。调试器仍可观察。
 */
static float lqrK[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT];

static void Control_Reset(void);

/** @brief 让目标量每周期最多移动maximum_step，避免腿长目标阶跃。 */
static float Move_Toward(float value, float target, float maximum_step)
{
    float positive_step = fabsf(maximum_step);

    if (value < target - positive_step)
    {
        return value + positive_step;
    }
    if (value > target + positive_step)
    {
        return value - positive_step;
    }
    return target;
}

/**
 * @brief 将遥控运动目标写入正常站立的十维目标，不做斜坡。
 *
 * 位移和航向都按"有输入走速度、松杆锁位置"处理：
 * 有前进输入时位移状态在本文件后段被清零，位移项不参与，只跟速度；
 * 松杆后位移从零开始积分，把车锁在松杆位置。
 * 有偏航输入时航向目标按给定角速度积分；松杆瞬间锁存当前航向并保持，
 * 偏航角速度目标同时清零。
 */
static void Motion_Update(void)
{
    float dt = Chassis.dt;
    float model_yaw_rad = Chassis.imu.yaw_total *
                          Chassis_Config.imu.yaw_angle_scale;
    float top_phase_rad;

    if (Chassis.mode == CHASSIS_MODE_TOP)
    {
        top_phase_rad = model_yaw_rad - Chassis.top_fai;
        Chassis.top_d_s =
            Chassis.goal.d_s * cosf(top_phase_rad) +
            Chassis.goal.d_y * sinf(top_phase_rad);
        Chassis.lqr.target[CHASSIS_STATE_D_S] = Chassis.top_d_s;
        /* 小陀螺持续旋转，航向目标始终跟随实际，只靠角速度控制。 */
        Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
        Chassis.lqr.target[CHASSIS_STATE_D_FAI] = Chassis.goal.d_fai;
        Chassis.yaw_stick_flag = 0U;
    }
    else
    {
        Chassis.top_d_s = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_D_S] = Chassis.goal.d_s;

        if (Chassis.goal.d_fai != 0.0f)
        {
            /* 有偏航输入：按给定角速度积分出航向目标。 */
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = Chassis.goal.d_fai;
            Chassis.lqr.target[CHASSIS_STATE_FAI] +=
                Chassis.goal.d_fai * dt;
            Chassis.yaw_stick_flag = 1U;
        }
        else
        {
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = 0.0f;
            if (Chassis.yaw_stick_flag != 0U)
            {
                /* 松杆瞬间锁存当前航向，清掉转向期间的跟踪误差后保持。 */
                Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
                Chassis.yaw_stick_flag = 0U;
            }
        }
    }

    // Chassis.leg[CHASSIS_LEFT].target_L0 =
    //     Move_Toward(Chassis.leg[CHASSIS_LEFT].target_L0,
    //                        Chassis.goal.L0,
    //                        Chassis_Config.recovery.L0_rate *
    //                            dt);
    // Chassis.leg[CHASSIS_RIGHT].target_L0 =
    //     Move_Toward(Chassis.leg[CHASSIS_RIGHT].target_L0,
    //                        Chassis.goal.L0,
    //                        Chassis_Config.recovery.L0_rate *
    //                            dt);

    Chassis.leg[CHASSIS_LEFT].target_L0 = Chassis.goal.L0;
    Chassis.leg[CHASSIS_RIGHT].target_L0 = Chassis.goal.L0;
}

/** @brief 判断周期角是否存在落在指定连续区间内的等价角。 */
static uint8_t Angle_In_Range(float angle_rad,
                                        float minimum_rad,
                                        float maximum_rad)
{
    float center_rad = (minimum_rad + maximum_rad) * 0.5f;
    float equivalent_rad =
        Algorithm_AngleNearestEquivalentRad(angle_rad, center_rad);

    return ((equivalent_rad >= minimum_rad) &&
            (equivalent_rad <= maximum_rad)) ? 1U : 0U;
}

/**
 * @brief 汇总只影响最终电机输出许可的故障位。
 *
 * 这些故障不会阻断VMC、PID或LQR中间量计算；真正的几何和控制计算
 * 故障由对应控制流程另外追加到Chassis.fault。
 */
static uint32_t Output_Fault_Get(void)
{
    uint32_t fault = CHASSIS_FAULT_NONE;
    uint32_t index;

    if (Chassis.enable_flag == 0U)
    {
        fault |= CHASSIS_FAULT_DISABLED;
    }
    if ((Chassis.imu.init_flag == 0U) ||
        (Chassis.imu.attitude_flag == 0U) ||
        (Chassis.imu.error_code != 0U))
    {
        fault |= CHASSIS_FAULT_IMU;
    }
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        if (Chassis.dm_motor[index].online_flag == 0U)
        {
            fault |= CHASSIS_FAULT_DM_MOTOR;
        }
        /*
         * DM反馈状态0为失能、1为使能，8~E为超压、欠压、过流、过温、通讯丢失和过载。
         * 电机报错后会自行退出使能但仍在发反馈帧，只判online会漏掉这类静默失效。
         */
        if (Chassis.dm_motor[index].err_state >= 8U)
        {
            fault |= CHASSIS_FAULT_DM_ERROR;
        }
    }
    /* 轮通道关闭时不驱动轮电机，其在线状态不参与封锁，避免只调髋关节时
     * 轮电调未上电连带封锁关节力矩。 */
    if (Chassis_Config.output.wheel_flag != 0U)
    {
        for (index = 0U; index < APP_WHEEL_COUNT; index++)
        {
            if (Chassis.wheel_motor[index].online_flag == 0U)
            {
                fault |= CHASSIS_FAULT_DJI_MOTOR;
                break;
            }
        }
    }
    if (Chassis.can_error_count > APP_CAN_TX_ERROR_MAX)
    {
        fault |= CHASSIS_FAULT_CAN;
    }
    if ((Chassis.remote_online_flag == 0U) || (Chassis.remote_stop_flag != 0U))
    {
        fault |= CHASSIS_FAULT_REMOTE;
    }

    return fault;
}

/**
 * @brief 清空本轮全部控制请求量与最终电机命令，并置回发送前安全门。
 *
 * 每条控制链在入口调用一次，保证任何提前返回都不会遗留上一周期的命令。
 */
static void Output_Clear(void)
{
    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    memset(Chassis.output.T_wheel, 0, sizeof(Chassis.output.T_wheel));
    Chassis.output.safe_flag = 1U;
}

/** @brief 清空四个关节串级PID状态、目标和调试请求量。 */
static void Joint_Control_Reset(void)
{
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        Algorithm_PID_Init(&Chassis.joint_angle_pid[index]);
        Algorithm_PID_Init(&Chassis.joint_speed_pid[index]);
    }
    memset(Chassis.output.target_angle,
           0,
           sizeof(Chassis.output.target_angle));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
}

/**
 * @brief 切换内部控制状态并初始化该状态所需目标和控制器。
 *
 * 相同状态不重复进入，避免每周期清空计时器、PID和目标斜坡。
 */
static void State_Enter(Chassis_State_t state)
{
    uint32_t side;

    if (Chassis.state == state)
    {
        return;
    }

    Chassis.state = state;
    Chassis.state_time = 0.0f;
    Chassis.stable_time = 0.0f;
    Output_Clear();
    Joint_Control_Reset();

    if ((state == CHASSIS_STANDING) || (state == CHASSIS_BENCH) ||
        (state == CHASSIS_STEP))
    {
        Control_Reset();
        memcpy(Chassis.lqr.target,
               Chassis_Config.target,
               sizeof(Chassis.lqr.target));
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_FAI] =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    }

    if (state == CHASSIS_STANDING)
    {
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_LEFT]);
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_RIGHT]);
        Algorithm_PID_Init(&Chassis.roll_pid);
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            /* 五连杆正解有效时从当前腿长接管，否则回到配置目标腿长。 */
            Chassis.leg[side].target_L0 =
                ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
                 (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ?
                    Chassis.leg[side].L0 :
                    Chassis_Config.leg[side].target_L0;
        }
    }
    else if ((state == CHASSIS_FALLEN) ||
             (state == CHASSIS_FALLING_TO_STAND))
    {
        /*
         * 自救的腿长和腿角目标都从当前实际姿态起步，之后逐周期按速率推进。
         * 不锁存的话目标会相对实际角常值超前，关节PID全程满输出，动作很猛。
         * 正解无效时实际姿态是0，只能回到配置目标，否则逆解直接失败。
         */
        uint8_t leg_valid =
            ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
             (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;

        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_phi0 =
                (leg_valid != 0U) ? Chassis.leg[side].phi0_total
                                  : Chassis_Config.recovery.bench_phi0;
            Chassis.leg[side].target_L0 =
                (leg_valid != 0U) ? Chassis.leg[side].L0
                                  : Chassis_Config.leg[side].target_L0;
        }
    }
    else if (state == CHASSIS_STEP)
    {
        Chassis.step_phase = CHASSIS_STEP_PREPARE;
        Chassis.step_fai =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
        memset(Chassis.contact_time,
               0,
               sizeof(Chassis.contact_time));
        memset(Chassis.step_contact_flag,
               0,
               sizeof(Chassis.step_contact_flag));
        memset(Chassis.step_contact_latch_flag,
               0,
               sizeof(Chassis.step_contact_latch_flag));
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Algorithm_PID_Init(&Chassis.step_leg_angle_pid[side]);
            Chassis.leg[side].target_L0 =
                Chassis_Config.step.approach_L0;
        }
    }
}

/** @brief 重新初始化速度Kalman、前进速度、加速度和位移状态。 */
static void Control_Reset(void)
{
    Algorithm_Kalman_Init(&Chassis.speed_kalman, 2U, 2U);
    memcpy(Chassis.speed_kalman.covariance,
           Chassis_Config.speed_kalman.initial_covariance,
           sizeof(Chassis_Config.speed_kalman.initial_covariance));
    memcpy(Chassis.speed_kalman.processNoise,
           Chassis_Config.speed_kalman.process_noise,
           sizeof(Chassis_Config.speed_kalman.process_noise));
    memcpy(Chassis.speed_kalman.measurementNoise,
           Chassis_Config.speed_kalman.measurement_noise,
           sizeof(Chassis_Config.speed_kalman.measurement_noise));

    Chassis.speed_kalman.stateTransition[0] = 1.0f;
    Chassis.speed_kalman.stateTransition[1] = APP_CTRL_DT_S;
    Chassis.speed_kalman.stateTransition[2] = 0.0f;
    Chassis.speed_kalman.stateTransition[3] = 1.0f;
    Chassis.speed_kalman.measurementMatrix[0] = 1.0f;
    Chassis.speed_kalman.measurementMatrix[1] = 0.0f;
    Chassis.speed_kalman.measurementMatrix[2] = 0.0f;
    Chassis.speed_kalman.measurementMatrix[3] = 1.0f;

    Chassis.body.s = 0.0f;
    Chassis.body.d_s_raw = 0.0f;
    Chassis.body.d_s = 0.0f;
    Chassis.body.dd_s = 0.0f;
    Chassis.body.dd_s_fused = 0.0f;
    Chassis_Slip_Init(&Chassis.slip);
    Chassis_Ground_Init(&Chassis.ground);
    Chassis_Turn_Init(&Chassis.turn);
    Chassis_Stuck_Init(&Chassis.stuck);
}

/**
 * @brief 清空实际发送量和请求量，用于主动零力或计算失败状态。
 */
void Chassis_Zero_Output(void)
{
    Output_Clear();
    Control_Reset();
}

/** @brief 初始化唯一底盘状态、目标、PID和速度融合器。 */
void Chassis_Init(void)
{
    uint32_t index;

    memset(&Chassis, 0, sizeof(Chassis));
    Chassis.mode = CHASSIS_MODE_ZERO_FORCE;
    Chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
    Chassis.state = CHASSIS_ZERO_FORCE;
    Chassis.output.safe_flag = 1U;
    Chassis.dt = APP_CTRL_DT_S;
    Chassis.goal.d_s = 0.0f;
    Chassis.goal.d_y = 0.0f;
    Chassis.goal.d_fai = 0.0f;
    Chassis.goal.L0 = APP_RC_LEG_M;
    memcpy(Chassis.lqr.target,
           Chassis_Config.target,
           sizeof(Chassis.lqr.target));

    Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_LEFT]);
    Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_RIGHT]);
    Algorithm_PID_Init(&Chassis.roll_pid);
    Joint_Control_Reset();
    for (index = 0U; index < CHASSIS_LEG_COUNT; index++)
    {
        Chassis.leg[index].target_L0 =
            Chassis_Config.leg[index].target_L0;
        Chassis.leg[index].target_phi0 =
            Chassis_Config.recovery.bench_phi0;
    }
    Control_Reset();
}

/**
 * @brief 使用任务层保存的四个DM反馈更新左右腿五连杆状态。
 *
 * 本函数不检查电机online标志，离线调试时仍使用最后一次反馈计算。
 * 左右腿分别提交本轮已经定义的中间量，valid只描述数学完整性。
 */
void Chassis_Leg_Update(void)
{
    uint32_t side;

    /* 每条腿独立提交，便于Watch区分是哪一侧的数学计算不完整。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float last_phi0 = Chassis.leg[side].phi0_total;
        uint8_t phi1_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        /* 配置索引直接用作 dm_motor 下标，越界时不得继续访问数组。 */
        if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT))
        {
            Chassis.leg[CHASSIS_LEFT].valid_flag = 0U;
            Chassis.leg[CHASSIS_RIGHT].valid_flag = 0U;
            return;
        }

        VMC_State_Calc(&Chassis_Config.leg[side],
                       Chassis.dm_motor[phi1_index].position_rad,
                       Chassis.dm_motor[phi4_index].position_rad,
                       Chassis.dm_motor[phi1_index].speed_radps,
                       Chassis.dm_motor[phi4_index].speed_radps,
                       &Chassis.leg[side]);
        if (Chassis.leg[side].L0 > 0.0f)
        {
            /* phi0主值跨过+-pi时选择距离上一有效角最近的等价角。 */
            Chassis.leg[side].phi0_total =
                Algorithm_AngleNearestEquivalentRad(
                    Chassis.leg[side].phi0,
                    last_phi0);
        }
        else
        {
            /* 当前phi0无定义时保留上次连续角，避免恢复后丢失圈数。 */
            Chassis.leg[side].phi0_total = last_phi0;
        }
    }
}

/**
 * @brief 把外部mode转换成内部state，并维护输出故障和站立保护。
 *
 * 该函数只选择本周期应执行的控制流程，不计算VMC、PID或LQR。设备
 * 和数学故障只封锁最终输出；姿态越界仍会切入零力状态。
 */
void Chassis_State_Update(void)
{
    float theta_b;
    uint8_t leg_valid_flag;
    uint32_t output_fault;
    uint32_t active_fault;
    uint32_t last_output_fault;

    /* 1. 外部主动零力具有最高优先级，不进入任何闭环控制。 */
    if (Chassis.mode == CHASSIS_MODE_ZERO_FORCE)
    {
        State_Enter(CHASSIS_ZERO_FORCE);
        Chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
        Chassis.fault = CHASSIS_FAULT_NONE;
        return;
    }

    /* 2. 更新输出故障，并在故障全部清除的边沿重置动态控制状态。 */
    theta_b = Chassis.imu.pitch *
                     Chassis_Config.imu.pitch_angle_scale;
    leg_valid_flag =
        ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
         (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;
    output_fault = Output_Fault_Get();
    active_fault = output_fault;
    if (leg_valid_flag == 0U)
    {
        active_fault |= CHASSIS_FAULT_KINEMATICS;
    }
    last_output_fault =
        Chassis.fault & CHASSIS_OUTPUT_FAULT_MASK;
    if ((last_output_fault != CHASSIS_FAULT_NONE) &&
        (active_fault == CHASSIS_FAULT_NONE))
    {
        /*
         * 输出重新放行前丢弃故障期间由陈旧反馈积累的动态状态，首个
         * 完整控制周期从当前姿态重新建立速度、积分和 yaw 平衡点。
         */
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_LEFT]);
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_RIGHT]);
        Algorithm_PID_Init(&Chassis.roll_pid);
        Joint_Control_Reset();
        Control_Reset();
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_FAI] =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    }

    /* 3. 仅在外部模式变化时选择入口状态，避免每周期重复初始化。 */
    if (Chassis.mode != Chassis.last_mode)
    {
        Chassis.fault = active_fault;
        switch (Chassis.mode)
        {
        case CHASSIS_MODE_TOP:
            Chassis.top_fai =
                Chassis.imu.yaw_total *
                Chassis_Config.imu.yaw_angle_scale;
            Chassis.lqr.target[CHASSIS_STATE_FAI] =
                Chassis.top_fai;
            /* TOP和FOLLOW进入站立的姿态条件相同。 */
            /* fall through */

        case CHASSIS_MODE_FOLLOW:
            if (Chassis.state != CHASSIS_STANDING)
            {
                /*
                 * 能否直接进入站立：左右腿正解有效、pitch较小且两条
                 * 虚拟腿都位于准备角区间；正解无效时也放行，中间量继
                 * 续计算，最终输出由故障门封锁。
                 */
                if ((leg_valid_flag == 0U) ||
                    ((fabsf(theta_b) <=
                      Chassis_Config.recovery.direct_pitch) &&
                     (Angle_In_Range(
                          Chassis.leg[CHASSIS_LEFT].phi0_total,
                          Chassis_Config.recovery.phi0_min,
                          Chassis_Config.recovery.phi0_max) != 0U) &&
                     (Angle_In_Range(
                          Chassis.leg[CHASSIS_RIGHT].phi0_total,
                          Chassis_Config.recovery.phi0_min,
                          Chassis_Config.recovery.phi0_max) != 0U)))
                {
                    State_Enter(CHASSIS_STANDING);
                }
                else
                {
                    Chassis.fault =
                        output_fault | CHASSIS_FAULT_CONTROL;
                    State_Enter(CHASSIS_ZERO_FORCE);
                }
            }
            break;

        case CHASSIS_MODE_SELF_SAVE:
            State_Enter(CHASSIS_FALLEN);
            break;

        case CHASSIS_MODE_BENCH:
            State_Enter(CHASSIS_BENCH);
            break;

        case CHASSIS_MODE_STEP:
            State_Enter(CHASSIS_STEP);
            break;

        case CHASSIS_MODE_ZERO_FORCE:
        default:
            State_Enter(CHASSIS_ZERO_FORCE);
            break;
        }
        Chassis.last_mode = Chassis.mode;
    }

    /* 4. 数学有效时才用当前腿角执行站立姿态保护。 */
    if ((Chassis.state == CHASSIS_STANDING) &&
        (leg_valid_flag != 0U) &&
        ((fabsf(theta_b) >
          Chassis_Config.recovery.pitch_limit) ||
          (Angle_In_Range(
               Chassis.leg[CHASSIS_LEFT].phi0_total,
               Chassis_Config.recovery.stand_phi0_min,
               Chassis_Config.recovery.stand_phi0_max) == 0U) ||
          (Angle_In_Range(
               Chassis.leg[CHASSIS_RIGHT].phi0_total,
               Chassis_Config.recovery.stand_phi0_min,
               Chassis_Config.recovery.stand_phi0_max) == 0U)))
    {
        Chassis.fault = output_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
    }

    /*
     * 5. 台阶动作腿摆角本来就大，用放宽后的限值单独做姿态保护。
     * 参考HERO_LEG磕台阶模式抬高倒地判断阈值的做法，避免越台阶瞬间误判，
     * 同时不让台阶状态完全失去翻倒保护。
     */
    if ((Chassis.state == CHASSIS_STEP) &&
        (leg_valid_flag != 0U) &&
        ((fabsf(theta_b) > Chassis_Config.step.pitch_limit) ||
         (Angle_In_Range(
              Chassis.leg[CHASSIS_LEFT].phi0_total,
              Chassis_Config.step.phi0_min,
              Chassis_Config.step.phi0_max) == 0U) ||
         (Angle_In_Range(
              Chassis.leg[CHASSIS_RIGHT].phi0_total,
              Chassis_Config.step.phi0_min,
              Chassis_Config.step.phi0_max) == 0U)))
    {
        Chassis.fault = output_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
    }

    /* 活动状态保留设备和数学故障，供末端输出门和Watch共同使用。 */
    if (Chassis.state != CHASSIS_ZERO_FORCE)
    {
        Chassis.fault = active_fault;
    }
}

/**
 * @brief 板凳和恢复模式的关节角度-速度串级控制。
 *
 * 逆运动学给出主动关节角目标，角度PID生成速度目标，速度PID生成
 * 力矩请求。输出故障只阻止请求量复制到最终关节力矩数组。
 *
 * 故障统一记录在 Chassis.fault；关节映射或逆解无定义时自行切入零力状态，
 * 后续轮输出阶段会因为 Chassis.fault 非零而自然封锁。
 */
static void Joint_Control(void)
{
    VMC_Joint_Target_t joint_target[CHASSIS_LEG_COUNT];
    float geometric_torque_nm;
    float target_speed_radps;
    float feedback_angle_rad;
    float feedback_speed_radps;
    float dt = Chassis.dt;
    uint32_t side;
    uint32_t joint;

    /* 1. 输出故障只参与末端许可，当前串级控制仍继续计算请求量。 */
    Chassis.fault = Output_Fault_Get();
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
    }

    /* 2. 目标腿长和连续phi0经逆运动学转换为四个关节目标角。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        uint8_t phi1_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        /* 配置索引直接用作关节数组下标，越界时不得继续访问数组。 */
        if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT) ||
            (VMC_Inverse_Calc(&Chassis_Config.leg[side],
                              &Chassis.leg[side],
                              Chassis.leg[side].target_L0,
                              Chassis.leg[side].target_phi0,
                              &joint_target[side]) == 0U))
        {
            Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Chassis.output.target_angle[phi1_index] = joint_target[side].phi1;
        Chassis.output.target_angle[phi4_index] = joint_target[side].phi4;
    }

    /* 3. 关节角度环输出目标速度，速度环输出有方向的力矩请求。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            uint8_t motor_index =
                Chassis_Config.leg[side].joint[joint].motor_index;
            float joint_scale =
                Chassis_Config.leg[side].joint[joint].scale *
                Chassis_Config.leg[side].joint[joint].ratio;

            if (joint == CHASSIS_JOINT_PHI1)
            {
                feedback_angle_rad = Chassis.leg[side].phi1;
            }
            else
            {
                feedback_angle_rad = Chassis.leg[side].phi4;
            }
            feedback_speed_radps =
                joint_scale * Chassis.dm_motor[motor_index].speed_radps;
            target_speed_radps = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &Chassis_Config.recovery.joint_angle_pid,
                &Chassis.joint_angle_pid[motor_index],
                Chassis.output.target_angle[motor_index],
                feedback_angle_rad,
                feedback_speed_radps,
                dt,
                &target_speed_radps);
            Chassis.output.target_speed[motor_index] =
                target_speed_radps;

            geometric_torque_nm = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &Chassis_Config.recovery.joint_speed_pid,
                &Chassis.joint_speed_pid[motor_index],
                target_speed_radps,
                feedback_speed_radps,
                0.0f,
                dt,
                &geometric_torque_nm);
            /* 请求量不限幅，限幅统一在最终命令处；PID输出限幅已界定量级。 */
            Chassis.output.T_joint_req[motor_index] =
                geometric_torque_nm * joint_scale;
        }

        /*
         * 叠加腿自重的重力前馈：位置串级只需处理误差，不必独自扛住整条腿，
         * 因此增益可以调小、动作变柔和。参考HERO_LEG自救时在F0上加补偿的做法，
         * 但那台车有气弹簧且前馈输入是实测支撑力，这里只按腿自重建模。
         * 重力沿伸腿方向拉F0、theta为正时给出负摆矩，前馈取相反号抵消。
         */
        if (Chassis_Config.recovery.gravity_ff_scale != 0.0f)
        {
            const Chassis_Model_Config_t *model = &Chassis_Config.model;
            float leg_weight_n = model->leg_mass * model->gravity *
                                 Chassis_Config.recovery.gravity_ff_scale;
            float cm_arm_m = Chassis_Config.recovery.leg_cm_ratio *
                             Chassis.leg[side].L0;
            VMC_Torque_t gravity_torque;

            if (VMC_Torque_Calc(&Chassis_Config.leg[side],
                                &Chassis.leg[side],
                                -leg_weight_n * cosf(Chassis.leg[side].theta),
                                leg_weight_n * cm_arm_m *
                                    sinf(Chassis.leg[side].theta),
                                &gravity_torque) != 0U)
            {
                uint8_t phi1_motor =
                    Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1]
                        .motor_index;
                uint8_t phi4_motor =
                    Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4]
                        .motor_index;

                Chassis.output.T_joint_req[phi1_motor] += gravity_torque.T1;
                Chassis.output.T_joint_req[phi4_motor] += gravity_torque.T4;
            }
        }
    }

    /* 4. 仅在全部输出条件满足时，把调试请求限幅后复制到最终命令。 */
    if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.joint_flag != 0U) &&
        (Chassis_Config.output.joint_T_limit > 0.0f))
    {
        for (side = 0U; side < APP_DM_COUNT; side++)
        {
            Chassis.output.T_joint[side] =
                Algorithm_LimitSymmetric(Chassis.output.T_joint_req[side],
                                Chassis_Config.output.joint_T_limit);
        }
        Chassis.output.safe_flag = 0U;
    }
}

/**
 * @brief 执行倒地转腿和小板凳准备两阶段重新站立状态机。
 */
void Chassis_Recovery(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    float theta_b;
    float theta[CHASSIS_LEG_COUNT];
    float rotate_rate[CHASSIS_LEG_COUNT];
    float direction;
    float theta_ref;
    uint8_t theta_flag[CHASSIS_LEG_COUNT];
    uint8_t prepare_flag;
    uint32_t side;
    float dt = Chassis.dt;

    Output_Clear();

    if ((Chassis.state != CHASSIS_FALLEN) &&
        (Chassis.state != CHASSIS_FALLING_TO_STAND))
    {
        Chassis_Zero_Output();
        return;
    }
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        /*
         * 当前腿姿态不足以推进恢复阶段时冻结阶段跳转，但仍运行已有目标下
         * 的关节串级控制，保留可定义请求量供Watch观察。
         * 计时必须继续：几何长期无解会让恢复永不超时，模式和状态一起锁死。
         */
        float stuck_timeout = (Chassis.state == CHASSIS_FALLEN) ?
                                  recovery->fallen_timeout :
                                  recovery->prepare_timeout;

        Chassis.state_time += dt;
        if (Chassis.state_time >= stuck_timeout)
        {
            Chassis_Zero_Output();
            Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Joint_Control();
        return;
    }

    theta_b = Chassis.imu.pitch *
                     Chassis_Config.imu.pitch_angle_scale;
    theta_ref = (recovery->theta_min + recovery->theta_max) * 0.5f;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        theta[side] = Algorithm_AngleNearestEquivalentRad(
            Chassis.leg[side].phi0_total -
                Chassis_Config.phi0_offset + theta_b,
            theta_ref);
    }

    if (Chassis.state == CHASSIS_FALLEN)
    {
        Chassis.state_time += dt;
        /* 倒地时姿态已经满足准备条件，直接跳过转腿阶段。 */
        if ((fabsf(theta_b) <= recovery->direct_pitch) &&
            (Angle_In_Range(
                 Chassis.leg[CHASSIS_LEFT].phi0_total,
                 recovery->phi0_min,
                 recovery->phi0_max) != 0U) &&
            (Angle_In_Range(
                 Chassis.leg[CHASSIS_RIGHT].phi0_total,
                 recovery->phi0_min,
                 recovery->phi0_max) != 0U))
        {
            State_Enter(CHASSIS_FALLING_TO_STAND);
        }
        else
        {
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                theta_flag[side] =
                    ((theta[side] >= recovery->theta_min) &&
                     (theta[side] <= recovery->theta_max)) ? 1U : 0U;
                rotate_rate[side] = recovery->rotate_rate;
            }
            /* 机体俯仰方向决定倒地后虚拟腿应向哪一侧翻转。 */
            direction = (theta_b < 0.0f) ? 1.0f : -1.0f;

            /*
             * 机体仍明显倾斜且双腿进度不一致时，让距离准备区间中心
             * 更远的一侧使用更大的追赶速率，避免一条腿提前停住。
             */
            if ((fabsf(theta[CHASSIS_LEFT] - theta[CHASSIS_RIGHT]) >
                  recovery->theta_diff) &&
                (fabsf(theta_b) > recovery->ready_pitch))
            {
                if (fabsf(theta[CHASSIS_LEFT] - theta_ref) >
                    fabsf(theta[CHASSIS_RIGHT] - theta_ref))
                {
                    rotate_rate[CHASSIS_LEFT] = recovery->lag_rate;
                }
                else
                {
                    rotate_rate[CHASSIS_RIGHT] = recovery->lag_rate;
                }
            }

            /*
             * 腿角到位且机体已接近可准备姿态时目标停在原处；否则从进入
             * 本状态时锁存的起点按速率继续推进。pitch 门槛防止严重倾斜时
             * 过早停止转腿。目标只按速率走，不跟随实际角，动作因此可控。
             */
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.leg[side].target_L0 =
                    Move_Toward(Chassis.leg[side].target_L0,
                                recovery->extend_L0,
                                recovery->L0_rate * dt);
                if ((theta_flag[side] == 0U) ||
                    (fabsf(theta_b) > recovery->direct_pitch))
                {
                    Chassis.leg[side].target_phi0 +=
                        direction * rotate_rate[side] * dt;
                }
                /* 腿被卡住时目标不能无限跑远，否则解卡瞬间会甩。 */
                Chassis.leg[side].target_phi0 = Algorithm_LimitRange(
                    Chassis.leg[side].target_phi0,
                    Chassis.leg[side].phi0_total - recovery->rotate_lead_max,
                    Chassis.leg[side].phi0_total + recovery->rotate_lead_max);
            }

            /* 双腿到位且 pitch 足够小并持续稳定后才进入板凳准备阶段。 */
            if ((theta_flag[CHASSIS_LEFT] != 0U) &&
                (theta_flag[CHASSIS_RIGHT] != 0U) &&
                (fabsf(theta_b) <= recovery->ready_pitch))
            {
                Chassis.stable_time += dt;
            }
            else
            {
                Chassis.stable_time = 0.0f;
            }

            if (Chassis.stable_time >= recovery->stable_time)
            {
                State_Enter(CHASSIS_FALLING_TO_STAND);
            }
            else if (Chassis.state_time >= recovery->fallen_timeout)
            {
                Chassis_Zero_Output();
                Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
                State_Enter(CHASSIS_ZERO_FORCE);
                return;
            }
            else
            {
                Joint_Control();
                return;
            }
        }
    }

    if (Chassis.state == CHASSIS_FALLING_TO_STAND)
    {
        Chassis.state_time += dt;
        prepare_flag = (fabsf(theta_b) <= recovery->ready_pitch) ? 1U : 0U;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            /* 腿长和腿角都按速率靠近板凳姿态，不再一步阶跃。 */
            Chassis.leg[side].target_L0 =
                Move_Toward(Chassis.leg[side].target_L0,
                            recovery->bench_L0,
                            recovery->L0_rate * dt);
            Chassis.leg[side].target_phi0 =
                Move_Toward(Chassis.leg[side].target_phi0,
                            Algorithm_AngleNearestEquivalentRad(
                                recovery->bench_phi0,
                                Chassis.leg[side].target_phi0),
                            recovery->rotate_rate * dt);
            Chassis.leg[side].target_phi0 = Algorithm_LimitRange(
                Chassis.leg[side].target_phi0,
                Chassis.leg[side].phi0_total - recovery->rotate_lead_max,
                Chassis.leg[side].phi0_total + recovery->rotate_lead_max);
            if ((fabsf(Chassis.leg[side].L0 - recovery->bench_L0) >
                 recovery->L0_tol) ||
                (fabsf(Algorithm_AngleNormalizeRad(
                     recovery->bench_phi0 - Chassis.leg[side].phi0_total)) >
                 recovery->angle_tol))
            {
                prepare_flag = 0U;
            }
        }
        if (prepare_flag != 0U)
        {
            Chassis.stable_time += dt;
        }
        else
        {
            Chassis.stable_time = 0.0f;
        }

        if (Chassis.stable_time >= recovery->stable_time)
        {
            State_Enter(CHASSIS_STANDING);
            return;
        }
        if (Chassis.state_time >= recovery->prepare_timeout)
        {
            Chassis_Zero_Output();
            Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Joint_Control();
    }
}

/**
 * @brief 保持板凳腿长/腿角，再复用主控制环计算轮LQR和关节位置环。
 *
 * 进入板凳时锁存当前实际姿态作为起点，此后按遥控模块给出的调节速率
 * 逐周期积分，腿长限制在机构可达区间内。
 */
void Chassis_Bench(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    uint32_t side;

    if (Chassis.state_time == 0.0f)
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_L0 = Chassis.leg[side].L0;
            Chassis.leg[side].target_phi0 = Chassis.leg[side].phi0_total;
        }
    }
    Chassis.state_time += Chassis.dt;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 +=
            Chassis.goal.bench_d_L0[side] * Chassis.dt;
        Chassis.leg[side].target_phi0 +=
            Chassis.goal.bench_d_phi0[side] * Chassis.dt;
        Chassis.leg[side].target_L0 =
            fminf(fmaxf(Chassis.leg[side].target_L0,
                        recovery->bench_L0_min),
                  recovery->bench_L0_max);
    }

    Chassis_Control();
}

/** @brief 按当前模式缩放十维误差并完成四路LQR统一点乘。 */
static void LQR_Calc(void)
{
    float *T[CHASSIS_OUTPUT_COUNT] = {
        &Chassis.output.T_wheel[CHASSIS_LEFT],
        &Chassis.output.T_wheel[CHASSIS_RIGHT],
        &Chassis.leg[CHASSIS_LEFT].Tp,
        &Chassis.leg[CHASSIS_RIGHT].Tp,
    };
    uint32_t output;
    uint32_t state;

    if ((Chassis.mode == CHASSIS_MODE_TOP) &&
        (Chassis.state == CHASSIS_STANDING))
    {
        memcpy(Chassis.lqr.scale,
               Chassis_Config.top.scale,
               sizeof(Chassis.lqr.scale));
    }
    else
    {
        for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
        {
            Chassis.lqr.scale[state] = 1.0f;
        }
    }

    // Chassis.lqr.scale[CHASSIS_STATE_FAI] = 0.0;
    // Chassis.lqr.scale[CHASSIS_STATE_D_FAI] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_S] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_S] = 1.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_THETA_L] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_THETA_L] = 0.0f;

    // Chassis.lqr.scale[CHASSIS_STATE_THETA_B] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_THETA_B] = 0.0f;
    /*
     * 误差先按状态限幅再进K点乘，防止位移积累或姿态瞬时越界时单一状态项
     * 主导四路输出。限幅只作用在误差输入端，不改K、不改状态顺序。
     */
    for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
    {
        float error = Chassis.lqr.target[state] - Chassis.lqr.x[state];
        float limit = Chassis_Config.lqr.error_limit[state];

        Chassis.lqr.error[state] =
            (limit > 0.0f) ? Algorithm_LimitSymmetric(error, limit) : error;
    }

    for (output = 0U; output < CHASSIS_OUTPUT_COUNT; output++)
    {
        float control_output = 0.0f;

        for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
        {
            control_output +=
                lqrK[output][state] *
                Chassis.lqr.error[state] *
                Chassis.lqr.scale[state];
        }
        *T[output] = control_output;
    }
}

/**
 * @brief 爬台阶摆腿阶段用角度PID覆盖LQR的两路虚拟腿摆力矩。
 *
 * CLIMB按HERO_LEG磕台阶控制分两段：先后摆蓄势再前摆越过台阶，最后归正。
 * 摆动段给固定力矩，摆过头才切回PID保持，避免撞机械限位。
 * RECOVER仍是单一腿摆角PID。
 */
static void Step_Leg_Control(float dt)
{
    const Chassis_Step_Config_t *config = &Chassis_Config.step;
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        /* 腿杆相对车体角，不含机体俯仰，对应HERO_LEG的abs_leg_theta。 */
        float phi0 = Chassis.leg[side].phi0_total - Chassis_Config.phi0_offset;
        float theta = Chassis.leg[side].theta;
        float output_nm = 0.0f;

        if (Chassis.step_phase != CHASSIS_STEP_CLIMB)
        {
            Algorithm_PID_UpdateByFeedbackRate(
                &config->leg_angle_pid,
                &Chassis.step_leg_angle_pid[side],
                config->recover_theta,
                theta,
                Chassis.leg[side].d_theta,
                dt,
                &output_nm);
            Chassis.leg[side].Tp = output_nm;
            continue;
        }

        switch (Chassis.swing[side])
        {
        case CHASSIS_SWING_BACK:
            if (fabsf(phi0) >= config->back_phi0_max)
            {
                Algorithm_PID_UpdateByFeedbackRate(
                    &config->leg_angle_pid,
                    &Chassis.step_leg_angle_pid[side],
                    config->back_phi0_hold,
                    phi0,
                    Chassis.leg[side].d_phi0,
                    dt,
                    &output_nm);
            }
            else
            {
                output_nm = config->back_Tp;
            }
            if (fabsf(theta) >= config->back_theta_exit)
            {
                Chassis.swing[side] = CHASSIS_SWING_FRONT;
            }
            break;

        case CHASSIS_SWING_FRONT:
            if (fabsf(theta) >= config->front_theta_max)
            {
                Algorithm_PID_UpdateByFeedbackRate(
                    &config->leg_angle_pid,
                    &Chassis.step_leg_angle_pid[side],
                    config->front_phi0_hold,
                    phi0,
                    Chassis.leg[side].d_phi0,
                    dt,
                    &output_nm);
            }
            else
            {
                output_nm = config->front_Tp;
            }
            if (theta <= config->front_theta_exit)
            {
                Chassis.swing[side] = CHASSIS_SWING_HOME;
            }
            break;

        case CHASSIS_SWING_HOME:
        default:
            Algorithm_PID_UpdateByFeedbackRate(
                &config->leg_angle_pid,
                &Chassis.step_leg_angle_pid[side],
                config->home_phi0,
                phi0,
                Chassis.leg[side].d_phi0,
                dt,
                &output_nm);
            break;
        }
        Chassis.leg[side].Tp = output_nm;
    }
}

/**
 * @brief 完成速度融合、十维状态、支撑力、LQR和VMC整条控制链。
 *
 * 函数开始先清最终命令；输出故障不阻断中间量计算，只在末端阶段
 * 阻止请求量进入最终T_joint和I_wheel。
 */
void Chassis_Control(void)
{
    float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT] = {0.0f};
    float dt;
    float theta_b;
    float d_theta_b;
    uint32_t side;

    Output_Clear();
    Chassis.lqr.limit_flag = 0U;

    /* 1. 输出故障只封锁最终命令，中间控制量继续使用最新反馈计算。 */
    Chassis.fault = Output_Fault_Get();
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
    }

    /* 配置轴号随后直接用作 imu.gyro/body_accel 下标，越界不得继续。 */
    if ((Chassis_Config.imu.pitch_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.roll_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.yaw_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.lateral_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.vertical_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.forward_accel_axis >= APP_IMU_AXIS_COUNT))
    {
        Chassis.fault |= CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
        return;
    }

    dt = Chassis.dt;
    if (Chassis.state == CHASSIS_STANDING)
    {
        Motion_Update();
    }

    /* 轮速、IMU 和腿部状态组成速度融合的两个测量量。 */
    theta_b = Chassis.imu.pitch *
                     Chassis_Config.imu.pitch_angle_scale;
    d_theta_b =
        Chassis.imu.gyro[Chassis_Config.imu.pitch_rate_axis] *
        Chassis_Config.imu.pitch_rate_scale;

    /*
     * phi0和pitch均绕车体+Y增大。虚拟腿相对地面的绝对角为
     * theta = phi0 - pi/2 + pitch；多圈phi0只用于保持几何连续。
     */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const float wheel_scale =
            (side == CHASSIS_LEFT) ?
                Chassis_Config.wheel.left_scale :
                Chassis_Config.wheel.right_scale;
        const Chassis_State_Index_t theta_index =
            (side == CHASSIS_LEFT) ?
                CHASSIS_STATE_THETA_L :
                CHASSIS_STATE_THETA_R;

        /* speed_rpm是电机转子转速，先除gear_ratio换算到轮轴角速度。 */
        Chassis.body.wheel_speed[side] =
            (float)Chassis.wheel_motor[side].speed_rpm *
            CHASSIS_RPM_TO_RADPS * wheel_scale /
            Chassis_Config.wheel.gear_ratio;
        Chassis.leg[side].theta = Algorithm_AngleNearestEquivalentRad(
            Chassis.leg[side].phi0_total -
                Chassis_Config.phi0_offset + theta_b,
            Chassis.lqr.target[theta_index]);
        Chassis.leg[side].d_theta =
            Chassis.leg[side].d_phi0 + d_theta_b;
        /*
         * ESC反馈是转子相对定子的速度。定子随虚拟腿转动，因此轮缘
         * 绝对速度还需加入R*d_theta，再叠加轮轴的腿部平动速度。
         */
        Chassis.body.side_speed[side] =
            Chassis_Config.wheel.R *
                (Chassis.body.wheel_speed[side] +
                 Chassis.leg[side].d_theta) +
            Chassis.leg[side].L0 * Chassis.leg[side].d_theta *
                cosf(Chassis.leg[side].theta) +
            Chassis.leg[side].d_L0 * sinf(Chassis.leg[side].theta);
    }
    Chassis.body.d_s_raw =
        0.5f * (Chassis.body.side_speed[CHASSIS_LEFT] +
                Chassis.body.side_speed[CHASSIS_RIGHT]);
    Chassis.body.dd_s =
        Chassis.imu.body_accel[Chassis_Config.imu.forward_accel_axis] *
        Chassis_Config.imu.forward_accel_scale;

    if (Chassis_Config.speed_kalman.enable_flag != 0U)
    {
        Chassis.speed_kalman.stateTransition[0] = 1.0f;
        Chassis.speed_kalman.stateTransition[1] = dt;
        Chassis.speed_kalman.stateTransition[2] = 0.0f;
        Chassis.speed_kalman.stateTransition[3] = 1.0f;
        measurement[0] = Chassis.body.d_s_raw;
        measurement[1] = Chassis.body.dd_s;
        Algorithm_Kalman_Update(&Chassis.speed_kalman, measurement);
        Chassis.body.d_s = Chassis.speed_kalman.state[0];
        Chassis.body.dd_s_fused = Chassis.speed_kalman.state[1];
    }
    else
    {
        Chassis.body.d_s = Chassis.body.d_s_raw;
        Chassis.body.dd_s_fused = Chassis.body.dd_s;
        Chassis.speed_kalman.state[0] = Chassis.body.d_s;
        Chassis.speed_kalman.state[1] = Chassis.body.dd_s_fused;
    }

    if ((fabsf(Chassis.lqr.target[CHASSIS_STATE_D_S]) <= 1.0e-4f) &&
        (Chassis_Config.speed_kalman.position_d_s_limit > 0.0f) &&
        (fabsf(Chassis.body.d_s) <=
         Chassis_Config.speed_kalman.position_d_s_limit))
    {
        Chassis.body.s += Chassis.body.d_s * dt;
    }
    else
    {
        Chassis.body.s = 0.0f;
    }

    Chassis.body.fai =
        Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    Chassis.body.d_fai =
        Chassis.imu.gyro[Chassis_Config.imu.yaw_rate_axis] *
        Chassis_Config.imu.yaw_rate_scale;

    /* 十维数组只表达模型接口，物理状态在 body 和 leg 中各有唯一所有者。 */
    Chassis.lqr.x[CHASSIS_STATE_S] = Chassis.body.s;
    Chassis.lqr.x[CHASSIS_STATE_D_S] = Chassis.body.d_s;
    Chassis.lqr.x[CHASSIS_STATE_FAI] = Chassis.body.fai;
    Chassis.lqr.x[CHASSIS_STATE_D_FAI] = Chassis.body.d_fai;
    Chassis.lqr.x[CHASSIS_STATE_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].theta;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].theta;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_B] = theta_b;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_B] = d_theta_b;
    /*
     * 四套只读观测，各自先算观测量再出判定。
     * 顺序有依赖：转向的转弯半径要先知道是否打滑，卡腿要先知道是否整车离地。
     */
    Chassis_Slip_Update(&Chassis_Config, &Chassis);
    Chassis_Slip_Calc(&Chassis_Config, &Chassis);
    Chassis_Ground_Update(&Chassis_Config, &Chassis);
    Chassis_Ground_Calc(&Chassis_Config, &Chassis);
    Chassis_Turn_Update(&Chassis_Config, &Chassis);
    Chassis_Turn_Calc(&Chassis_Config, &Chassis);
    Chassis_Stuck_Update(&Chassis_Config, &Chassis);
    Chassis_Stuck_Calc(&Chassis_Config, &Chassis);

    /* 3. 小板凳由关节位置环保持腿姿态，不再叠加腿长和横滚支撑力。 */
    if (Chassis.state == CHASSIS_BENCH)
    {
        Chassis.leg[CHASSIS_LEFT].F0 = 0.0f;
        Chassis.leg[CHASSIS_RIGHT].F0 = 0.0f;
    }
    else
    {
        float length_pid[CHASSIS_LEG_COUNT] = {0.0f, 0.0f};
        float length_force[CHASSIS_LEG_COUNT];
        float gravity_force[CHASSIS_LEG_COUNT];
        float roll =
            Chassis.imu.roll * Chassis_Config.imu.roll_angle_scale;
        float d_roll =
            Chassis.imu.gyro[Chassis_Config.imu.roll_rate_axis] *
            Chassis_Config.imu.roll_rate_scale;
        float roll_pid = 0.0f;
        float roll_force;

        /*腿长PID*/
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Algorithm_PID_UpdateByFeedbackRate(
                &Chassis_Config.leg_length_pid,
                &Chassis.leg_length_pid[side],
                Chassis.leg[side].target_L0,
                Chassis.leg[side].L0,
                Chassis.leg[side].d_L0,
                dt,
                &length_pid[side]);
            length_force[side] = length_pid[side];
        }
        /*横滚PID*/
        Algorithm_PID_UpdateByFeedbackRate(&Chassis_Config.roll_pid,
                                           &Chassis.roll_pid,
                                           Chassis_Config.roll_target,
                                           roll,
                                           d_roll,
                                           dt,
                                           &roll_pid);

        roll_force = roll_pid;
        /*
         * 重力前馈：整车重力在虚拟腿轴向的投影，单腿承担一半。
         * 腿摆得越靠前或靠后，轴向需要承担的分量越小，因此乘cos(theta)。
         * F0符号约定为正向撑地，gravity_force取负值后由下方减号还原成正贡献。
         */
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            gravity_force[side] =
                -0.5f * Chassis_Model_Mass(&Chassis_Config.model) *
                Chassis_Config.model.gravity *
                Chassis_Config.F0_gravity_scale *
                cosf(Chassis.leg[side].theta);
            // gravity_force[side] =
            //     -0.5f * Chassis_Config.model.body_mass *
            //     Chassis_Config.model.gravity *
            //     Chassis_Config.F0_gravity_scale *
            //     cosf(Chassis.leg[side].theta);
        }
        Chassis.leg[CHASSIS_LEFT].F0 =
            roll_force + 
            length_force[CHASSIS_LEFT]
             -
            gravity_force[CHASSIS_LEFT] +
            Chassis_Config.F0_left;
        Chassis.leg[CHASSIS_RIGHT].F0 =
            -roll_force + 
            length_force[CHASSIS_RIGHT]
             -
            gravity_force[CHASSIS_RIGHT] -
            Chassis_Config.F0_right;
    }

    /* 4. 始终根据左右实时腿长拟合K，固定目标腿长不伪造K输入。 */
    Algorithm_LQR_FitLqrKPoly22(
        Chassis_Config.lqr.coefficients,
        CHASSIS_OUTPUT_COUNT,
        CHASSIS_STATE_COUNT,
        Chassis.leg[CHASSIS_LEFT].L0,
        Chassis.leg[CHASSIS_RIGHT].L0,
        Chassis_Config.lqr.L0_min,
        Chassis_Config.lqr.L0_max,
        &lqrK[0][0],
        &Chassis.leg[CHASSIS_LEFT].K_L0_fit,
        &Chassis.leg[CHASSIS_RIGHT].K_L0_fit,
        &Chassis.lqr.limit_flag);

    /* 5. TOP只屏蔽位移和航向位置反馈，K矩阵和状态顺序保持不变。 */
    LQR_Calc();

    // Chassis.output.T_wheel[CHASSIS_LEFT] = 0.15f;
    // Chassis.output.T_wheel[CHASSIS_RIGHT] = 0.15f;

    if (Chassis.state == CHASSIS_BENCH)
    {
        Chassis.leg[CHASSIS_LEFT].Tp = 0.0f;
        Chassis.leg[CHASSIS_RIGHT].Tp = 0.0f;
    }
    if ((Chassis.state == CHASSIS_STEP) &&
        ((Chassis.step_phase == CHASSIS_STEP_CLIMB) ||
         (Chassis.step_phase == CHASSIS_STEP_RECOVER)))
    {
        Step_Leg_Control(dt);
    }

    /* 轮通道只在力矩侧限幅一次，电调命令由换算得到，不再二次限幅。 */
    if ((Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f))
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            float wheel_scale =
                (side == CHASSIS_LEFT) ?
                    Chassis_Config.wheel.left_scale :
                    Chassis_Config.wheel.right_scale;

            Chassis.output.I_wheel_req[side] = (int16_t)(
                Algorithm_LimitSymmetric(Chassis.output.T_wheel[side],
                                Chassis_Config.wheel.T_limit) *
                Chassis_Config.wheel.T_to_I * wheel_scale);
        }
    }

    /* 6. 板凳使用关节串级，其他状态由VMC生成四路关节请求。 */
    if (Chassis.state == CHASSIS_BENCH)
    {
        Joint_Control();
    }
    else
    {
        VMC_Torque_t torque[CHASSIS_LEG_COUNT] = {0};

        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            uint8_t phi1_index =
                Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
            uint8_t phi4_index =
                Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

            /* 配置索引直接用作关节数组下标，越界时不得继续访问数组。 */
            if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT))
            {
                Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
                State_Enter(CHASSIS_ZERO_FORCE);
                return;
            }
            if (VMC_Torque_Calc(&Chassis_Config.leg[side],
                                &Chassis.leg[side],
                                Chassis.leg[side].F0,
                                Chassis.leg[side].Tp,
                                &torque[side]) == 0U)
            {
                Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
            }
            Chassis.output.T_joint_req[phi1_index] = torque[side].T1;
            Chassis.output.T_joint_req[phi4_index] = torque[side].T4;
        }

        /* 7. VMC请求经总开关、分路开关和限幅后进入最终关节命令。 */
        if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
            (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
            (Chassis_Config.output.joint_flag != 0U) &&
            (Chassis_Config.output.joint_T_limit > 0.0f))
        {
            for (side = 0U; side < APP_DM_COUNT; side++)
            {
                Chassis.output.T_joint[side] =
                    Algorithm_LimitSymmetric(Chassis.output.T_joint_req[side],
                                    Chassis_Config.output.joint_T_limit);
            }
            Chassis.output.safe_flag = 0U;
        }
    }

    /* 8. 轮请求在全部输出条件满足时进入最终CAN电流数组。 */
    if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.wheel_flag != 0U) &&
        (Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f))
    {
        memcpy(Chassis.output.I_wheel,
               Chassis.output.I_wheel_req,
               sizeof(Chassis.output.I_wheel));
        Chassis.output.safe_flag = 0U;
    }
}

/** @brief 切换爬台阶阶段并清空该阶段的计时和摆腿PID状态。 */
static void Step_Phase_Enter(Chassis_Step_Phase_t phase)
{
    uint32_t side;

    Chassis.step_phase = phase;
    Chassis.state_time = 0.0f;
    Chassis.stable_time = 0.0f;
    if ((phase == CHASSIS_STEP_CLIMB) ||
        (phase == CHASSIS_STEP_RECOVER))
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Algorithm_PID_Init(&Chassis.step_leg_angle_pid[side]);
            /* 两段摆腿每次进入CLIMB都从后摆重新开始，RECOVER不使用子步。 */
            if (phase == CHASSIS_STEP_CLIMB)
            {
                Chassis.swing[side] = CHASSIS_SWING_BACK;
            }
        }
    }
}

/** @brief 爬台阶阶段超时后封锁全部输出并保留专用故障位。 */
static void Step_Fail(void)
{
    Chassis_Zero_Output();
    Chassis.fault = CHASSIS_FAULT_STEP_TIMEOUT;
    State_Enter(CHASSIS_ZERO_FORCE);
}

/** @brief 更新左右轮碰撞候选，只有请求、反馈和腿角同时满足才锁存。 */
static void Step_Contact_Update(float dt)
{
    const Chassis_Step_Config_t *config = &Chassis_Config.step;
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float request_torque_nm =
            fabsf(Chassis.output.T_wheel[side]);
        float feedback_torque_nm = 0.0f;

        if (fabsf(Chassis_Config.wheel.T_to_I) > 1.0e-6f)
        {
            feedback_torque_nm =
                fabsf((float)Chassis.wheel_motor[side].current /
                      Chassis_Config.wheel.T_to_I);
        }
        Chassis.step_contact_flag[side] =
            ((request_torque_nm >= config->contact_T_req) &&
             (feedback_torque_nm >= config->contact_T_fb) &&
             (fabsf(Chassis.leg[side].theta) >=
              config->contact_theta)) ? 1U : 0U;

        if (Chassis.step_contact_latch_flag[side] != 0U)
        {
            continue;
        }
        if (Chassis.step_contact_flag[side] != 0U)
        {
            Chassis.contact_time[side] += dt;
            if (Chassis.contact_time[side] >=
                config->contact_time)
            {
                Chassis.step_contact_latch_flag[side] = 1U;
            }
        }
        else
        {
            Chassis.contact_time[side] = 0.0f;
        }
    }
}

/**
 * @brief 执行抬高、接近、收腿摆动和姿态恢复四阶段辅助爬台阶。
 *
 * 接触锁存后只清零I_wheel，LQR轮力矩和I_wheel_req继续
 * 更新，便于在Watch中判断碰撞条件和后续控制请求。
 */
void Chassis_Step(void)
{
    const Chassis_Step_Config_t *config = &Chassis_Config.step;
    float dt = Chassis.dt;
    float target_length_m;
    float target_speed_mps = 0.0f;
    float target_angle_rad = 0.0f;
    uint32_t side;

    if (Chassis.state != CHASSIS_STEP)
    {
        Chassis_Zero_Output();
        return;
    }

    target_length_m = config->approach_L0;
    if (Chassis.step_phase == CHASSIS_STEP_APPROACH)
    {
        target_speed_mps = Algorithm_LimitSymmetric(
            Chassis.goal.d_s,
            config->approach_d_s);
        if (target_speed_mps < 0.0f)
        {
            target_speed_mps = 0.0f;
        }
    }
    else if (Chassis.step_phase == CHASSIS_STEP_CLIMB)
    {
        /* CLIMB的腿摆由两段摆腿直接给Tp，十维腿角目标保持零。 */
        target_length_m = config->retract_L0;
    }
    else if (Chassis.step_phase == CHASSIS_STEP_RECOVER)
    {
        target_length_m = Chassis.goal.L0;
        target_angle_rad = config->recover_theta;
    }

    Chassis.state_time += dt;
    Chassis.lqr.target[CHASSIS_STATE_D_S] =
        Move_Toward(Chassis.lqr.target[CHASSIS_STATE_D_S],
                           target_speed_mps,
                           APP_RC_VEL_RATE * dt);
    Chassis.lqr.target[CHASSIS_STATE_FAI] = Chassis.step_fai;
    Chassis.lqr.target[CHASSIS_STATE_D_FAI] = 0.0f;
    Chassis.lqr.target[CHASSIS_STATE_THETA_L] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_THETA_R] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_D_THETA_L] = 0.0f;
    Chassis.lqr.target[CHASSIS_STATE_D_THETA_R] = 0.0f;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 =
            Move_Toward(
                Chassis.leg[side].target_L0,
                target_length_m,
                Chassis_Config.recovery.L0_rate * dt);
    }

    Chassis_Control();
    if (Chassis.state != CHASSIS_STEP)
    {
        return;
    }

    switch (Chassis.step_phase)
    {
    case CHASSIS_STEP_PREPARE:
        if ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                   config->approach_L0) <=
             config->L0_tol) &&
            (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                   config->approach_L0) <=
             config->L0_tol))
        {
            Chassis.stable_time += dt;
        }
        else
        {
            Chassis.stable_time = 0.0f;
        }
        if (Chassis.stable_time >= config->stable_time)
        {
            Step_Phase_Enter(CHASSIS_STEP_APPROACH);
        }
        else if (Chassis.state_time >= config->prepare_timeout)
        {
            Step_Fail();
        }
        break;

    case CHASSIS_STEP_APPROACH:
        Step_Contact_Update(dt);
        if ((Chassis.step_contact_latch_flag[CHASSIS_LEFT] != 0U) &&
            (Chassis.step_contact_latch_flag[CHASSIS_RIGHT] != 0U))
        {
            Step_Phase_Enter(CHASSIS_STEP_CLIMB);
        }
        else if (Chassis.state_time >= config->approach_timeout)
        {
            Step_Fail();
        }
        break;

    case CHASSIS_STEP_CLIMB:
        /* 双腿都走完后摆和前摆并进入归正，才认为已经越过台阶。 */
        if ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                   config->retract_L0) <=
             config->L0_tol) &&
            (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                   config->retract_L0) <=
             config->L0_tol) &&
            (Chassis.swing[CHASSIS_LEFT] == CHASSIS_SWING_HOME) &&
            (Chassis.swing[CHASSIS_RIGHT] == CHASSIS_SWING_HOME))
        {
            Chassis.stable_time += dt;
        }
        else
        {
            Chassis.stable_time = 0.0f;
        }
        if (Chassis.stable_time >= config->stable_time)
        {
            Step_Phase_Enter(CHASSIS_STEP_RECOVER);
        }
        else if (Chassis.state_time >= config->climb_timeout)
        {
            Step_Fail();
        }
        break;

    case CHASSIS_STEP_RECOVER:
        if ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                   target_length_m) <=
             config->L0_tol) &&
            (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                   target_length_m) <=
             config->L0_tol) &&
            (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_L] -
                   config->recover_theta) <=
             config->angle_tol) &&
            (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_R] -
                   config->recover_theta) <=
             config->angle_tol))
        {
            Chassis.stable_time += dt;
        }
        else
        {
            Chassis.stable_time = 0.0f;
        }
        if (Chassis.stable_time >= config->stable_time)
        {
            State_Enter(CHASSIS_STANDING);
        }
        else if (Chassis.state_time >= config->recover_timeout)
        {
            Step_Fail();
        }
        break;

    default:
        Step_Fail();
        break;
    }

    if ((Chassis.state == CHASSIS_STEP) &&
        ((Chassis.step_phase == CHASSIS_STEP_CLIMB) ||
         (Chassis.step_phase == CHASSIS_STEP_RECOVER)))
    {
        memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    }
}
