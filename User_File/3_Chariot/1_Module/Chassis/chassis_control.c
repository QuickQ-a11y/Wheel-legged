#include "chassis_control.h"

#include "Angle.h"
#include "LQR.h"
#include "PID.h"

#include <math.h>
#include <string.h>

#define CHASSIS_RPM_TO_RADPS 0.10471975512f
#define CHASSIS_OUTPUT_FAULT_MASK                                         \
    (CHASSIS_FAULT_DISABLED | CHASSIS_FAULT_IMU |                       \
     CHASSIS_FAULT_DM_MOTOR | CHASSIS_FAULT_DJI_MOTOR | CHASSIS_FAULT_CAN | \
     CHASSIS_FAULT_KINEMATICS | CHASSIS_FAULT_REMOTE)

Chassis_t Chassis;

static void Control_Reset(void);

/** @brief 按给定绝对值对称限制标量，非正限幅直接返回零。 */
static float Limit_Symmetric(float value, float limit)
{
    float positive_limit = fabsf(limit);

    if ((!isfinite(value)) || (!isfinite(positive_limit)) ||
        (positive_limit <= 0.0f))
    {
        return 0.0f;
    }
    if (value > positive_limit)
    {
        return positive_limit;
    }
    if (value < -positive_limit)
    {
        return -positive_limit;
    }
    return value;
}

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
 * @brief 将遥控运动目标按当前控制周期写入正常站立目标。
 *
 * 速度和航向目标使用独立变化率限制；航向目标的差分同时作为十维
 * LQR的目标角速度，避免只改变角度而产生不一致的参考轨迹。
 */
static void Motion_Update(void)
{
    float dt = Chassis.dt;
    float model_yaw_rad;
    float top_phase_rad;
    float yaw_step_rad;

    if (Chassis.remote_target_flag == 0U)
    {
        Chassis.lqr.target[CHASSIS_STATE_DOT_S] =
            Move_Toward(Chassis.lqr.target[CHASSIS_STATE_DOT_S],
                               0.0f,
                               APP_RC_VEL_RATE * dt);
        Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] =
            Move_Toward(
                Chassis.lqr.target[CHASSIS_STATE_DOT_FAI],
                0.0f,
                APP_RC_YAW_RATE * dt);
        return;
    }

    if (Chassis.mode == CHASSIS_MODE_TOP)
    {
        model_yaw_rad = Chassis.imu.yaw_total *
                        Chassis_Config.imu.yaw_angle_scale;
        top_phase_rad = model_yaw_rad - Chassis.top_fai;
        Chassis.top_d_s =
            Chassis.goal.d_s * cosf(top_phase_rad) +
            Chassis.goal.d_y * sinf(top_phase_rad);
        Chassis.lqr.target[CHASSIS_STATE_DOT_S] =
            Move_Toward(Chassis.lqr.target[CHASSIS_STATE_DOT_S],
                               Chassis.top_d_s,
                               APP_RC_VEL_RATE * dt);
        Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
        Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] =
            Move_Toward(
                Chassis.lqr.target[CHASSIS_STATE_DOT_FAI],
                Chassis.goal.d_fai,
                APP_RC_YAW_RATE * dt);
    }
    else
    {
        Chassis.top_d_s = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_DOT_S] =
            Move_Toward(Chassis.lqr.target[CHASSIS_STATE_DOT_S],
                               Chassis.goal.d_s,
                               APP_RC_VEL_RATE * dt);

        yaw_step_rad = Limit_Symmetric(
            Chassis.goal.fai -
                Chassis.lqr.target[CHASSIS_STATE_FAI],
            APP_RC_YAW_RATE * dt);
        Chassis.lqr.target[CHASSIS_STATE_FAI] += yaw_step_rad;
        if (dt > 0.0f)
        {
            Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] =
                yaw_step_rad / dt;
        }
    }

    Chassis.leg[CHASSIS_LEFT].target_L0 =
        Move_Toward(Chassis.leg[CHASSIS_LEFT].target_L0,
                           Chassis.goal.L0,
                           Chassis_Config.recovery.L0_rate *
                               dt);
    Chassis.leg[CHASSIS_RIGHT].target_L0 =
        Move_Toward(Chassis.leg[CHASSIS_RIGHT].target_L0,
                           Chassis.goal.L0,
                           Chassis_Config.recovery.L0_rate *
                               dt);
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

/** @brief 读取左右腿phi1和phi4关节的DM数组索引并检查数组边界。 */
static uint8_t Joint_Index_Get(
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT])
{
    uint32_t side;
    uint32_t joint;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            indices[side][joint] =
                Chassis_Config.leg[side].joint[joint].motor_index;
            if (indices[side][joint] >= APP_DM_COUNT)
            {
                return 0U;
            }
        }
    }
    return 1U;
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
            break;
        }
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        if (Chassis.wheel_motor[index].online_flag == 0U)
        {
            fault |= CHASSIS_FAULT_DJI_MOTOR;
            break;
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
 * @brief 在输出故障清除前重置所有可能积累陈旧反馈的动态状态。
 */
static void Dynamic_Control_Reset(void)
{
    Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_LEFT]);
    Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_RIGHT]);
    Algorithm_PID_Init(&Chassis.roll_pid);
    Joint_Control_Reset();
    Control_Reset();
}

/**
 * @brief 切换内部控制状态并初始化该状态所需目标和控制器。
 *
 * 相同状态不重复进入，避免每周期清空计时器、PID和目标斜坡。
 */
static void State_Enter(Chassis_State_t state)
{
    uint8_t leg_valid_flag;
    uint32_t side;

    if (Chassis.state == state)
    {
        return;
    }

    Chassis.state = state;
    Chassis.state_time = 0.0f;
    Chassis.stable_time = 0.0f;
    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    Chassis.output.safe_flag = 1U;
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
        leg_valid_flag =
            ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
             (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_LEFT]);
        Algorithm_PID_Init(&Chassis.leg_length_pid[CHASSIS_RIGHT]);
        Algorithm_PID_Init(&Chassis.roll_pid);
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_L0 =
                (leg_valid_flag != 0U) ?
                    Chassis.leg[side].L0 :
                    Chassis_Config.leg[side].target_L0;
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
    Chassis_Observer_Init(&Chassis.observer);
}

/**
 * @brief 清空实际发送量和请求量，用于主动零力或计算失败状态。
 */
void Chassis_Zero_Output(void)
{
    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    Chassis.output.safe_flag = 1U;
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
    Chassis.goal.fai = 0.0f;
    Chassis.goal.L0 = APP_RC_LEG_M;
    Chassis.goal.fai_anchor = 0.0f;
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
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];
    uint32_t side;

    if (Joint_Index_Get(indices) == 0U)
    {
        Chassis.leg[CHASSIS_LEFT].valid_flag = 0U;
        Chassis.leg[CHASSIS_RIGHT].valid_flag = 0U;
        return;
    }

    /* 每条腿独立提交，便于Watch区分是哪一侧的数学计算不完整。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float last_phi0 = Chassis.leg[side].phi0_total;

        VMC_State_Calc(&Chassis_Config.leg[side],
                       Chassis.dm_motor[indices[side][CHASSIS_JOINT_PHI1]]
                           .position_rad,
                       Chassis.dm_motor[indices[side][CHASSIS_JOINT_PHI4]]
                           .position_rad,
                       Chassis.dm_motor[indices[side][CHASSIS_JOINT_PHI1]]
                           .speed_radps,
                        Chassis.dm_motor[indices[side][CHASSIS_JOINT_PHI4]]
                            .speed_radps,
                        &Chassis.leg[side]);
        if ((Chassis.leg[side].L0 > 0.0f) &&
            isfinite(Chassis.leg[side].phi0))
        {
            if (isfinite(last_phi0))
            {
                /* phi0主值跨过+-pi时选择距离上一有效角最近的等价角。 */
                Chassis.leg[side].phi0_total =
                    Algorithm_AngleNearestEquivalentRad(
                        Chassis.leg[side].phi0,
                        last_phi0);
            }
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
    uint8_t posture_flag;
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
        Dynamic_Control_Reset();
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_FAI] =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    }
    /*
     * 3. posture_flag只判断FOLLOW/TOP能否直接进入站立控制：
     * 左右腿正解有效、pitch较小且两条虚拟腿都位于准备角区间。
     */
    posture_flag =
        ((leg_valid_flag != 0U) &&
         (fabsf(theta_b) <=
          Chassis_Config.recovery.direct_pitch) &&
         (Angle_In_Range(
              Chassis.leg[CHASSIS_LEFT].phi0_total,
              Chassis_Config.recovery.phi0_min,
              Chassis_Config.recovery.phi0_max) != 0U) &&
         (Angle_In_Range(
              Chassis.leg[CHASSIS_RIGHT].phi0_total,
              Chassis_Config.recovery.phi0_min,
              Chassis_Config.recovery.phi0_max) != 0U)) ? 1U : 0U;

    /* 4. 仅在外部模式变化时选择入口状态，避免每周期重复初始化。 */
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
                if ((posture_flag != 0U) || (leg_valid_flag == 0U))
                {
                    /* 数学无效时仍计算中间量，最终输出由故障门封锁。 */
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

    /* 5. 数学有效时才用当前腿角执行站立姿态保护。 */
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
 */
static uint8_t Joint_Control(uint32_t active_fault)
{
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];
    VMC_Joint_Target_t joint_target[CHASSIS_LEG_COUNT];
    float geometric_torque_nm;
    float target_speed_radps;
    float feedback_angle_rad;
    float feedback_speed_radps;
    float output_limit_nm;
    float dt = Chassis.dt;
    uint32_t side;
    uint32_t joint;

    /* 1. 每周期先清最终关节命令，防止任何提前返回遗留旧力矩。 */
    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    Chassis.output.safe_flag = 1U;

    /* 2. 输出故障只参与末端许可，当前串级控制仍继续计算请求量。 */
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        active_fault |= CHASSIS_FAULT_KINEMATICS;
    }
    Chassis.fault = active_fault | CHASSIS_FAULT_CONTROL;
    if (Joint_Index_Get(indices) == 0U)
    {
        Chassis.fault = active_fault | CHASSIS_FAULT_KINEMATICS;
        return 0U;
    }
    if ((dt < Chassis_Config.dt_min) ||
        (dt > Chassis_Config.dt_max))
    {
        dt = Chassis_Config.default_dt;
    }

    /* 3. 目标腿长和连续phi0经逆运动学转换为四个关节目标角。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (VMC_Inverse_Calc(&Chassis_Config.leg[side],
                             &Chassis.leg[side],
                             Chassis.leg[side].target_L0,
                             Chassis.leg[side].target_phi0,
                             &joint_target[side]) == 0U)
        {
            Chassis.fault = active_fault | CHASSIS_FAULT_KINEMATICS;
            return 0U;
        }
        Chassis.output.target_angle
            [indices[side][CHASSIS_JOINT_PHI1]] =
                joint_target[side].phi1;
        Chassis.output.target_angle
            [indices[side][CHASSIS_JOINT_PHI4]] =
                joint_target[side].phi4;
    }

    /* 4. 关节角度环输出目标速度，速度环输出有方向的力矩请求。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            uint8_t motor_index = indices[side][joint];
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
            if (!isfinite(feedback_speed_radps))
            {
                feedback_speed_radps = 0.0f;
            }
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
            Chassis.output.T_joint_req[motor_index] =
                Limit_Symmetric(
                    geometric_torque_nm * joint_scale,
                    Chassis_Config.recovery.joint_T_limit);
        }
    }

    /* 5. 仅在全部输出条件满足时，把调试请求复制到最终命令。 */
    if ((active_fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.joint_flag != 0U) &&
        (Chassis_Config.output.joint_T_limit > 0.0f) &&
        (Chassis_Config.recovery.joint_T_limit > 0.0f))
    {
        output_limit_nm = Chassis_Config.output.joint_T_limit;
        if (Chassis_Config.recovery.joint_T_limit < output_limit_nm)
        {
            output_limit_nm = Chassis_Config.recovery.joint_T_limit;
        }
        for (side = 0U; side < APP_DM_COUNT; side++)
        {
            Chassis.output.T_joint[side] =
                Limit_Symmetric(Chassis.output.T_joint_req[side],
                                output_limit_nm);
        }
        Chassis.output.safe_flag = 0U;
    }

    Chassis.fault = active_fault;
    return 1U;
}

/**
 * @brief 执行倒地转腿和小板凳准备两阶段重新站立状态机。
 */
void Chassis_Recovery(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    float theta_b;
    float theta[CHASSIS_LEG_COUNT];
    float rotate_phi0[CHASSIS_LEG_COUNT];
    float direction;
    float theta_ref;
    uint8_t theta_flag[CHASSIS_LEG_COUNT];
    uint8_t direct_flag;
    uint8_t prepare_flag;
    uint8_t leg_valid_flag;
    uint32_t side;
    float dt = Chassis.dt;

    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    Chassis.output.T_wheel[CHASSIS_LEFT] = 0.0f;
    Chassis.output.T_wheel[CHASSIS_RIGHT] = 0.0f;

    if ((dt < Chassis_Config.dt_min) ||
        (dt > Chassis_Config.dt_max))
    {
        dt = Chassis_Config.default_dt;
    }
    if ((Chassis.state != CHASSIS_FALLEN) &&
        (Chassis.state != CHASSIS_FALLING_TO_STAND))
    {
        Chassis_Zero_Output();
        return;
    }
    leg_valid_flag =
        ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
         (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;
    if (leg_valid_flag == 0U)
    {
        /*
         * 当前腿姿态不足以推进恢复阶段时冻结计时和跳转，但仍运行已有
         * 目标下的关节串级控制，保留可定义请求量供Watch观察。
         */
        if (Joint_Control(Output_Fault_Get()) == 0U)
        {
            State_Enter(CHASSIS_ZERO_FORCE);
        }
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
        direct_flag =
            ((fabsf(theta_b) <= recovery->direct_pitch) &&
             (Angle_In_Range(
                  Chassis.leg[CHASSIS_LEFT].phi0_total,
                  recovery->phi0_min,
                  recovery->phi0_max) != 0U) &&
             (Angle_In_Range(
                  Chassis.leg[CHASSIS_RIGHT].phi0_total,
                  recovery->phi0_min,
                  recovery->phi0_max) != 0U)) ? 1U : 0U;
        if (direct_flag != 0U)
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
                rotate_phi0[side] = recovery->rotate_phi0;
            }
            /* 机体俯仰方向决定倒地后虚拟腿应向哪一侧翻转。 */
            direction = (theta_b < 0.0f) ? 1.0f : -1.0f;

            /*
             * 机体仍明显倾斜且双腿进度不一致时，让距离准备区间中心
             * 更远的一侧使用更大的经验追赶量，避免一条腿提前停住。
             */
            if ((fabsf(theta[CHASSIS_LEFT] - theta[CHASSIS_RIGHT]) >
                  recovery->theta_diff) &&
                (fabsf(theta_b) > recovery->ready_pitch))
            {
                if (fabsf(theta[CHASSIS_LEFT] - theta_ref) >
                    fabsf(theta[CHASSIS_RIGHT] - theta_ref))
                {
                    rotate_phi0[CHASSIS_LEFT] = recovery->lag_phi0;
                }
                else
                {
                    rotate_phi0[CHASSIS_RIGHT] = recovery->lag_phi0;
                }
            }

            /*
             * 腿角到位且机体已接近可准备姿态时保持当前连续角；否则继续
             * 沿恢复方向转动。pitch 门槛防止严重倾斜时过早停止转腿。
             */
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.leg[side].target_L0 = recovery->extend_L0;
                Chassis.leg[side].target_phi0 =
                    ((theta_flag[side] != 0U) &&
                     (fabsf(theta_b) <= recovery->direct_pitch)) ?
                        Chassis.leg[side].phi0_total :
                        Chassis.leg[side].phi0_total +
                            direction * rotate_phi0[side];
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
                if (Joint_Control(Output_Fault_Get()) == 0U)
                {
                    State_Enter(CHASSIS_ZERO_FORCE);
                }
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
            Chassis.leg[side].target_L0 = recovery->bench_L0;
            Chassis.leg[side].target_phi0 =
                Algorithm_AngleNearestEquivalentRad(
                    recovery->bench_phi0,
                    Chassis.leg[side].phi0_total);
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
        if (Joint_Control(Output_Fault_Get()) == 0U)
        {
            State_Enter(CHASSIS_ZERO_FORCE);
        }
    }
}

/**
 * @brief 设置固定板凳腿长/腿角，再复用主控制环计算轮LQR和关节位置环。
 */
void Chassis_Bench(void)
{
    uint32_t side;

    Chassis.state_time += Chassis.dt;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 = Chassis_Config.recovery.bench_L0;
        Chassis.leg[side].target_phi0 =
            Algorithm_AngleNearestEquivalentRad(
                Chassis_Config.recovery.bench_phi0,
                Chassis.leg[side].phi0_total);
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

    for (output = 0U; output < CHASSIS_OUTPUT_COUNT; output++)
    {
        float control_output = 0.0f;

        for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
        {
            control_output +=
                Chassis.lqr.K[output][state] *
                (Chassis.lqr.target[state] - Chassis.lqr.x[state]) *
                Chassis.lqr.scale[state];
        }
        *T[output] = control_output;
    }
}

/** @brief 爬台阶摆腿阶段用角度PID覆盖LQR的两路虚拟腿摆力矩。 */
static void Step_Leg_Control(float dt)
{
    float target_angle_rad;
    float output_nm[CHASSIS_LEG_COUNT] = {0.0f, 0.0f};

    target_angle_rad =
        (Chassis.step_phase == CHASSIS_STEP_CLIMB) ?
            Chassis_Config.step.peak_theta :
            Chassis_Config.step.recover_theta;
    Algorithm_PID_UpdateByFeedbackRate(
        &Chassis_Config.step.leg_angle_pid,
        &Chassis.step_leg_angle_pid[CHASSIS_LEFT],
        target_angle_rad,
        Chassis.leg[CHASSIS_LEFT].theta,
        Chassis.leg[CHASSIS_LEFT].d_theta,
        dt,
        &output_nm[CHASSIS_LEFT]);
    Algorithm_PID_UpdateByFeedbackRate(
        &Chassis_Config.step.leg_angle_pid,
        &Chassis.step_leg_angle_pid[CHASSIS_RIGHT],
        target_angle_rad,
        Chassis.leg[CHASSIS_RIGHT].theta,
        Chassis.leg[CHASSIS_RIGHT].d_theta,
        dt,
        &output_nm[CHASSIS_RIGHT]);
    Chassis.leg[CHASSIS_LEFT].Tp =
        output_nm[CHASSIS_LEFT];
    Chassis.leg[CHASSIS_RIGHT].Tp =
        output_nm[CHASSIS_RIGHT];
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
    uint32_t active_fault;

    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
    Chassis.output.safe_flag = 1U;
    Chassis.lqr.limit_flag = 0U;

    /* 1. 输出故障只封锁最终命令，中间控制量继续使用最新反馈计算。 */
    active_fault = Output_Fault_Get();
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        active_fault |= CHASSIS_FAULT_KINEMATICS;
    }
    Chassis.fault = active_fault | CHASSIS_FAULT_CONTROL;

    /* 配置边界属于实机安全底线，运行期不再封装额外的状态查询函数。 */
    if ((Chassis_Config.imu.pitch_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.roll_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.yaw_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.forward_accel_axis >= APP_IMU_AXIS_COUNT))
    {
        Chassis.fault = active_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
        return;
    }
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const Chassis_Geometry_Config_t *geometry =
            &Chassis_Config.leg[side].geometry;

        if ((geometry->l1 <= 0.0f) || (geometry->l2 <= 0.0f) ||
            (geometry->l3 <= 0.0f) || (geometry->l4 <= 0.0f))
        {
            Chassis.fault = active_fault | CHASSIS_FAULT_CONTROL;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
    }

    if ((Chassis_Config.lqr.L0_min <= 0.0f) ||
        (Chassis_Config.lqr.L0_max <
         Chassis_Config.lqr.L0_min))
    {
        Chassis.fault = active_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
        return;
    }

    dt = Chassis.dt;
    if (Chassis.state == CHASSIS_STANDING)
    {
        Motion_Update();
        if (Chassis.remote_target_flag == 0U)
        {
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.leg[side].target_L0 =
                    Move_Toward(
                        Chassis.leg[side].target_L0,
                        Chassis_Config.leg[side].target_L0,
                        Chassis_Config.recovery.L0_rate * dt);
            }
        }
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

        Chassis.body.wheel_speed[side] =
            (float)Chassis.wheel_motor[side].speed_rpm *
            CHASSIS_RPM_TO_RADPS * wheel_scale;
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

    if ((fabsf(Chassis.lqr.target[CHASSIS_STATE_DOT_S]) <= 1.0e-4f) &&
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
    Chassis.body.theta = theta_b;
    Chassis.body.d_theta = d_theta_b;

    /* 十维数组只表达模型接口，物理状态在 body 和 leg 中各有唯一所有者。 */
    Chassis.lqr.x[CHASSIS_STATE_S] = Chassis.body.s;
    Chassis.lqr.x[CHASSIS_STATE_DOT_S] = Chassis.body.d_s;
    Chassis.lqr.x[CHASSIS_STATE_FAI] = Chassis.body.fai;
    Chassis.lqr.x[CHASSIS_STATE_DOT_FAI] = Chassis.body.d_fai;
    Chassis.lqr.x[CHASSIS_STATE_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].theta;
    Chassis.lqr.x[CHASSIS_STATE_DOT_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].theta;
    Chassis.lqr.x[CHASSIS_STATE_DOT_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_B] = Chassis.body.theta;
    Chassis.lqr.x[CHASSIS_STATE_DOT_THETA_B] = Chassis.body.d_theta;
    Chassis_Observer_Update(&Chassis_Config, &Chassis);

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
            length_force[side] = -length_pid[side];
        }
        /*横滚PID*/
        Algorithm_PID_UpdateByFeedbackRate(&Chassis_Config.roll_pid,
                                           &Chassis.roll_pid,
                                           Chassis_Config.roll_target,
                                           roll,
                                           d_roll,
                                           dt,
                                           &roll_pid);

        roll_force = -roll_pid;
        Chassis.leg[CHASSIS_LEFT].F0 =
            -roll_force + length_force[CHASSIS_LEFT] +
            Chassis_Config.F0_base +
            Chassis_Config.F0_left;
        Chassis.leg[CHASSIS_RIGHT].F0 =
            roll_force + length_force[CHASSIS_RIGHT] +
            Chassis_Config.F0_base -
            Chassis_Config.F0_right;
    }

    /* 4. 始终根据左右实时腿长拟合K，固定目标腿长不伪造K输入。 */
    Algorithm_LQR_FitLqrKPoly22(
        &Chassis_Config.lqr.coefficients[0][0][0],
        CHASSIS_OUTPUT_COUNT,
        CHASSIS_STATE_COUNT,
        Chassis.leg[CHASSIS_LEFT].L0,
        Chassis.leg[CHASSIS_RIGHT].L0,
        Chassis_Config.lqr.L0_min,
        Chassis_Config.lqr.L0_max,
        &Chassis.lqr.K[0][0],
        &Chassis.leg[CHASSIS_LEFT].K_L0_fit,
        &Chassis.leg[CHASSIS_RIGHT].K_L0_fit,
        &Chassis.lqr.limit_flag);

    /* 5. TOP只屏蔽位移和航向位置反馈，K矩阵和状态顺序保持不变。 */
    LQR_Calc();
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

    if ((Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f) &&
        (Chassis_Config.wheel.I_limit > 0))
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            float wheel_scale =
                (side == CHASSIS_LEFT) ?
                    Chassis_Config.wheel.left_scale :
                    Chassis_Config.wheel.right_scale;

            Chassis.output.I_wheel_req[side] =
                (int16_t)Limit_Symmetric(
                    Limit_Symmetric(Chassis.output.T_wheel[side],
                                    Chassis_Config.wheel.T_limit) *
                        Chassis_Config.wheel.T_to_I *
                        wheel_scale,
                    (float)Chassis_Config.wheel.I_limit);
        }
    }

    /* 6. 板凳使用关节串级，其他状态由VMC生成四路关节请求。 */
    if (Chassis.state == CHASSIS_BENCH)
    {
        if (Joint_Control(active_fault) == 0U)
        {
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
    }
    else
    {
        VMC_Torque_t torque[CHASSIS_LEG_COUNT] = {0};
        uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];

        if (Joint_Index_Get(indices) == 0U)
        {
            Chassis.fault = active_fault | CHASSIS_FAULT_KINEMATICS;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            if (VMC_Torque_Calc(&Chassis_Config.leg[side],
                                &Chassis.leg[side],
                                Chassis.leg[side].F0,
                                Chassis.leg[side].Tp,
                                &torque[side]) == 0U)
            {
                active_fault |= CHASSIS_FAULT_KINEMATICS;
            }
            Chassis.output.T_joint_req
                [indices[side][CHASSIS_JOINT_PHI1]] = torque[side].T1;
            Chassis.output.T_joint_req
                [indices[side][CHASSIS_JOINT_PHI4]] = torque[side].T4;
        }

        /* 7. VMC请求经总开关、分路开关和限幅后进入最终关节命令。 */
        if ((active_fault == CHASSIS_FAULT_NONE) &&
            (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
            (Chassis_Config.output.joint_flag != 0U) &&
            (Chassis_Config.output.joint_T_limit > 0.0f))
        {
            for (side = 0U; side < APP_DM_COUNT; side++)
            {
                Chassis.output.T_joint[side] =
                    Limit_Symmetric(Chassis.output.T_joint_req[side],
                                    Chassis_Config.output.joint_T_limit);
            }
            Chassis.output.safe_flag = 0U;
        }
    }

    /* 8. 轮请求在全部输出条件满足时进入最终CAN电流数组。 */
    if ((active_fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.wheel_flag != 0U) &&
        (Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f) &&
        (Chassis_Config.wheel.I_limit > 0))
    {
        memcpy(Chassis.output.I_wheel,
               Chassis.output.I_wheel_req,
               sizeof(Chassis.output.I_wheel));
        Chassis.output.safe_flag = 0U;
    }

    Chassis.fault = active_fault;
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
    uint8_t phase_flag = 0U;
    uint32_t side;

    if (Chassis.state != CHASSIS_STEP)
    {
        Chassis_Zero_Output();
        return;
    }
    if ((dt < Chassis_Config.dt_min) ||
        (dt > Chassis_Config.dt_max))
    {
        dt = Chassis_Config.default_dt;
    }

    target_length_m = config->approach_L0;
    if (Chassis.step_phase == CHASSIS_STEP_APPROACH)
    {
        target_speed_mps = Limit_Symmetric(
            Chassis.goal.d_s,
            config->approach_d_s);
        if (target_speed_mps < 0.0f)
        {
            target_speed_mps = 0.0f;
        }
    }
    else if (Chassis.step_phase == CHASSIS_STEP_CLIMB)
    {
        target_length_m = config->retract_L0;
        target_angle_rad = config->peak_theta;
    }
    else if (Chassis.step_phase == CHASSIS_STEP_RECOVER)
    {
        target_length_m = Chassis.goal.L0;
        target_angle_rad = config->recover_theta;
    }

    Chassis.state_time += dt;
    Chassis.lqr.target[CHASSIS_STATE_DOT_S] =
        Move_Toward(Chassis.lqr.target[CHASSIS_STATE_DOT_S],
                           target_speed_mps,
                           APP_RC_VEL_RATE * dt);
    Chassis.lqr.target[CHASSIS_STATE_FAI] = Chassis.step_fai;
    Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] = 0.0f;
    Chassis.lqr.target[CHASSIS_STATE_THETA_L] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_THETA_R] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_DOT_THETA_L] = 0.0f;
    Chassis.lqr.target[CHASSIS_STATE_DOT_THETA_R] = 0.0f;
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
        phase_flag =
            ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                    config->approach_L0) <=
              config->L0_tol) &&
             (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                    config->approach_L0) <=
              config->L0_tol)) ? 1U : 0U;
        if (phase_flag != 0U)
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
        phase_flag =
            ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                    config->retract_L0) <=
              config->L0_tol) &&
             (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                    config->retract_L0) <=
              config->L0_tol) &&
             (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_L] -
                    config->peak_theta) <=
              config->angle_tol) &&
             (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_R] -
                    config->peak_theta) <=
              config->angle_tol)) ? 1U : 0U;
        if (phase_flag != 0U)
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
        phase_flag =
            ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
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
              config->angle_tol)) ? 1U : 0U;
        if (phase_flag != 0U)
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
