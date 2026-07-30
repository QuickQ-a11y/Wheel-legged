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
     CHASSIS_FAULT_KINEMATICS)

chassis_t chassis;

/** @brief 按给定绝对值对称限制标量，非正限幅直接返回零。 */
static float Chassis_LimitSymmetric(float value, float limit)
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
static float Chassis_MoveToward(float value, float target, float maximum_step)
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

/** @brief 判断周期角是否存在落在指定连续区间内的等价角。 */
static uint8_t Chassis_IsAngleInIntervalRad(float angle_rad,
                                        float minimum_rad,
                                        float maximum_rad)
{
    float center_rad = (minimum_rad + maximum_rad) * 0.5f;
    float equivalent_rad =
        Algorithm_AngleNearestEquivalentRad(angle_rad, center_rad);

    return ((equivalent_rad >= minimum_rad) &&
            (equivalent_rad <= maximum_rad)) ? 1U : 0U;
}

/** @brief 读取左右腿前后关节的DM数组索引并检查数组边界。 */
static uint8_t Chassis_GetJointIndices(
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT])
{
    uint32_t side;
    uint32_t joint;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            indices[side][joint] =
                chassis_config.leg[side].joint[joint].motor_index;
            if (indices[side][joint] >= APP_DM_COUNT)
            {
                return 0U;
            }
        }
    }
    return 1U;
}

/** @brief 判断左右腿本轮位置和速度解算是否都具有数学定义。 */
static uint8_t Chassis_AreLegStatesValid(void)
{
    return ((chassis.leg[CHASSIS_LEFT].valid != 0U) &&
            (chassis.leg[CHASSIS_RIGHT].valid != 0U)) ? 1U : 0U;
}

/**
 * @brief 汇总只影响最终电机输出许可的故障位。
 *
 * 这些故障不会阻断VMC、PID或LQR中间量计算；真正的几何和控制计算
 * 故障由对应控制流程另外追加到fault_flags。
 */
static uint32_t Chassis_GetOutputFaults(void)
{
    uint32_t fault_flags = CHASSIS_FAULT_NONE;
    uint32_t index;

    if (chassis.enabled == 0U)
    {
        fault_flags |= CHASSIS_FAULT_DISABLED;
    }
    if ((chassis.imu.initialized == 0U) ||
        (chassis.imu.attitude_ready == 0U) ||
        (chassis.imu.error_code != 0U))
    {
        fault_flags |= CHASSIS_FAULT_IMU;
    }
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        if (chassis.dm_motor[index].online == 0U)
        {
            fault_flags |= CHASSIS_FAULT_DM_MOTOR;
            break;
        }
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        if (chassis.wheel_motor[index].online == 0U)
        {
            fault_flags |= CHASSIS_FAULT_DJI_MOTOR;
            break;
        }
    }
    if (chassis.can_tx_error_count > APP_CAN_TX_ERROR_MAX)
    {
        fault_flags |= CHASSIS_FAULT_CAN;
    }
    return fault_flags;
}

/** @brief 清空四个关节串级PID状态、目标和调试请求量。 */
static void Chassis_ResetJointControl(void)
{
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        Algorithm_PID_Init(&chassis.joint_angle_pid[index]);
        Algorithm_PID_Init(&chassis.joint_speed_pid[index]);
    }
    memset(chassis.target_joint_angle_rad,
           0,
           sizeof(chassis.target_joint_angle_rad));
    memset(chassis.target_joint_speed_radps,
           0,
           sizeof(chassis.target_joint_speed_radps));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
}

/**
 * @brief 在输出故障清除前重置所有可能积累陈旧反馈的动态状态。
 */
static void Chassis_ResetDynamicControl(void)
{
    Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_LEFT]);
    Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_RIGHT]);
    Algorithm_PID_Init(&chassis.roll_pid);
    Chassis_ResetJointControl();
    Chassis_ControlReset();
}

/**
 * @brief 切换内部控制状态并初始化该状态所需目标和控制器。
 *
 * 相同状态不重复进入，避免每周期清空计时器、PID和目标斜坡。
 */
static void Chassis_EnterState(chassis_control_state_t state)
{
    uint32_t side;

    if (chassis.state == state)
    {
        return;
    }

    chassis.state = state;
    chassis.state_elapsed_s = 0.0f;
    chassis.state_stable_s = 0.0f;
    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.wheel_current_request,
           0,
           sizeof(chassis.wheel_current_request));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;
    Chassis_ResetJointControl();

    if ((state == CHASSIS_STANDING) || (state == CHASSIS_BENCH))
    {
        Chassis_ControlReset();
        memcpy(chassis.target_state,
               chassis_config.target_state,
               sizeof(chassis.target_state));
        chassis.target_state[CHASSIS_STATE_S] = 0.0f;
        chassis.target_state[CHASSIS_STATE_FAI] =
            chassis.imu.yaw_total_rad * chassis_config.imu.yaw_angle_scale;
    }

    if (state == CHASSIS_STANDING)
    {
        Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_LEFT]);
        Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_RIGHT]);
        Algorithm_PID_Init(&chassis.roll_pid);
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            chassis.target_leg_length_m[side] =
                (Chassis_AreLegStatesValid() != 0U) ?
                    chassis.leg[side].length_m :
                    chassis_config.leg[side].target_leg_length_m;
        }
    }
}

/** @brief 重新初始化速度Kalman、前进速度、加速度和位移状态。 */
void Chassis_ControlReset(void)
{
    Algorithm_Kalman_Init(&chassis.speed_kalman, 2U, 2U);
    memcpy(chassis.speed_kalman.covariance,
           chassis_config.speed_kalman.initial_covariance,
           sizeof(chassis_config.speed_kalman.initial_covariance));
    memcpy(chassis.speed_kalman.processNoise,
           chassis_config.speed_kalman.process_noise,
           sizeof(chassis_config.speed_kalman.process_noise));
    memcpy(chassis.speed_kalman.measurementNoise,
           chassis_config.speed_kalman.measurement_noise,
           sizeof(chassis_config.speed_kalman.measurement_noise));

    chassis.speed_kalman.stateTransition[0] = 1.0f;
    chassis.speed_kalman.stateTransition[1] = APP_CTRL_DT_S;
    chassis.speed_kalman.stateTransition[2] = 0.0f;
    chassis.speed_kalman.stateTransition[3] = 1.0f;
    chassis.speed_kalman.measurementMatrix[0] = 1.0f;
    chassis.speed_kalman.measurementMatrix[1] = 0.0f;
    chassis.speed_kalman.measurementMatrix[2] = 0.0f;
    chassis.speed_kalman.measurementMatrix[3] = 1.0f;

    chassis.forward_position_m = 0.0f;
    chassis.forward_speed_raw_mps = 0.0f;
    chassis.forward_speed_mps = 0.0f;
    chassis.forward_accel_mps2 = 0.0f;
    chassis.forward_accel_fused_mps2 = 0.0f;
}

/**
 * @brief 清空实际发送量和请求量，用于主动零力或计算失败状态。
 */
void Chassis_ZeroOutput(void)
{
    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.wheel_current_request,
           0,
           sizeof(chassis.wheel_current_request));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    memset(chassis.target_joint_speed_radps,
           0,
           sizeof(chassis.target_joint_speed_radps));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;
    Chassis_ControlReset();
}

/** @brief 初始化唯一底盘状态、目标、PID和速度融合器。 */
void Chassis_ControlInit(void)
{
    uint32_t index;

    memset(&chassis, 0, sizeof(chassis));
    chassis.mode = CHASSIS_MODE_ZERO_FORCE;
    chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
    chassis.state = CHASSIS_ZERO_FORCE;
    chassis.safe_output = 1U;
    chassis.control_dt_s = APP_CTRL_DT_S;
    memcpy(chassis.target_state,
           chassis_config.target_state,
           sizeof(chassis.target_state));

    Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_LEFT]);
    Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_RIGHT]);
    Algorithm_PID_Init(&chassis.roll_pid);
    Chassis_ResetJointControl();
    for (index = 0U; index < CHASSIS_LEG_COUNT; index++)
    {
        chassis.target_leg_length_m[index] =
            chassis_config.leg[index].target_leg_length_m;
        chassis.target_leg_phi0_rad[index] =
            chassis_config.recovery.bench_phi0_rad;
    }
    Chassis_ControlReset();
}

/**
 * @brief 使用任务层保存的四个DM反馈更新左右腿五连杆状态。
 *
 * 本函数不检查电机online标志，离线调试时仍使用最后一次反馈计算。
 * 左右腿分别提交本轮已经定义的中间量，valid只描述数学完整性。
 */
void Chassis_ControlUpdateLegState(void)
{
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];
    chassis_vmc_state_t next_leg[CHASSIS_LEG_COUNT] = {0};
    chassis_vmc_state_t previous_leg[CHASSIS_LEG_COUNT];
    uint32_t side;

    /* 连续腿角属于跨周期状态；瞬时几何量始终使用本周期反馈重算。 */
    memcpy(previous_leg, chassis.leg, sizeof(previous_leg));
    if (Chassis_GetJointIndices(indices) == 0U)
    {
        chassis.leg[CHASSIS_LEFT].valid = 0U;
        chassis.leg[CHASSIS_RIGHT].valid = 0U;
        return;
    }

    /* 每条腿独立提交，便于Watch区分是哪一侧的数学计算不完整。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        VMC_CalcState(&chassis_config.leg[side],
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_FRONT]]
                           .position_rad,
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_BACK]]
                           .position_rad,
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_FRONT]]
                           .speed_radps,
                        chassis.dm_motor[indices[side][CHASSIS_JOINT_BACK]]
                            .speed_radps,
                        &next_leg[side]);
        if ((next_leg[side].length_m > 0.0f) &&
            isfinite(next_leg[side].phi0_rad))
        {
            if (isfinite(previous_leg[side].phi0_total_rad))
            {
                /* phi0主值跨过+-pi时选择距离上一有效角最近的等价角。 */
                next_leg[side].phi0_total_rad =
                    Algorithm_AngleNearestEquivalentRad(
                        next_leg[side].phi0_rad,
                        previous_leg[side].phi0_total_rad);
            }
        }
        else
        {
            /* 当前phi0无定义时保留上次连续角，避免恢复后丢失圈数。 */
            next_leg[side].phi0_total_rad =
                previous_leg[side].phi0_total_rad;
        }
        chassis.leg[side] = next_leg[side];
    }
}

/**
 * @brief 把外部mode转换成内部state，并维护输出故障和站立保护。
 *
 * 该函数只选择本周期应执行的控制流程，不计算VMC、PID或LQR。设备
 * 和数学故障只封锁最终输出；姿态越界仍会切入零力状态。
 */
void Chassis_ControlUpdateState(void)
{
    float body_pitch_rad;
    uint8_t posture_ready;
    uint8_t leg_states_valid;
    uint32_t output_faults;
    uint32_t active_faults;
    uint32_t previous_output_faults;

    /* 1. 外部主动零力具有最高优先级，不进入任何闭环控制。 */
    if (chassis.mode == CHASSIS_MODE_ZERO_FORCE)
    {
        Chassis_EnterState(CHASSIS_ZERO_FORCE);
        chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
        chassis.fault_flags = CHASSIS_FAULT_NONE;
        return;
    }

    /* 2. 更新输出故障，并在故障全部清除的边沿重置动态控制状态。 */
    body_pitch_rad = chassis.imu.pitch_rad *
                     chassis_config.imu.pitch_angle_scale;
    leg_states_valid = Chassis_AreLegStatesValid();
    output_faults = Chassis_GetOutputFaults();
    active_faults = output_faults;
    if (leg_states_valid == 0U)
    {
        active_faults |= CHASSIS_FAULT_KINEMATICS;
    }
    previous_output_faults =
        chassis.fault_flags & CHASSIS_OUTPUT_FAULT_MASK;
    if ((previous_output_faults != CHASSIS_FAULT_NONE) &&
        (active_faults == CHASSIS_FAULT_NONE))
    {
        /*
         * 输出重新放行前丢弃故障期间由陈旧反馈积累的动态状态，首个
         * 完整控制周期从当前姿态重新建立速度、积分和 yaw 平衡点。
         */
        Chassis_ResetDynamicControl();
        chassis.target_state[CHASSIS_STATE_S] = 0.0f;
        chassis.target_state[CHASSIS_STATE_FAI] =
            chassis.imu.yaw_total_rad * chassis_config.imu.yaw_angle_scale;
    }
    /*
     * 3. posture_ready只判断FOLLOW/TOP能否直接进入站立控制：
     * 左右腿正解有效、pitch较小且两条虚拟腿都位于准备角区间。
     */
    posture_ready =
        ((leg_states_valid != 0U) &&
         (fabsf(body_pitch_rad) <=
          chassis_config.recovery.direct_prepare_pitch_rad) &&
         (Chassis_IsAngleInIntervalRad(
              chassis.leg[CHASSIS_LEFT].phi0_total_rad,
              chassis_config.recovery.direct_phi0_min_rad,
              chassis_config.recovery.direct_phi0_max_rad) != 0U) &&
         (Chassis_IsAngleInIntervalRad(
              chassis.leg[CHASSIS_RIGHT].phi0_total_rad,
              chassis_config.recovery.direct_phi0_min_rad,
              chassis_config.recovery.direct_phi0_max_rad) != 0U)) ? 1U : 0U;

    /* 4. 仅在外部模式变化时选择入口状态，避免每周期重复初始化。 */
    if (chassis.mode != chassis.last_mode)
    {
        chassis.fault_flags = active_faults;
        switch (chassis.mode)
        {
        case CHASSIS_MODE_FOLLOW:
        case CHASSIS_MODE_TOP:
            if (chassis.state != CHASSIS_STANDING)
            {
                if ((posture_ready != 0U) || (leg_states_valid == 0U))
                {
                    /* 数学无效时仍进入站立计算链，但最终输出保持封锁。 */
                    Chassis_EnterState(CHASSIS_STANDING);
                }
                else
                {
                    chassis.fault_flags =
                        output_faults | CHASSIS_FAULT_CONTROL;
                    Chassis_EnterState(CHASSIS_ZERO_FORCE);
                }
            }
            break;

        case CHASSIS_MODE_SELF_SAVE:
            Chassis_EnterState(CHASSIS_FALLEN);
            break;

        case CHASSIS_MODE_BENCH:
            Chassis_EnterState(CHASSIS_BENCH);
            break;

        case CHASSIS_MODE_ZERO_FORCE:
        default:
            Chassis_EnterState(CHASSIS_ZERO_FORCE);
            break;
        }
        chassis.last_mode = chassis.mode;
    }

    /* 5. 数学有效时才用当前腿角执行站立姿态保护。 */
    if ((chassis.state == CHASSIS_STANDING) &&
        (leg_states_valid != 0U) &&
        ((fabsf(body_pitch_rad) >
          chassis_config.recovery.standing_pitch_limit_rad) ||
          (Chassis_IsAngleInIntervalRad(
               chassis.leg[CHASSIS_LEFT].phi0_total_rad,
               chassis_config.recovery.standing_phi0_min_rad,
               chassis_config.recovery.standing_phi0_max_rad) == 0U) ||
          (Chassis_IsAngleInIntervalRad(
               chassis.leg[CHASSIS_RIGHT].phi0_total_rad,
               chassis_config.recovery.standing_phi0_min_rad,
               chassis_config.recovery.standing_phi0_max_rad) == 0U)))
    {
        chassis.fault_flags = output_faults | CHASSIS_FAULT_CONTROL;
        Chassis_EnterState(CHASSIS_ZERO_FORCE);
    }

    /* 活动状态保留设备和数学故障，供末端输出门和Watch共同使用。 */
    if (chassis.state != CHASSIS_ZERO_FORCE)
    {
        chassis.fault_flags = active_faults;
    }
}

/**
 * @brief 板凳和恢复模式的关节角度-速度串级控制。
 *
 * 逆运动学给出主动关节角目标，角度PID生成速度目标，速度PID生成
 * 力矩请求。输出故障只阻止请求量复制到最终关节力矩数组。
 */
static uint8_t Chassis_JointPositionControl(uint8_t wheel_output_enabled)
{
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];
    chassis_vmc_joint_target_t joint_target[CHASSIS_LEG_COUNT];
    float geometric_torque_nm;
    float target_speed_radps;
    float feedback_angle_rad;
    float feedback_speed_radps;
    float output_limit_nm;
    float dt_s = chassis.control_dt_s;
    uint8_t joint_output_enabled;
    uint8_t leg_states_valid;
    uint32_t output_faults;
    uint32_t active_faults;
    uint32_t side;
    uint32_t joint;

    /* 1. 每周期先清最终关节命令，防止任何提前返回遗留旧力矩。 */
    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    memset(chassis.target_joint_speed_radps,
           0,
           sizeof(chassis.target_joint_speed_radps));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;

    /* 2. 输出故障只参与末端许可，当前串级控制仍继续计算请求量。 */
    output_faults = Chassis_GetOutputFaults();
    leg_states_valid = Chassis_AreLegStatesValid();
    active_faults = output_faults;
    if (leg_states_valid == 0U)
    {
        active_faults |= CHASSIS_FAULT_KINEMATICS;
        wheel_output_enabled = 0U;
    }
    chassis.fault_flags = active_faults | CHASSIS_FAULT_CONTROL;
    if (Chassis_GetJointIndices(indices) == 0U)
    {
        chassis.fault_flags = output_faults | CHASSIS_FAULT_KINEMATICS;
        return 0U;
    }
    if ((dt_s < chassis_config.min_dt_s) ||
        (dt_s > chassis_config.max_dt_s))
    {
        dt_s = chassis_config.default_dt_s;
    }

    /* 3. 目标腿长和连续phi0经逆运动学转换为四个关节目标角。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (VMC_CalcJointTarget(&chassis_config.leg[side],
                                  &chassis.leg[side],
                                  chassis.target_leg_length_m[side],
                                  chassis.target_leg_phi0_rad[side],
                                  &joint_target[side]) == 0U)
        {
            chassis.fault_flags = output_faults | CHASSIS_FAULT_KINEMATICS;
            return 0U;
        }
        chassis.target_joint_angle_rad
            [indices[side][CHASSIS_JOINT_FRONT]] =
                joint_target[side].phi1_rad;
        chassis.target_joint_angle_rad
            [indices[side][CHASSIS_JOINT_BACK]] =
                joint_target[side].phi4_rad;
    }

    /* 4. 关节角度环输出目标速度，速度环输出有方向的力矩请求。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            uint8_t motor_index = indices[side][joint];

            if (joint == CHASSIS_JOINT_FRONT)
            {
                feedback_angle_rad = chassis.leg[side].phi1_rad;
            }
            else
            {
                feedback_angle_rad = chassis.leg[side].phi4_rad;
            }
            feedback_speed_radps =
                chassis_config.leg[side].joint[joint].angle_scale *
                chassis.dm_motor[motor_index].speed_radps;
            if (!isfinite(feedback_speed_radps))
            {
                feedback_speed_radps = 0.0f;
            }
            target_speed_radps = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &chassis_config.recovery.joint_angle_pid,
                &chassis.joint_angle_pid[motor_index],
                chassis.target_joint_angle_rad[motor_index],
                feedback_angle_rad,
                feedback_speed_radps,
                dt_s,
                &target_speed_radps);
            chassis.target_joint_speed_radps[motor_index] =
                target_speed_radps;

            geometric_torque_nm = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &chassis_config.recovery.joint_speed_pid,
                &chassis.joint_speed_pid[motor_index],
                target_speed_radps,
                feedback_speed_radps,
                0.0f,
                dt_s,
                &geometric_torque_nm);
            chassis.joint_torque_request_nm[motor_index] =
                Chassis_LimitSymmetric(
                    geometric_torque_nm *
                        chassis_config.leg[side].joint[joint].torque_scale,
                    chassis_config.recovery.joint_torque_limit_nm);
        }
    }

    /* 5. 仅在全部输出条件满足时，把调试请求复制到最终命令。 */
    joint_output_enabled =
        ((active_faults == CHASSIS_FAULT_NONE) &&
         (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
         (chassis_config.output.joint_enabled != 0U) &&
         (chassis_config.output.joint_torque_limit_nm > 0.0f) &&
         (chassis_config.recovery.joint_torque_limit_nm > 0.0f)) ? 1U : 0U;
    if (joint_output_enabled != 0U)
    {
        output_limit_nm = chassis_config.output.joint_torque_limit_nm;
        if (chassis_config.recovery.joint_torque_limit_nm < output_limit_nm)
        {
            output_limit_nm = chassis_config.recovery.joint_torque_limit_nm;
        }
        for (side = 0U; side < APP_DM_COUNT; side++)
        {
            chassis.joint_torque_nm[side] =
                Chassis_LimitSymmetric(chassis.joint_torque_request_nm[side],
                                output_limit_nm);
        }
    }

    chassis.fault_flags = active_faults;
    chassis.safe_output =
        ((joint_output_enabled == 0U) && (wheel_output_enabled == 0U)) ? 1U : 0U;
    chassis.state_valid = 1U;
    return 1U;
}

/**
 * @brief 执行倒地转腿和小板凳准备两阶段重新站立状态机。
 */
void Chassis_RecoveryControlLoop(void)
{
    const chassis_recovery_config_t *recovery = &chassis_config.recovery;
    float body_pitch_rad;
    float left_theta_rad;
    float right_theta_rad;
    float left_rotate_offset_rad;
    float right_rotate_offset_rad;
    float rotate_direction;
    float ready_theta_center_rad;
    uint8_t left_theta_ready;
    uint8_t right_theta_ready;
    uint8_t direct_prepare;
    uint8_t prepare_ready;
    uint8_t leg_states_valid;
    float dt_s = chassis.control_dt_s;

    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.wheel_current_request,
           0,
           sizeof(chassis.wheel_current_request));
    chassis.lqr_output[CHASSIS_OUTPUT_LEFT_WHEEL] = 0.0f;
    chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_WHEEL] = 0.0f;

    if ((dt_s < chassis_config.min_dt_s) ||
        (dt_s > chassis_config.max_dt_s))
    {
        dt_s = chassis_config.default_dt_s;
    }
    if ((chassis.state != CHASSIS_FALLEN) &&
        (chassis.state != CHASSIS_FALLING_TO_STAND))
    {
        Chassis_ZeroOutput();
        return;
    }
    leg_states_valid = Chassis_AreLegStatesValid();
    if (leg_states_valid == 0U)
    {
        /*
         * 当前腿姿态不足以推进恢复阶段时冻结计时和跳转，但仍运行已有
         * 目标下的关节串级控制，保留可定义请求量供Watch观察。
         */
        if (Chassis_JointPositionControl(0U) == 0U)
        {
            Chassis_EnterState(CHASSIS_ZERO_FORCE);
        }
        return;
    }

    body_pitch_rad = chassis.imu.pitch_rad *
                     chassis_config.imu.pitch_angle_scale;
    ready_theta_center_rad =
        (recovery->ready_theta_min_rad +
         recovery->ready_theta_max_rad) * 0.5f;
    left_theta_rad = Algorithm_AngleNearestEquivalentRad(
        chassis.leg[CHASSIS_LEFT].phi0_total_rad -
            chassis_config.leg_vertical_offset_rad - body_pitch_rad,
        ready_theta_center_rad);
    right_theta_rad = Algorithm_AngleNearestEquivalentRad(
        chassis.leg[CHASSIS_RIGHT].phi0_total_rad -
            chassis_config.leg_vertical_offset_rad - body_pitch_rad,
        ready_theta_center_rad);

    if (chassis.state == CHASSIS_FALLEN)
    {
        chassis.state_elapsed_s += dt_s;
        direct_prepare =
            ((fabsf(body_pitch_rad) <= recovery->direct_prepare_pitch_rad) &&
             (Chassis_IsAngleInIntervalRad(
                  chassis.leg[CHASSIS_LEFT].phi0_total_rad,
                  recovery->direct_phi0_min_rad,
                  recovery->direct_phi0_max_rad) != 0U) &&
             (Chassis_IsAngleInIntervalRad(
                  chassis.leg[CHASSIS_RIGHT].phi0_total_rad,
                  recovery->direct_phi0_min_rad,
                  recovery->direct_phi0_max_rad) != 0U)) ? 1U : 0U;
        if (direct_prepare != 0U)
        {
            Chassis_EnterState(CHASSIS_FALLING_TO_STAND);
        }
        else
        {
            left_theta_ready =
                ((left_theta_rad >= recovery->ready_theta_min_rad) &&
                 (left_theta_rad <= recovery->ready_theta_max_rad)) ? 1U : 0U;
            right_theta_ready =
                ((right_theta_rad >= recovery->ready_theta_min_rad) &&
                 (right_theta_rad <= recovery->ready_theta_max_rad)) ? 1U : 0U;
            /* 机体俯仰方向决定倒地后虚拟腿应向哪一侧翻转。 */
            rotate_direction = (body_pitch_rad < 0.0f) ? 1.0f : -1.0f;
            left_rotate_offset_rad = recovery->rotate_offset_rad;
            right_rotate_offset_rad = recovery->rotate_offset_rad;

            /*
             * 机体仍明显倾斜且双腿进度不一致时，让距离准备区间中心
             * 更远的一侧使用更大的经验追赶量，避免一条腿提前停住。
             */
            if ((fabsf(left_theta_rad - right_theta_rad) >
                  recovery->leg_difference_threshold_rad) &&
                (fabsf(body_pitch_rad) > recovery->ready_pitch_rad))
            {
                if (fabsf(left_theta_rad - ready_theta_center_rad) >
                    fabsf(right_theta_rad - ready_theta_center_rad))
                {
                    left_rotate_offset_rad =
                        recovery->lagging_rotate_offset_rad;
                }
                else
                {
                    right_rotate_offset_rad =
                        recovery->lagging_rotate_offset_rad;
                }
            }

            chassis.target_leg_length_m[CHASSIS_LEFT] =
                recovery->extended_leg_length_m;
            chassis.target_leg_length_m[CHASSIS_RIGHT] =
                recovery->extended_leg_length_m;
            /*
             * 腿角到位且机体已接近可准备姿态时保持当前连续角；否则继续
             * 沿恢复方向转动。pitch 门槛防止严重倾斜时过早停止转腿。
             */
            chassis.target_leg_phi0_rad[CHASSIS_LEFT] =
                ((left_theta_ready != 0U) &&
                  (fabsf(body_pitch_rad) <=
                   recovery->direct_prepare_pitch_rad)) ?
                    chassis.leg[CHASSIS_LEFT].phi0_total_rad :
                    chassis.leg[CHASSIS_LEFT].phi0_total_rad +
                        rotate_direction * left_rotate_offset_rad;
            chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
                ((right_theta_ready != 0U) &&
                  (fabsf(body_pitch_rad) <=
                   recovery->direct_prepare_pitch_rad)) ?
                    chassis.leg[CHASSIS_RIGHT].phi0_total_rad :
                    chassis.leg[CHASSIS_RIGHT].phi0_total_rad +
                        rotate_direction * right_rotate_offset_rad;

            /* 双腿到位且 pitch 足够小并持续稳定后才进入板凳准备阶段。 */
            if ((left_theta_ready != 0U) &&
                (right_theta_ready != 0U) &&
                (fabsf(body_pitch_rad) <= recovery->ready_pitch_rad))
            {
                chassis.state_stable_s += dt_s;
            }
            else
            {
                chassis.state_stable_s = 0.0f;
            }

            if (chassis.state_stable_s >= recovery->stable_time_s)
            {
                Chassis_EnterState(CHASSIS_FALLING_TO_STAND);
            }
            else if (chassis.state_elapsed_s >= recovery->fallen_timeout_s)
            {
                Chassis_ZeroOutput();
                chassis.fault_flags = CHASSIS_FAULT_RECOVERY_TIMEOUT;
                Chassis_EnterState(CHASSIS_ZERO_FORCE);
                return;
            }
            else
            {
                if (Chassis_JointPositionControl(0U) == 0U)
                {
                    Chassis_EnterState(CHASSIS_ZERO_FORCE);
                }
                return;
            }
        }
    }

    if (chassis.state == CHASSIS_FALLING_TO_STAND)
    {
        chassis.state_elapsed_s += dt_s;
        chassis.target_leg_length_m[CHASSIS_LEFT] =
            recovery->bench_leg_length_m;
        chassis.target_leg_length_m[CHASSIS_RIGHT] =
            recovery->bench_leg_length_m;
        chassis.target_leg_phi0_rad[CHASSIS_LEFT] =
            Algorithm_AngleNearestEquivalentRad(
                recovery->bench_phi0_rad,
                chassis.leg[CHASSIS_LEFT].phi0_total_rad);
        chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
            Algorithm_AngleNearestEquivalentRad(
                recovery->bench_phi0_rad,
                chassis.leg[CHASSIS_RIGHT].phi0_total_rad);

        prepare_ready =
            ((fabsf(chassis.leg[CHASSIS_LEFT].length_m -
                    recovery->bench_leg_length_m) <=
              recovery->leg_length_tolerance_m) &&
             (fabsf(chassis.leg[CHASSIS_RIGHT].length_m -
                    recovery->bench_leg_length_m) <=
              recovery->leg_length_tolerance_m) &&
             (fabsf(Algorithm_AngleNormalizeRad(
                  recovery->bench_phi0_rad -
                  chassis.leg[CHASSIS_LEFT].phi0_total_rad)) <=
              recovery->leg_angle_tolerance_rad) &&
             (fabsf(Algorithm_AngleNormalizeRad(
                  recovery->bench_phi0_rad -
                  chassis.leg[CHASSIS_RIGHT].phi0_total_rad)) <=
              recovery->leg_angle_tolerance_rad) &&
             (fabsf(body_pitch_rad) <= recovery->ready_pitch_rad)) ? 1U : 0U;
        if (prepare_ready != 0U)
        {
            chassis.state_stable_s += dt_s;
        }
        else
        {
            chassis.state_stable_s = 0.0f;
        }

        if (chassis.state_stable_s >= recovery->stable_time_s)
        {
            Chassis_EnterState(CHASSIS_STANDING);
            return;
        }
        if (chassis.state_elapsed_s >= recovery->prepare_timeout_s)
        {
            Chassis_ZeroOutput();
            chassis.fault_flags = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            Chassis_EnterState(CHASSIS_ZERO_FORCE);
            return;
        }
        if (Chassis_JointPositionControl(0U) == 0U)
        {
            Chassis_EnterState(CHASSIS_ZERO_FORCE);
        }
    }
}

/**
 * @brief 设置固定板凳腿长/腿角，再复用主控制环计算轮LQR和关节位置环。
 */
void Chassis_BenchControlLoop(void)
{
    chassis.state_elapsed_s += chassis.control_dt_s;
    chassis.target_leg_length_m[CHASSIS_LEFT] =
        chassis_config.recovery.bench_leg_length_m;
    chassis.target_leg_length_m[CHASSIS_RIGHT] =
        chassis_config.recovery.bench_leg_length_m;
    chassis.target_leg_phi0_rad[CHASSIS_LEFT] =
        Algorithm_AngleNearestEquivalentRad(
            chassis_config.recovery.bench_phi0_rad,
            chassis.leg[CHASSIS_LEFT].phi0_total_rad);
    chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
        Algorithm_AngleNearestEquivalentRad(
            chassis_config.recovery.bench_phi0_rad,
            chassis.leg[CHASSIS_RIGHT].phi0_total_rad);
    Chassis_ControlLoop();
    if ((chassis.state == CHASSIS_BENCH) && (chassis.state_valid == 0U))
    {
        Chassis_EnterState(CHASSIS_ZERO_FORCE);
    }
}

/**
 * @brief 完成速度融合、十维状态、支撑力、LQR和VMC整条控制链。
 *
 * 函数开始先清最终命令；输出故障不阻断中间量计算，只在第7阶段
 * 阻止请求量进入joint_torque_nm和wheel_current。
 */
void Chassis_ControlLoop(void)
{
    chassis_vmc_torque_t left_torque = {0.0f, 0.0f};
    chassis_vmc_torque_t right_torque = {0.0f, 0.0f};
    float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT] = {0.0f};
    float dt_s;
    float body_pitch_rad;
    float body_pitch_rate_radps;
    float left_leg_angle_rad;
    float left_leg_angle_rate_radps;
    float right_leg_angle_rad;
    float right_leg_angle_rate_radps;
    float roll_rad;
    float roll_rate_radps;
    float left_length_pid = 0.0f;
    float right_length_pid = 0.0f;
    float roll_pid = 0.0f;
    float left_length_force_n;
    float right_length_force_n;
    float roll_force_n;
    uint8_t left_front_index;
    uint8_t left_back_index;
    uint8_t right_front_index;
    uint8_t right_back_index;
    uint8_t k_input_limited = 0U;
    uint8_t bench_mode;
    uint8_t joint_output_enabled;
    uint8_t wheel_output_enabled;
    uint8_t wheel_request_valid;
    uint8_t leg_states_valid;
    uint8_t left_torque_valid;
    uint8_t right_torque_valid;
    uint32_t output_faults;
    uint32_t active_faults;

    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.wheel_current_request,
           0,
           sizeof(chassis.wheel_current_request));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;
    chassis.k_fit_enabled = 0U;
    chassis.k_length_limited = 0U;
    bench_mode = (chassis.state == CHASSIS_BENCH) ? 1U : 0U;

    /* 1. 输出故障只封锁最终命令，中间控制量继续使用最新反馈计算。 */
    output_faults = Chassis_GetOutputFaults();
    leg_states_valid = Chassis_AreLegStatesValid();
    active_faults = output_faults;
    if (leg_states_valid == 0U)
    {
        active_faults |= CHASSIS_FAULT_KINEMATICS;
    }
    chassis.fault_flags = active_faults | CHASSIS_FAULT_CONTROL;

    /* 配置边界属于实机安全底线，运行期不再封装额外的状态查询函数。 */
    if ((chassis_config.imu.pitch_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.roll_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.yaw_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.forward_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link1_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link2_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link3_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link4_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link1_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link2_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link3_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link4_m <= 0.0f))
    {
        return;
    }

    left_front_index = chassis_config.leg[CHASSIS_LEFT]
                           .joint[CHASSIS_JOINT_FRONT]
                           .motor_index;
    left_back_index = chassis_config.leg[CHASSIS_LEFT]
                          .joint[CHASSIS_JOINT_BACK]
                          .motor_index;
    right_front_index = chassis_config.leg[CHASSIS_RIGHT]
                            .joint[CHASSIS_JOINT_FRONT]
                            .motor_index;
    right_back_index = chassis_config.leg[CHASSIS_RIGHT]
                           .joint[CHASSIS_JOINT_BACK]
                           .motor_index;
    if ((left_front_index >= APP_DM_COUNT) ||
        (left_back_index >= APP_DM_COUNT) ||
        (right_front_index >= APP_DM_COUNT) ||
        (right_back_index >= APP_DM_COUNT))
    {
        return;
    }

    if ((chassis_config.lqr.enabled != 0U) &&
        ((chassis_config.lqr.min_leg_length_m <= 0.0f) ||
         (chassis_config.lqr.max_leg_length_m <
          chassis_config.lqr.min_leg_length_m) ||
         ((chassis_config.lqr.length_source != CHASSIS_K_LENGTH_FIXED) &&
          (chassis_config.lqr.length_source != CHASSIS_K_LENGTH_MEASURED)) ||
         ((chassis_config.lqr.length_source == CHASSIS_K_LENGTH_FIXED) &&
          ((chassis_config.lqr.fixed_left_length_m <= 0.0f) ||
           (chassis_config.lqr.fixed_right_length_m <= 0.0f)))))
    {
        return;
    }

    dt_s = chassis.control_dt_s;
    if (bench_mode == 0U)
    {
        chassis.target_leg_length_m[CHASSIS_LEFT] =
            Chassis_MoveToward(chassis.target_leg_length_m[CHASSIS_LEFT],
                        chassis_config.leg[CHASSIS_LEFT].target_leg_length_m,
                        chassis_config.recovery.standing_length_rate_mps * dt_s);
        chassis.target_leg_length_m[CHASSIS_RIGHT] =
            Chassis_MoveToward(chassis.target_leg_length_m[CHASSIS_RIGHT],
                        chassis_config.leg[CHASSIS_RIGHT].target_leg_length_m,
                        chassis_config.recovery.standing_length_rate_mps * dt_s);
    }

    /* 轮速、IMU 和腿部状态组成速度融合的两个测量量。 */
    chassis.wheel_speed_radps[CHASSIS_LEFT] =
        (float)chassis.wheel_motor[CHASSIS_LEFT].speed_rpm *
        CHASSIS_RPM_TO_RADPS * chassis_config.wheel.left_speed_scale;
    chassis.wheel_speed_radps[CHASSIS_RIGHT] =
        (float)chassis.wheel_motor[CHASSIS_RIGHT].speed_rpm *
        CHASSIS_RPM_TO_RADPS * chassis_config.wheel.right_speed_scale;

    body_pitch_rad = chassis.imu.pitch_rad *
                     chassis_config.imu.pitch_angle_scale;
    body_pitch_rate_radps =
        chassis.imu.gyro_radps[chassis_config.imu.pitch_rate_axis] *
        chassis_config.imu.pitch_rate_scale;

    /*
     * theta = phi0 - pi/2 - pitch，与离线模型一致。多圈 phi0 只用于
     * 保持几何连续，送入线性 LQR 前选择目标平衡点附近的等价角。
     */
    left_leg_angle_rad = Algorithm_AngleNearestEquivalentRad(
        chassis.leg[CHASSIS_LEFT].phi0_total_rad -
            chassis_config.leg_vertical_offset_rad - body_pitch_rad,
        chassis.target_state[CHASSIS_STATE_THETA_L]);
    left_leg_angle_rate_radps =
        chassis.leg[CHASSIS_LEFT].phi0_speed_radps -
        body_pitch_rate_radps;
    right_leg_angle_rad = Algorithm_AngleNearestEquivalentRad(
        chassis.leg[CHASSIS_RIGHT].phi0_total_rad -
            chassis_config.leg_vertical_offset_rad - body_pitch_rad,
        chassis.target_state[CHASSIS_STATE_THETA_R]);
    right_leg_angle_rate_radps =
        chassis.leg[CHASSIS_RIGHT].phi0_speed_radps -
        body_pitch_rate_radps;

    chassis.forward_speed_raw_mps =
        chassis_config.wheel.radius_m *
            (chassis.wheel_speed_radps[CHASSIS_LEFT] +
             chassis.wheel_speed_radps[CHASSIS_RIGHT]) *
            0.5f +
        0.5f *
            (chassis.leg[CHASSIS_LEFT].length_m *
                 left_leg_angle_rate_radps * cosf(left_leg_angle_rad) +
             chassis.leg[CHASSIS_RIGHT].length_m *
                 right_leg_angle_rate_radps * cosf(right_leg_angle_rad)) +
        0.5f *
            (chassis.leg[CHASSIS_LEFT].length_speed_mps *
                 sinf(left_leg_angle_rad) +
             chassis.leg[CHASSIS_RIGHT].length_speed_mps *
                 sinf(right_leg_angle_rad));
    chassis.forward_accel_mps2 =
        chassis.imu.motion_accel_mps2[chassis_config.imu.forward_accel_axis] *
        chassis_config.imu.forward_accel_scale;

    if (chassis_config.speed_kalman.enabled != 0U)
    {
        chassis.speed_kalman.stateTransition[0] = 1.0f;
        chassis.speed_kalman.stateTransition[1] = dt_s;
        chassis.speed_kalman.stateTransition[2] = 0.0f;
        chassis.speed_kalman.stateTransition[3] = 1.0f;
        measurement[0] = chassis.forward_speed_raw_mps;
        measurement[1] = chassis.forward_accel_mps2;
        Algorithm_Kalman_Update(&chassis.speed_kalman, measurement);
        chassis.forward_speed_mps = chassis.speed_kalman.state[0];
        chassis.forward_accel_fused_mps2 = chassis.speed_kalman.state[1];
    }
    else
    {
        chassis.forward_speed_mps = chassis.forward_speed_raw_mps;
        chassis.forward_accel_fused_mps2 = chassis.forward_accel_mps2;
        chassis.speed_kalman.state[0] = chassis.forward_speed_mps;
        chassis.speed_kalman.state[1] = chassis.forward_accel_fused_mps2;
    }

    if ((chassis_config.speed_kalman.position_speed_limit_mps > 0.0f) &&
        (fabsf(chassis.forward_speed_mps) <=
         chassis_config.speed_kalman.position_speed_limit_mps))
    {
        chassis.forward_position_m += chassis.forward_speed_mps * dt_s;
    }
    else
    {
        chassis.forward_position_m = 0.0f;
    }

    /* 十维状态只在此处集中赋值，顺序不得与 MATLAB 模型分离。 */
    chassis.lqr_state[CHASSIS_STATE_S] = chassis.forward_position_m;
    chassis.lqr_state[CHASSIS_STATE_DOT_S] = chassis.forward_speed_mps;
    chassis.lqr_state[CHASSIS_STATE_FAI] =
        chassis.imu.yaw_total_rad * chassis_config.imu.yaw_angle_scale;
    chassis.lqr_state[CHASSIS_STATE_DOT_FAI] =
        chassis.imu.gyro_radps[chassis_config.imu.yaw_rate_axis] *
        chassis_config.imu.yaw_rate_scale;
    chassis.lqr_state[CHASSIS_STATE_THETA_L] = left_leg_angle_rad;
    chassis.lqr_state[CHASSIS_STATE_DOT_THETA_L] =
        left_leg_angle_rate_radps;
    chassis.lqr_state[CHASSIS_STATE_THETA_R] = right_leg_angle_rad;
    chassis.lqr_state[CHASSIS_STATE_DOT_THETA_R] =
        right_leg_angle_rate_radps;
    chassis.lqr_state[CHASSIS_STATE_THETA_B] = body_pitch_rad;
    chassis.lqr_state[CHASSIS_STATE_DOT_THETA_B] = body_pitch_rate_radps;

    /* 3. 小板凳由关节位置环保持腿姿态，不再叠加腿长和横滚支撑力。 */
    if (bench_mode != 0U)
    {
        chassis.support_force_n[CHASSIS_LEFT] = 0.0f;
        chassis.support_force_n[CHASSIS_RIGHT] = 0.0f;
    }
    else
    {
        roll_rad = chassis.imu.roll_rad * chassis_config.imu.roll_angle_scale;
        roll_rate_radps =
            chassis.imu.gyro_radps[chassis_config.imu.roll_rate_axis] *
            chassis_config.imu.roll_rate_scale;
        Algorithm_PID_UpdateByFeedbackRate(
            &chassis_config.leg_length_pid,
            &chassis.leg_length_pid[CHASSIS_LEFT],
            chassis.target_leg_length_m[CHASSIS_LEFT],
            chassis.leg[CHASSIS_LEFT].length_m,
            chassis.leg[CHASSIS_LEFT].length_speed_mps,
            dt_s,
            &left_length_pid);
        Algorithm_PID_UpdateByFeedbackRate(
            &chassis_config.leg_length_pid,
            &chassis.leg_length_pid[CHASSIS_RIGHT],
            chassis.target_leg_length_m[CHASSIS_RIGHT],
            chassis.leg[CHASSIS_RIGHT].length_m,
            chassis.leg[CHASSIS_RIGHT].length_speed_mps,
            dt_s,
            &right_length_pid);
        Algorithm_PID_UpdateByFeedbackRate(&chassis_config.roll_pid,
                                           &chassis.roll_pid,
                                           chassis_config.target_roll_rad,
                                           roll_rad,
                                           roll_rate_radps,
                                           dt_s,
                                           &roll_pid);

        left_length_force_n = -left_length_pid;
        right_length_force_n = -right_length_pid;
        roll_force_n = -roll_pid;
        chassis.support_force_n[CHASSIS_LEFT] =
            -roll_force_n + left_length_force_n +
            chassis_config.base_support_force_n +
            chassis_config.left_support_feedforward_n;
        chassis.support_force_n[CHASSIS_RIGHT] =
            roll_force_n + right_length_force_n +
            chassis_config.base_support_force_n -
            chassis_config.right_support_feedforward_n;
    }

    /* 4. 固定腿长调试和实时变腿长共用同一套双腿长 K 矩阵拟合。 */
    if (chassis_config.lqr.enabled == 0U)
    {
        memcpy(chassis.lqr_k,
               chassis_config.fixed_lqr_k,
               sizeof(chassis_config.fixed_lqr_k));
        chassis.k_input_length_m[CHASSIS_LEFT] = 0.0f;
        chassis.k_input_length_m[CHASSIS_RIGHT] = 0.0f;
        chassis.k_limited_length_m[CHASSIS_LEFT] = 0.0f;
        chassis.k_limited_length_m[CHASSIS_RIGHT] = 0.0f;
    }
    else
    {
        if (chassis_config.lqr.length_source == CHASSIS_K_LENGTH_MEASURED)
        {
            chassis.k_input_length_m[CHASSIS_LEFT] =
                chassis.leg[CHASSIS_LEFT].length_m;
            chassis.k_input_length_m[CHASSIS_RIGHT] =
                chassis.leg[CHASSIS_RIGHT].length_m;
        }
        else
        {
            chassis.k_input_length_m[CHASSIS_LEFT] =
                chassis_config.lqr.fixed_left_length_m;
            chassis.k_input_length_m[CHASSIS_RIGHT] =
                chassis_config.lqr.fixed_right_length_m;
        }

        Algorithm_LQR_FitLqrKPoly22(
            &chassis_config.lqr.coefficients[0][0][0],
            CHASSIS_OUTPUT_COUNT,
            CHASSIS_STATE_COUNT,
            chassis.k_input_length_m[CHASSIS_LEFT],
            chassis.k_input_length_m[CHASSIS_RIGHT],
            chassis_config.lqr.min_leg_length_m,
            chassis_config.lqr.max_leg_length_m,
            &chassis.lqr_k[0][0],
            &chassis.k_limited_length_m[CHASSIS_LEFT],
            &chassis.k_limited_length_m[CHASSIS_RIGHT],
            &k_input_limited);
        chassis.k_fit_enabled = 1U;
        chassis.k_length_limited = k_input_limited;
    }

    /* 5. 四行 K 直接对应左轮、右轮、左腿摆和右腿摆广义力矩。 */
    chassis.lqr_output[CHASSIS_OUTPUT_LEFT_WHEEL] =
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_S] *
            (chassis.target_state[CHASSIS_STATE_S] - chassis.lqr_state[CHASSIS_STATE_S]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_DOT_S] *
            (chassis.target_state[CHASSIS_STATE_DOT_S] - chassis.lqr_state[CHASSIS_STATE_DOT_S]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_FAI] *
            (chassis.target_state[CHASSIS_STATE_FAI] - chassis.lqr_state[CHASSIS_STATE_FAI]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_DOT_FAI] *
            (chassis.target_state[CHASSIS_STATE_DOT_FAI] - chassis.lqr_state[CHASSIS_STATE_DOT_FAI]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_THETA_L] *
            (chassis.target_state[CHASSIS_STATE_THETA_L] - chassis.lqr_state[CHASSIS_STATE_THETA_L]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_DOT_THETA_L] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_L] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_L]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_THETA_R] *
            (chassis.target_state[CHASSIS_STATE_THETA_R] - chassis.lqr_state[CHASSIS_STATE_THETA_R]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_DOT_THETA_R] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_R] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_R]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_THETA_B] *
            (chassis.target_state[CHASSIS_STATE_THETA_B] - chassis.lqr_state[CHASSIS_STATE_THETA_B]) +
        chassis.lqr_k[CHASSIS_OUTPUT_LEFT_WHEEL][CHASSIS_STATE_DOT_THETA_B] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_B] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_B]);

    chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_WHEEL] =
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_S] *
            (chassis.target_state[CHASSIS_STATE_S] - chassis.lqr_state[CHASSIS_STATE_S]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_DOT_S] *
            (chassis.target_state[CHASSIS_STATE_DOT_S] - chassis.lqr_state[CHASSIS_STATE_DOT_S]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_FAI] *
            (chassis.target_state[CHASSIS_STATE_FAI] - chassis.lqr_state[CHASSIS_STATE_FAI]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_DOT_FAI] *
            (chassis.target_state[CHASSIS_STATE_DOT_FAI] - chassis.lqr_state[CHASSIS_STATE_DOT_FAI]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_THETA_L] *
            (chassis.target_state[CHASSIS_STATE_THETA_L] - chassis.lqr_state[CHASSIS_STATE_THETA_L]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_DOT_THETA_L] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_L] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_L]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_THETA_R] *
            (chassis.target_state[CHASSIS_STATE_THETA_R] - chassis.lqr_state[CHASSIS_STATE_THETA_R]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_DOT_THETA_R] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_R] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_R]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_THETA_B] *
            (chassis.target_state[CHASSIS_STATE_THETA_B] - chassis.lqr_state[CHASSIS_STATE_THETA_B]) +
        chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_WHEEL][CHASSIS_STATE_DOT_THETA_B] *
            (chassis.target_state[CHASSIS_STATE_DOT_THETA_B] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_B]);

    if (bench_mode != 0U)
    {
        chassis.lqr_output[CHASSIS_OUTPUT_LEFT_LEG] = 0.0f;
        chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_LEG] = 0.0f;
    }
    else
    {
        chassis.lqr_output[CHASSIS_OUTPUT_LEFT_LEG] =
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_S] *
                (chassis.target_state[CHASSIS_STATE_S] - chassis.lqr_state[CHASSIS_STATE_S]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_DOT_S] *
                (chassis.target_state[CHASSIS_STATE_DOT_S] - chassis.lqr_state[CHASSIS_STATE_DOT_S]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_FAI] *
                (chassis.target_state[CHASSIS_STATE_FAI] - chassis.lqr_state[CHASSIS_STATE_FAI]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_DOT_FAI] *
                (chassis.target_state[CHASSIS_STATE_DOT_FAI] - chassis.lqr_state[CHASSIS_STATE_DOT_FAI]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_THETA_L] *
                (chassis.target_state[CHASSIS_STATE_THETA_L] - chassis.lqr_state[CHASSIS_STATE_THETA_L]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_DOT_THETA_L] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_L] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_L]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_THETA_R] *
                (chassis.target_state[CHASSIS_STATE_THETA_R] - chassis.lqr_state[CHASSIS_STATE_THETA_R]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_DOT_THETA_R] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_R] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_R]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_THETA_B] *
                (chassis.target_state[CHASSIS_STATE_THETA_B] - chassis.lqr_state[CHASSIS_STATE_THETA_B]) +
            chassis.lqr_k[CHASSIS_OUTPUT_LEFT_LEG][CHASSIS_STATE_DOT_THETA_B] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_B] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_B]);

        chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_LEG] =
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_S] *
                (chassis.target_state[CHASSIS_STATE_S] - chassis.lqr_state[CHASSIS_STATE_S]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_DOT_S] *
                (chassis.target_state[CHASSIS_STATE_DOT_S] - chassis.lqr_state[CHASSIS_STATE_DOT_S]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_FAI] *
                (chassis.target_state[CHASSIS_STATE_FAI] - chassis.lqr_state[CHASSIS_STATE_FAI]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_DOT_FAI] *
                (chassis.target_state[CHASSIS_STATE_DOT_FAI] - chassis.lqr_state[CHASSIS_STATE_DOT_FAI]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_THETA_L] *
                (chassis.target_state[CHASSIS_STATE_THETA_L] - chassis.lqr_state[CHASSIS_STATE_THETA_L]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_DOT_THETA_L] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_L] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_L]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_THETA_R] *
                (chassis.target_state[CHASSIS_STATE_THETA_R] - chassis.lqr_state[CHASSIS_STATE_THETA_R]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_DOT_THETA_R] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_R] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_R]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_THETA_B] *
                (chassis.target_state[CHASSIS_STATE_THETA_B] - chassis.lqr_state[CHASSIS_STATE_THETA_B]) +
            chassis.lqr_k[CHASSIS_OUTPUT_RIGHT_LEG][CHASSIS_STATE_DOT_THETA_B] *
                (chassis.target_state[CHASSIS_STATE_DOT_THETA_B] - chassis.lqr_state[CHASSIS_STATE_DOT_THETA_B]);
    }

    wheel_request_valid =
        ((chassis_config.wheel.torque_limit_nm > 0.0f) &&
         (chassis_config.wheel.torque_to_current != 0.0f) &&
         (chassis_config.wheel.current_limit > 0)) ? 1U : 0U;
    wheel_output_enabled =
        ((active_faults == CHASSIS_FAULT_NONE) &&
         (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
         (chassis_config.output.wheel_enabled != 0U) &&
         (wheel_request_valid != 0U)) ? 1U : 0U;
    if (wheel_request_valid != 0U)
    {
        chassis.wheel_current_request[CHASSIS_LEFT] =
            (int16_t)Chassis_LimitSymmetric(
                Chassis_LimitSymmetric(chassis.lqr_output[CHASSIS_OUTPUT_LEFT_WHEEL],
                                chassis_config.wheel.torque_limit_nm) *
                    chassis_config.wheel.torque_to_current,
                (float)chassis_config.wheel.current_limit);
        chassis.wheel_current_request[CHASSIS_RIGHT] =
            (int16_t)Chassis_LimitSymmetric(
                Chassis_LimitSymmetric(chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_WHEEL],
                                chassis_config.wheel.torque_limit_nm) *
                    chassis_config.wheel.torque_to_current,
                (float)chassis_config.wheel.current_limit);
    }
    if (bench_mode != 0U)
    {
        if (Chassis_JointPositionControl(wheel_output_enabled) == 0U)
        {
            Chassis_EnterState(CHASSIS_ZERO_FORCE);
        }
        else if (wheel_output_enabled != 0U)
        {
            memcpy(chassis.wheel_current,
                   chassis.wheel_current_request,
                   sizeof(chassis.wheel_current));
        }
        return;
    }

    /* 6. 左右腿支撑力和摆力矩经 VMC 映射为四个 DM 关节力矩。 */
    left_torque_valid =
        VMC_CalcTorque(&chassis_config.leg[CHASSIS_LEFT],
                       &chassis.leg[CHASSIS_LEFT],
                       chassis.support_force_n[CHASSIS_LEFT],
                       chassis.lqr_output[CHASSIS_OUTPUT_LEFT_LEG],
                       &left_torque);
    right_torque_valid =
        VMC_CalcTorque(&chassis_config.leg[CHASSIS_RIGHT],
                       &chassis.leg[CHASSIS_RIGHT],
                       chassis.support_force_n[CHASSIS_RIGHT],
                       chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_LEG],
                       &right_torque);
    if ((left_torque_valid == 0U) || (right_torque_valid == 0U))
    {
        active_faults |= CHASSIS_FAULT_KINEMATICS;
        wheel_output_enabled = 0U;
    }

    chassis.joint_torque_request_nm[left_front_index] =
        left_torque.front_nm;
    chassis.joint_torque_request_nm[left_back_index] =
        left_torque.back_nm;
    chassis.joint_torque_request_nm[right_front_index] =
        right_torque.front_nm;
    chassis.joint_torque_request_nm[right_back_index] =
        right_torque.back_nm;

    /* 7. 物理力矩经方向和限幅后写入任务层实际发送的关节命令。 */
    joint_output_enabled =
        ((active_faults == CHASSIS_FAULT_NONE) &&
         (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
         (chassis_config.output.joint_enabled != 0U) &&
         (chassis_config.output.joint_torque_limit_nm > 0.0f)) ? 1U : 0U;
    if (joint_output_enabled != 0U)
    {
        chassis.joint_torque_nm[left_front_index] =
            Chassis_LimitSymmetric(left_torque.front_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[left_back_index] =
            Chassis_LimitSymmetric(left_torque.back_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[right_front_index] =
            Chassis_LimitSymmetric(right_torque.front_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[right_back_index] =
            Chassis_LimitSymmetric(right_torque.back_nm,
                            chassis_config.output.joint_torque_limit_nm);
    }
    if (wheel_output_enabled != 0U)
    {
        memcpy(chassis.wheel_current,
               chassis.wheel_current_request,
               sizeof(chassis.wheel_current));
    }

    chassis.fault_flags = active_faults;
    chassis.safe_output =
        ((joint_output_enabled == 0U) && (wheel_output_enabled == 0U)) ? 1U : 0U;
    chassis.state_valid = 1U;
}
