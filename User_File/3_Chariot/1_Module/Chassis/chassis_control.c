#include "chassis_control.h"

#include "LQR.h"
#include "PID.h"

#include <math.h>
#include <string.h>

#define CHASSIS_RPM_TO_RADPS 0.10471975512f
#define CHASSIS_CONTROL_EPSILON 1.0e-6f

chassis_t chassis;

static float limit_symmetric(float value, float limit)
{
    float positive_limit = fabsf(limit);

    if (positive_limit <= 0.0f)
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

static float move_toward(float value, float target, float maximum_step)
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

static float angle_error_rad(float target_rad, float feedback_rad)
{
    float error_rad = target_rad - feedback_rad;

    while (error_rad > CHASSIS_PI)
    {
        error_rad -= 2.0f * CHASSIS_PI;
    }
    while (error_rad < -CHASSIS_PI)
    {
        error_rad += 2.0f * CHASSIS_PI;
    }
    return error_rad;
}

static uint8_t chassis_get_joint_indices(
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

static uint32_t chassis_get_feedback_faults(void)
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

static void chassis_reset_joint_control(void)
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

static void chassis_enter_state(chassis_control_state_t state)
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
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;
    chassis_reset_joint_control();

    if (state == CHASSIS_STANDING)
    {
        Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_LEFT]);
        Algorithm_PID_Init(&chassis.leg_length_pid[CHASSIS_RIGHT]);
        Algorithm_PID_Init(&chassis.roll_pid);
        chassis_control_reset();
        memcpy(chassis.target_state,
               chassis_config.target_state,
               sizeof(chassis.target_state));
        chassis.target_state[CHASSIS_STATE_S] = 0.0f;
        chassis.target_state[CHASSIS_STATE_FAI] =
            chassis.imu.yaw_rad * chassis_config.imu.yaw_angle_scale;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            chassis.target_leg_length_m[side] =
                (chassis.leg_state_valid != 0U) ?
                    chassis.leg[side].length_m :
                    chassis_config.leg[side].target_leg_length_m;
        }
    }
}

void chassis_control_reset(void)
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

void chassis_zero_output(void)
{
    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    memset(chassis.target_joint_speed_radps,
           0,
           sizeof(chassis.target_joint_speed_radps));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;
    chassis_control_reset();
}

void chassis_control_init(void)
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
    chassis_reset_joint_control();
    for (index = 0U; index < CHASSIS_LEG_COUNT; index++)
    {
        chassis.target_leg_length_m[index] =
            chassis_config.leg[index].target_leg_length_m;
        chassis.target_leg_phi0_rad[index] =
            chassis_config.recovery.bench_phi0_rad;
    }
    chassis_control_reset();
}

void chassis_control_update_leg_state(void)
{
    uint8_t indices[CHASSIS_LEG_COUNT][CHASSIS_JOINT_COUNT];
    uint32_t side;

    chassis.leg_state_valid = 0U;
    memset(chassis.leg, 0, sizeof(chassis.leg));
    if (chassis_get_joint_indices(indices) == 0U)
    {
        return;
    }

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        vmc_calc_state(&chassis_config.leg[side],
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_FRONT]]
                           .position_rad,
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_BACK]]
                           .position_rad,
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_FRONT]]
                           .speed_radps,
                       chassis.dm_motor[indices[side][CHASSIS_JOINT_BACK]]
                           .speed_radps,
                       &chassis.leg[side]);
        if ((chassis.leg[side].length_m <=
             chassis_config.leg[side].geometry.min_leg_length_m) ||
            (!isfinite(chassis.leg[side].length_m)) ||
            (!isfinite(chassis.leg[side].phi0_rad)))
        {
            return;
        }
    }
    chassis.leg_state_valid = 1U;
}

void chassis_control_update_state(void)
{
    float body_pitch_rad;
    uint8_t posture_ready;
    uint32_t feedback_faults;

    if (chassis.enabled == 0U)
    {
        chassis_enter_state(CHASSIS_ZERO_FORCE);
        chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
        chassis.fault_flags = CHASSIS_FAULT_DISABLED;
        return;
    }

    if (chassis.mode == CHASSIS_MODE_ZERO_FORCE)
    {
        chassis_enter_state(CHASSIS_ZERO_FORCE);
        chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
        chassis.fault_flags = CHASSIS_FAULT_NONE;
        return;
    }

    body_pitch_rad = chassis.imu.pitch_rad *
                     chassis_config.imu.pitch_angle_scale;
    feedback_faults = chassis_get_feedback_faults();
    posture_ready =
        ((feedback_faults == CHASSIS_FAULT_NONE) &&
         (chassis.leg_state_valid != 0U) &&
         (fabsf(body_pitch_rad) <=
          chassis_config.recovery.direct_prepare_pitch_rad) &&
         (chassis.leg[CHASSIS_LEFT].phi0_rad >=
          chassis_config.recovery.direct_phi0_min_rad) &&
         (chassis.leg[CHASSIS_LEFT].phi0_rad <=
          chassis_config.recovery.direct_phi0_max_rad) &&
         (chassis.leg[CHASSIS_RIGHT].phi0_rad >=
          chassis_config.recovery.direct_phi0_min_rad) &&
         (chassis.leg[CHASSIS_RIGHT].phi0_rad <=
          chassis_config.recovery.direct_phi0_max_rad)) ? 1U : 0U;

    if (chassis.mode != chassis.last_mode)
    {
        chassis.fault_flags = CHASSIS_FAULT_NONE;
        switch (chassis.mode)
        {
        case CHASSIS_MODE_FOLLOW:
        case CHASSIS_MODE_TOP:
            if (chassis.state != CHASSIS_STANDING)
            {
                if (posture_ready != 0U)
                {
                    chassis_enter_state(CHASSIS_STANDING);
                }
                else
                {
                    if (feedback_faults != CHASSIS_FAULT_NONE)
                    {
                        chassis.fault_flags = feedback_faults;
                    }
                    else if (chassis.leg_state_valid == 0U)
                    {
                        chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
                    }
                    else
                    {
                        chassis.fault_flags = CHASSIS_FAULT_CONTROL;
                    }
                    chassis_enter_state(CHASSIS_ZERO_FORCE);
                }
            }
            break;

        case CHASSIS_MODE_SELF_SAVE:
            chassis_enter_state(CHASSIS_FALLEN);
            break;

        case CHASSIS_MODE_BENCH:
            chassis_enter_state(CHASSIS_BENCH);
            break;

        case CHASSIS_MODE_ZERO_FORCE:
        default:
            chassis_enter_state(CHASSIS_ZERO_FORCE);
            break;
        }
        chassis.last_mode = chassis.mode;
    }

    if ((chassis.state == CHASSIS_STANDING) &&
        (feedback_faults != CHASSIS_FAULT_NONE))
    {
        chassis.fault_flags = feedback_faults;
        chassis_enter_state(CHASSIS_ZERO_FORCE);
        return;
    }

    if ((chassis.state == CHASSIS_STANDING) &&
        (chassis.leg_state_valid == 0U))
    {
        chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
        chassis_enter_state(CHASSIS_ZERO_FORCE);
        return;
    }

    if ((chassis.state == CHASSIS_STANDING) &&
        ((fabsf(body_pitch_rad) >
          chassis_config.recovery.standing_pitch_limit_rad) ||
         (chassis.leg[CHASSIS_LEFT].phi0_rad <
          chassis_config.recovery.standing_phi0_min_rad) ||
         (chassis.leg[CHASSIS_LEFT].phi0_rad >
          chassis_config.recovery.standing_phi0_max_rad) ||
         (chassis.leg[CHASSIS_RIGHT].phi0_rad <
          chassis_config.recovery.standing_phi0_min_rad) ||
         (chassis.leg[CHASSIS_RIGHT].phi0_rad >
          chassis_config.recovery.standing_phi0_max_rad)))
    {
        chassis.fault_flags = CHASSIS_FAULT_CONTROL;
        chassis_enter_state(CHASSIS_ZERO_FORCE);
    }
}

static uint8_t chassis_joint_position_control(void)
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
    uint32_t fault_flags;
    uint32_t side;
    uint32_t joint;

    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    memset(chassis.target_joint_speed_radps,
           0,
           sizeof(chassis.target_joint_speed_radps));
    chassis.safe_output = 1U;
    chassis.state_valid = 0U;

    fault_flags = chassis_get_feedback_faults();
    if (fault_flags != CHASSIS_FAULT_NONE)
    {
        chassis.fault_flags = fault_flags;
        return 0U;
    }
    if ((chassis.leg_state_valid == 0U) ||
        (chassis_get_joint_indices(indices) == 0U))
    {
        chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
        return 0U;
    }
    if ((dt_s < chassis_config.min_dt_s) ||
        (dt_s > chassis_config.max_dt_s))
    {
        dt_s = chassis_config.default_dt_s;
    }

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (vmc_calc_joint_target(&chassis_config.leg[side],
                                  &chassis.leg[side],
                                  chassis.target_leg_length_m[side],
                                  chassis.target_leg_phi0_rad[side],
                                  &joint_target[side]) == 0U)
        {
            chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
            return 0U;
        }
        chassis.target_joint_angle_rad
            [indices[side][CHASSIS_JOINT_FRONT]] =
                joint_target[side].phi1_rad;
        chassis.target_joint_angle_rad
            [indices[side][CHASSIS_JOINT_BACK]] =
                joint_target[side].phi4_rad;
    }

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
                limit_symmetric(
                    geometric_torque_nm *
                        chassis_config.leg[side].joint[joint].torque_scale,
                    chassis_config.recovery.joint_torque_limit_nm);
        }
    }

    joint_output_enabled =
        ((APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
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
                limit_symmetric(chassis.joint_torque_request_nm[side],
                                output_limit_nm);
        }
    }

    chassis.fault_flags = CHASSIS_FAULT_NONE;
    chassis.safe_output = (joint_output_enabled == 0U) ? 1U : 0U;
    chassis.state_valid = 1U;
    return 1U;
}

void chassis_recovery_control_loop(void)
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
    float dt_s = chassis.control_dt_s;

    if ((dt_s < chassis_config.min_dt_s) ||
        (dt_s > chassis_config.max_dt_s))
    {
        dt_s = chassis_config.default_dt_s;
    }
    if ((chassis.state != CHASSIS_FALLEN) &&
        (chassis.state != CHASSIS_FALLING_TO_STAND))
    {
        chassis_zero_output();
        return;
    }
    if (chassis.leg_state_valid == 0U)
    {
        chassis_zero_output();
        chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
        chassis_enter_state(CHASSIS_ZERO_FORCE);
        return;
    }

    body_pitch_rad = chassis.imu.pitch_rad *
                     chassis_config.imu.pitch_angle_scale;
    left_theta_rad = chassis.leg[CHASSIS_LEFT].phi0_rad -
                     chassis_config.leg_vertical_offset_rad -
                     body_pitch_rad;
    right_theta_rad = chassis.leg[CHASSIS_RIGHT].phi0_rad -
                      chassis_config.leg_vertical_offset_rad -
                      body_pitch_rad;

    if (chassis.state == CHASSIS_FALLEN)
    {
        chassis.state_elapsed_s += dt_s;
        direct_prepare =
            ((fabsf(body_pitch_rad) <= recovery->direct_prepare_pitch_rad) &&
             (chassis.leg[CHASSIS_LEFT].phi0_rad >=
              recovery->direct_phi0_min_rad) &&
             (chassis.leg[CHASSIS_LEFT].phi0_rad <=
              recovery->direct_phi0_max_rad) &&
             (chassis.leg[CHASSIS_RIGHT].phi0_rad >=
              recovery->direct_phi0_min_rad) &&
             (chassis.leg[CHASSIS_RIGHT].phi0_rad <=
              recovery->direct_phi0_max_rad)) ? 1U : 0U;
        if (direct_prepare != 0U)
        {
            chassis_enter_state(CHASSIS_FALLING_TO_STAND);
        }
        else
        {
            left_theta_ready =
                ((left_theta_rad >= recovery->ready_theta_min_rad) &&
                 (left_theta_rad <= recovery->ready_theta_max_rad)) ? 1U : 0U;
            right_theta_ready =
                ((right_theta_rad >= recovery->ready_theta_min_rad) &&
                 (right_theta_rad <= recovery->ready_theta_max_rad)) ? 1U : 0U;
            rotate_direction = (body_pitch_rad < 0.0f) ? 1.0f : -1.0f;
            left_rotate_offset_rad = recovery->rotate_offset_rad;
            right_rotate_offset_rad = recovery->rotate_offset_rad;
            ready_theta_center_rad =
                (recovery->ready_theta_min_rad +
                 recovery->ready_theta_max_rad) * 0.5f;

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
            chassis.target_leg_phi0_rad[CHASSIS_LEFT] =
                ((left_theta_ready != 0U) &&
                 (fabsf(body_pitch_rad) <=
                  recovery->direct_prepare_pitch_rad)) ?
                    chassis.leg[CHASSIS_LEFT].phi0_rad :
                    chassis.leg[CHASSIS_LEFT].phi0_rad +
                        rotate_direction * left_rotate_offset_rad;
            chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
                ((right_theta_ready != 0U) &&
                 (fabsf(body_pitch_rad) <=
                  recovery->direct_prepare_pitch_rad)) ?
                    chassis.leg[CHASSIS_RIGHT].phi0_rad :
                    chassis.leg[CHASSIS_RIGHT].phi0_rad +
                        rotate_direction * right_rotate_offset_rad;

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
                chassis_enter_state(CHASSIS_FALLING_TO_STAND);
            }
            else if (chassis.state_elapsed_s >= recovery->fallen_timeout_s)
            {
                chassis_zero_output();
                chassis.fault_flags = CHASSIS_FAULT_RECOVERY_TIMEOUT;
                chassis_enter_state(CHASSIS_ZERO_FORCE);
                return;
            }
            else
            {
                if (chassis_joint_position_control() == 0U)
                {
                    chassis_enter_state(CHASSIS_ZERO_FORCE);
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
            recovery->bench_phi0_rad;
        chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
            recovery->bench_phi0_rad;

        prepare_ready =
            ((fabsf(chassis.leg[CHASSIS_LEFT].length_m -
                    recovery->bench_leg_length_m) <=
              recovery->leg_length_tolerance_m) &&
             (fabsf(chassis.leg[CHASSIS_RIGHT].length_m -
                    recovery->bench_leg_length_m) <=
              recovery->leg_length_tolerance_m) &&
             (fabsf(angle_error_rad(recovery->bench_phi0_rad,
                                    chassis.leg[CHASSIS_LEFT].phi0_rad)) <=
              recovery->leg_angle_tolerance_rad) &&
             (fabsf(angle_error_rad(recovery->bench_phi0_rad,
                                    chassis.leg[CHASSIS_RIGHT].phi0_rad)) <=
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
            chassis_enter_state(CHASSIS_STANDING);
            return;
        }
        if (chassis.state_elapsed_s >= recovery->prepare_timeout_s)
        {
            chassis_zero_output();
            chassis.fault_flags = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            chassis_enter_state(CHASSIS_ZERO_FORCE);
            return;
        }
        if (chassis_joint_position_control() == 0U)
        {
            chassis_enter_state(CHASSIS_ZERO_FORCE);
        }
    }
}

void chassis_bench_control_loop(void)
{
    chassis.state_elapsed_s += chassis.control_dt_s;
    chassis.target_leg_length_m[CHASSIS_LEFT] =
        chassis_config.recovery.bench_leg_length_m;
    chassis.target_leg_length_m[CHASSIS_RIGHT] =
        chassis_config.recovery.bench_leg_length_m;
    chassis.target_leg_phi0_rad[CHASSIS_LEFT] =
        chassis_config.recovery.bench_phi0_rad;
    chassis.target_leg_phi0_rad[CHASSIS_RIGHT] =
        chassis_config.recovery.bench_phi0_rad;
    if (chassis_joint_position_control() == 0U)
    {
        chassis_enter_state(CHASSIS_ZERO_FORCE);
    }
}

void chassis_control_loop(void)
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
    uint8_t joint_output_enabled;
    uint8_t wheel_output_enabled;
    uint32_t fault_flags = CHASSIS_FAULT_NONE;

    memset(chassis.joint_torque_nm, 0, sizeof(chassis.joint_torque_nm));
    memset(chassis.wheel_current, 0, sizeof(chassis.wheel_current));
    memset(chassis.joint_torque_request_nm,
           0,
           sizeof(chassis.joint_torque_request_nm));
    chassis.safe_output = 1U;
    chassis.fault_flags = CHASSIS_FAULT_CONTROL;
    chassis.state_valid = 0U;
    chassis.k_fit_enabled = 0U;
    chassis.k_length_limited = 0U;

    /* 1. 输入来自任务层反馈；设备异常时保持本轮零命令并清空运动融合。 */
    fault_flags = chassis_get_feedback_faults();
    if (fault_flags != CHASSIS_FAULT_NONE)
    {
        chassis.fault_flags = fault_flags;
        chassis_control_reset();
        return;
    }

    /* 配置边界属于实机安全底线，运行期不再封装额外的状态查询函数。 */
    if ((chassis_config.imu.pitch_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.roll_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.yaw_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.imu.forward_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link1_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link2_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link3_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.link4_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_LEFT].geometry.min_leg_length_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link1_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link2_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link3_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.link4_m <= 0.0f) ||
        (chassis_config.leg[CHASSIS_RIGHT].geometry.min_leg_length_m <= 0.0f))
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

    /* 2. 任务反馈阶段已统一更新五连杆状态，所有控制模式共用同一份结果。 */
    if (chassis.leg_state_valid == 0U)
    {
        chassis.fault_flags = CHASSIS_FAULT_KINEMATICS;
        return;
    }

    dt_s = chassis.control_dt_s;
    chassis.target_leg_length_m[CHASSIS_LEFT] =
        move_toward(chassis.target_leg_length_m[CHASSIS_LEFT],
                    chassis_config.leg[CHASSIS_LEFT].target_leg_length_m,
                    chassis_config.recovery.standing_length_rate_mps * dt_s);
    chassis.target_leg_length_m[CHASSIS_RIGHT] =
        move_toward(chassis.target_leg_length_m[CHASSIS_RIGHT],
                    chassis_config.leg[CHASSIS_RIGHT].target_leg_length_m,
                    chassis_config.recovery.standing_length_rate_mps * dt_s);

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

    /* theta = phi0 - pi/2 - pitch，与当前十维模型离线求 K 定义一致。 */
    left_leg_angle_rad = chassis.leg[CHASSIS_LEFT].phi0_rad -
                         chassis_config.leg_vertical_offset_rad -
                         body_pitch_rad;
    left_leg_angle_rate_radps =
        chassis.leg[CHASSIS_LEFT].phi0_speed_radps -
        body_pitch_rate_radps;
    right_leg_angle_rad = chassis.leg[CHASSIS_RIGHT].phi0_rad -
                          chassis_config.leg_vertical_offset_rad -
                          body_pitch_rad;
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
        chassis.imu.yaw_rad * chassis_config.imu.yaw_angle_scale;
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

    /* 3. 腿长 PID、支撑力前馈和横滚补偿输出左右腿虚拟支撑力。 */
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

    /* 6. 左右腿支撑力和摆力矩经 VMC 映射为四个 DM 关节力矩。 */
    if ((fabsf(chassis.leg[CHASSIS_LEFT].length_m) <= CHASSIS_CONTROL_EPSILON) ||
        (fabsf(chassis.leg[CHASSIS_RIGHT].length_m) <= CHASSIS_CONTROL_EPSILON) ||
        (fabsf(sinf(chassis.leg[CHASSIS_LEFT].phi2_rad -
                    chassis.leg[CHASSIS_LEFT].phi3_rad)) <= CHASSIS_CONTROL_EPSILON) ||
        (fabsf(sinf(chassis.leg[CHASSIS_RIGHT].phi2_rad -
                    chassis.leg[CHASSIS_RIGHT].phi3_rad)) <= CHASSIS_CONTROL_EPSILON))
    {
        return;
    }

    vmc_calc_torque(&chassis_config.leg[CHASSIS_LEFT],
                    &chassis.leg[CHASSIS_LEFT],
                    chassis.support_force_n[CHASSIS_LEFT],
                    chassis.lqr_output[CHASSIS_OUTPUT_LEFT_LEG],
                    &left_torque);
    vmc_calc_torque(&chassis_config.leg[CHASSIS_RIGHT],
                    &chassis.leg[CHASSIS_RIGHT],
                    chassis.support_force_n[CHASSIS_RIGHT],
                    chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_LEG],
                    &right_torque);

    chassis.joint_torque_request_nm[left_front_index] =
        left_torque.front_nm;
    chassis.joint_torque_request_nm[left_back_index] =
        left_torque.back_nm;
    chassis.joint_torque_request_nm[right_front_index] =
        right_torque.front_nm;
    chassis.joint_torque_request_nm[right_back_index] =
        right_torque.back_nm;

    /* 7. 物理力矩经方向、限幅和换算后写入任务层实际发送的命令数组。 */
    joint_output_enabled =
        ((APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
         (chassis_config.output.joint_enabled != 0U) &&
         (chassis_config.output.joint_torque_limit_nm > 0.0f)) ? 1U : 0U;
    wheel_output_enabled =
        ((APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
         (chassis_config.output.wheel_enabled != 0U) &&
         (chassis_config.wheel.torque_limit_nm > 0.0f) &&
         (chassis_config.wheel.torque_to_current != 0.0f) &&
         (chassis_config.wheel.current_limit > 0)) ? 1U : 0U;

    if (joint_output_enabled != 0U)
    {
        chassis.joint_torque_nm[left_front_index] =
            limit_symmetric(left_torque.front_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[left_back_index] =
            limit_symmetric(left_torque.back_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[right_front_index] =
            limit_symmetric(right_torque.front_nm,
                            chassis_config.output.joint_torque_limit_nm);
        chassis.joint_torque_nm[right_back_index] =
            limit_symmetric(right_torque.back_nm,
                            chassis_config.output.joint_torque_limit_nm);
    }

    if (wheel_output_enabled != 0U)
    {
        chassis.wheel_current[CHASSIS_LEFT] = (int16_t)limit_symmetric(
            limit_symmetric(chassis.lqr_output[CHASSIS_OUTPUT_LEFT_WHEEL],
                            chassis_config.wheel.torque_limit_nm) *
                chassis_config.wheel.torque_to_current,
            (float)chassis_config.wheel.current_limit);
        chassis.wheel_current[CHASSIS_RIGHT] = (int16_t)limit_symmetric(
            limit_symmetric(chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_WHEEL],
                            chassis_config.wheel.torque_limit_nm) *
                chassis_config.wheel.torque_to_current,
            (float)chassis_config.wheel.current_limit);
    }

    chassis.fault_flags = CHASSIS_FAULT_NONE;
    chassis.safe_output =
        ((joint_output_enabled == 0U) && (wheel_output_enabled == 0U)) ? 1U : 0U;
    chassis.state_valid = 1U;
}
