#include "chassis_control.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TEST_TOLERANCE 2.0e-5f

static void set_online_feedback(void)
{
    uint32_t index;

    chassis.enabled = 1U;
    chassis.control_dt_s = chassis_config.default_dt_s;
    chassis.imu.initialized = 1U;
    chassis.imu.attitude_ready = 1U;
    chassis.imu.error_code = 0U;
    chassis.can_tx_error_count = 0U;
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        chassis.dm_motor[index].online = 1U;
        chassis.dm_motor[index].speed_radps = 0.0f;
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        chassis.wheel_motor[index].online = 1U;
        chassis.wheel_motor[index].speed_rpm = 0;
    }
}

static void set_leg_pose(chassis_leg_side_t side,
                         float length_m,
                         float phi0_rad)
{
    const chassis_leg_config_t *config = &chassis_config.leg[side];
    chassis_vmc_state_t current_leg = {
        .phi1_rad = CHASSIS_PI,
        .phi4_rad = 0.0f,
    };
    chassis_vmc_joint_target_t target;
    uint8_t front_index = config->joint[CHASSIS_JOINT_FRONT].motor_index;
    uint8_t back_index = config->joint[CHASSIS_JOINT_BACK].motor_index;

    assert(vmc_calc_joint_target(config,
                                 &current_leg,
                                 length_m,
                                 phi0_rad,
                                 &target) == 1U);
    chassis.dm_motor[front_index].position_rad =
        (target.phi1_rad -
         config->joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config->joint[CHASSIS_JOINT_FRONT].angle_scale;
    chassis.dm_motor[back_index].position_rad =
        (target.phi4_rad -
         config->joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config->joint[CHASSIS_JOINT_BACK].angle_scale;
}

static void set_symmetric_leg_pose(float length_m, float phi0_rad)
{
    set_leg_pose(CHASSIS_LEFT, length_m, phi0_rad);
    set_leg_pose(CHASSIS_RIGHT, length_m, phi0_rad);
    chassis_control_update_leg_state();
    assert(chassis.leg_state_valid == 1U);
}

static void assert_zero_final_output(void)
{
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        assert(chassis.joint_torque_nm[index] == 0.0f);
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        assert(chassis.wheel_current[index] == 0);
    }
    assert(chassis.safe_output == 1U);
}

static void test_bench_control(void)
{
    float maximum_request_nm = 0.0f;
    uint32_t index;

    chassis_control_init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_BENCH;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_BENCH);

    chassis_bench_control_loop();
    assert(chassis.state == CHASSIS_BENCH);
    assert(chassis.state_valid == 1U);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] - 0.15f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_RIGHT] - 0.15f) <
           TEST_TOLERANCE);
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(chassis.joint_torque_request_nm[index]);

        assert(request_nm <=
               chassis_config.recovery.joint_torque_limit_nm +
                   TEST_TOLERANCE);
        if (request_nm > maximum_request_nm)
        {
            maximum_request_nm = request_nm;
        }
    }
    assert(maximum_request_nm > 0.0f);
    assert_zero_final_output();
}

static void test_recovery_handoff(void)
{
    uint32_t iteration;

    chassis_control_init();
    set_online_feedback();
    chassis.imu.pitch_rad = 0.0f;
    chassis.imu.yaw_rad = 0.70f;
    set_symmetric_leg_pose(chassis_config.recovery.bench_leg_length_m,
                           chassis_config.recovery.bench_phi0_rad);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_FALLEN);

    for (iteration = 0U; iteration < 150U; iteration++)
    {
        chassis.joint_torque_nm[0] = 0.5f;
        chassis_control_update_leg_state();
        chassis_control_update_state();
        if ((chassis.state == CHASSIS_FALLEN) ||
            (chassis.state == CHASSIS_FALLING_TO_STAND))
        {
            chassis_recovery_control_loop();
        }
        if (chassis.state == CHASSIS_STANDING)
        {
            break;
        }
    }

    assert(chassis.state == CHASSIS_STANDING);
    assert_zero_final_output();
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] -
                 chassis_config.recovery.bench_leg_length_m) <
           TEST_TOLERANCE);
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_STANDING);

    chassis_control_loop();
    assert(chassis.state_valid == 1U);
    assert(chassis.target_leg_length_m[CHASSIS_LEFT] >
           chassis_config.recovery.bench_leg_length_m);
    assert_zero_final_output();
}

static void test_fallen_timeout(void)
{
    float left_phi0_rad;

    chassis_control_init();
    set_online_feedback();
    chassis.imu.pitch_rad = 1.20f;
    set_symmetric_leg_pose(0.30f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_FALLEN);

    left_phi0_rad = chassis.leg[CHASSIS_LEFT].phi0_rad;
    chassis_recovery_control_loop();
    assert(chassis.state == CHASSIS_FALLEN);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] -
                 chassis_config.recovery.extended_leg_length_m) <
           TEST_TOLERANCE);
    assert(chassis.target_leg_phi0_rad[CHASSIS_LEFT] < left_phi0_rad);
    assert_zero_final_output();

    chassis.state_elapsed_s =
        chassis_config.recovery.fallen_timeout_s -
        chassis.control_dt_s * 0.5f;
    chassis_recovery_control_loop();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assert_zero_final_output();

    chassis_control_update_state();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
}

static void test_prepare_timeout(void)
{
    chassis_control_init();
    set_online_feedback();
    chassis.imu.pitch_rad = 0.0f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_FALLEN);

    chassis_recovery_control_loop();
    assert(chassis.state == CHASSIS_FALLING_TO_STAND);
    chassis.state_elapsed_s =
        chassis_config.recovery.prepare_timeout_s -
        chassis.control_dt_s * 0.5f;
    chassis_recovery_control_loop();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assert_zero_final_output();
}

static void test_invalid_leg_feedback(void)
{
    uint8_t left_front_index;
    uint8_t left_back_index;
    uint8_t right_front_index;
    uint8_t right_back_index;

    chassis_control_init();
    set_online_feedback();
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

    /* phi1 == phi4 且髋轴距离为零时，B/D 两点重合，闭链状态无解。 */
    chassis.dm_motor[left_front_index].position_rad = CHASSIS_PI;
    chassis.dm_motor[left_back_index].position_rad = 0.0f;
    chassis.dm_motor[right_front_index].position_rad = CHASSIS_PI;
    chassis.dm_motor[right_back_index].position_rad = 0.0f;
    chassis_control_update_leg_state();
    assert(chassis.leg_state_valid == 0U);

    chassis.mode = CHASSIS_MODE_BENCH;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_BENCH);
    chassis_bench_control_loop();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_KINEMATICS);
    assert_zero_final_output();
}

static void test_standing_fault_latches_zero_force(void)
{
    uint8_t left_front_index;

    chassis_control_init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_FOLLOW;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_STANDING);

    left_front_index = chassis_config.leg[CHASSIS_LEFT]
                           .joint[CHASSIS_JOINT_FRONT]
                           .motor_index;
    chassis.dm_motor[left_front_index].online = 0U;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert((chassis.fault_flags & CHASSIS_FAULT_DM_MOTOR) != 0U);
    assert_zero_final_output();

    chassis.dm_motor[left_front_index].online = 1U;
    chassis_control_update_state();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert((chassis.fault_flags & CHASSIS_FAULT_DM_MOTOR) != 0U);
}

int main(void)
{
    test_bench_control();
    test_recovery_handoff();
    test_fallen_timeout();
    test_prepare_timeout();
    test_invalid_leg_feedback();
    test_standing_fault_latches_zero_force();
    return 0;
}
