#include "chassis_control.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TEST_TOLERANCE 2.0e-5f

static void setOnlineFeedback(void)
{
    uint32_t index;

    chassis.enabled = 1U;
    chassis.control_dt_s = chassis_config.default_dt_s;
    chassis.imu.initialized = 1U;
    chassis.imu.attitude_ready = 1U;
    chassis.imu.error_code = 0U;
    chassis.remote_online = 1U;
    chassis.remote_stop = 0U;
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

static void setLegPose(chassis_leg_side_t side,
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

    assert(VMC_CalcJointTarget(config,
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

static void setSymmetricLegPose(float length_m, float phi0_rad)
{
    setLegPose(CHASSIS_LEFT, length_m, phi0_rad);
    setLegPose(CHASSIS_RIGHT, length_m, phi0_rad);
    Chassis_ControlUpdateLegState();
    assert(chassis.leg[CHASSIS_LEFT].valid == 1U);
    assert(chassis.leg[CHASSIS_RIGHT].valid == 1U);
}

static void setSingularLegFeedback(void)
{
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        uint8_t front_index = chassis_config.leg[side]
                                  .joint[CHASSIS_JOINT_FRONT]
                                  .motor_index;
        uint8_t back_index = chassis_config.leg[side]
                                 .joint[CHASSIS_JOINT_BACK]
                                 .motor_index;

        /* 转换后phi1==phi4且髋轴同心，主动杆端点B/D重合。 */
        chassis.dm_motor[front_index].position_rad = CHASSIS_PI;
        chassis.dm_motor[back_index].position_rad = 0.0f;
    }
    Chassis_ControlUpdateLegState();
}

static void assertZeroFinalOutput(void)
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

static void assertZeroWheelRequest(void)
{
    uint32_t index;

    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        assert(chassis.wheel_current_request[index] == 0);
    }
}

static void assertControlRequestsCalculated(void)
{
    float maximum_joint_request_nm = 0.0f;
    int32_t maximum_wheel_request = 0;
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(chassis.joint_torque_request_nm[index]);

        if (request_nm > maximum_joint_request_nm)
        {
            maximum_joint_request_nm = request_nm;
        }
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        int32_t request = chassis.wheel_current_request[index];

        if (request < 0)
        {
            request = -request;
        }
        if (request > maximum_wheel_request)
        {
            maximum_wheel_request = request;
        }
    }
    assert(maximum_joint_request_nm > 0.0f);
    assert(maximum_wheel_request > 0);
}

static void assertJointRequestsCalculated(void)
{
    float maximum_joint_request_nm = 0.0f;
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(chassis.joint_torque_request_nm[index]);

        if (request_nm > maximum_joint_request_nm)
        {
            maximum_joint_request_nm = request_nm;
        }
    }
    assert(maximum_joint_request_nm > 0.0f);
}

static void testBenchControl(void)
{
    float maximum_request_nm = 0.0f;
    int32_t maximum_wheel_current_request = 0;
    uint32_t index;

    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.05f;
    chassis.imu.yaw_rad = 0.70f;
    chassis.imu.yaw_total_rad = 0.70f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_BENCH);

    Chassis_BenchControlLoop();
    assert(chassis.state == CHASSIS_BENCH);
    assert(chassis.state_valid == 1U);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] - 0.15f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_RIGHT] - 0.15f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(chassis.lqr_output[CHASSIS_OUTPUT_LEFT_LEG] == 0.0f);
    assert(chassis.lqr_output[CHASSIS_OUTPUT_RIGHT_LEG] == 0.0f);
    assert(chassis.support_force_n[CHASSIS_LEFT] == 0.0f);
    assert(chassis.support_force_n[CHASSIS_RIGHT] == 0.0f);
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
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        int32_t current_request = chassis.wheel_current_request[index];

        if (current_request < 0)
        {
            current_request = -current_request;
        }
        assert(current_request <= chassis_config.wheel.current_limit);
        if (current_request > maximum_wheel_current_request)
        {
            maximum_wheel_current_request = current_request;
        }
    }
    assert(maximum_wheel_current_request > 0);
    assertZeroFinalOutput();
}

static void testRecoveryHandoff(void)
{
    uint32_t iteration;

    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.0f;
    chassis.imu.yaw_rad = 0.70f;
    chassis.imu.yaw_total_rad = 0.70f;
    setSymmetricLegPose(chassis_config.recovery.bench_leg_length_m,
                           chassis_config.recovery.bench_phi0_rad);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_FALLEN);

    for (iteration = 0U; iteration < 150U; iteration++)
    {
        chassis.joint_torque_nm[0] = 0.5f;
        Chassis_ControlUpdateLegState();
        Chassis_ControlUpdateState();
        if ((chassis.state == CHASSIS_FALLEN) ||
            (chassis.state == CHASSIS_FALLING_TO_STAND))
        {
            Chassis_RecoveryControlLoop();
        }
        if (chassis.state == CHASSIS_STANDING)
        {
            break;
        }
    }

    assert(chassis.state == CHASSIS_STANDING);
    assertZeroFinalOutput();
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] -
                 chassis_config.recovery.bench_leg_length_m) <
           TEST_TOLERANCE);
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);

    Chassis_ControlLoop();
    assert(chassis.state_valid == 1U);
    assert(chassis.target_leg_length_m[CHASSIS_LEFT] >
           chassis_config.recovery.bench_leg_length_m);
    assertZeroFinalOutput();
}

static void testFallenTimeout(void)
{
    float left_phi0_rad;

    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 1.20f;
    setSymmetricLegPose(0.30f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_FALLEN);

    left_phi0_rad = chassis.leg[CHASSIS_LEFT].phi0_rad;
    Chassis_RecoveryControlLoop();
    assert(chassis.state == CHASSIS_FALLEN);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] -
                 chassis_config.recovery.extended_leg_length_m) <
           TEST_TOLERANCE);
    assert(chassis.target_leg_phi0_rad[CHASSIS_LEFT] < left_phi0_rad);
    assertZeroWheelRequest();
    assertZeroFinalOutput();

    chassis.state_elapsed_s =
        chassis_config.recovery.fallen_timeout_s -
        chassis.control_dt_s * 0.5f;
    Chassis_RecoveryControlLoop();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assertZeroFinalOutput();

    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
}

static void testPrepareTimeout(void)
{
    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.0f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_FALLEN);

    Chassis_RecoveryControlLoop();
    assert(chassis.state == CHASSIS_FALLING_TO_STAND);
    assertZeroWheelRequest();
    chassis.state_elapsed_s =
        chassis_config.recovery.prepare_timeout_s -
        chassis.control_dt_s * 0.5f;
    Chassis_RecoveryControlLoop();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assertZeroFinalOutput();
}

static void testInvalidLegFeedback(void)
{
    Chassis_ControlInit();
    setOnlineFeedback();
    setSingularLegFeedback();
    assert(chassis.leg[CHASSIS_LEFT].valid == 0U);
    assert(chassis.leg[CHASSIS_RIGHT].valid == 0U);
    assert(chassis.leg[CHASSIS_LEFT].phi1_rad == 0.0f);
    assert(chassis.leg[CHASSIS_LEFT].phi4_rad == 0.0f);

    chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_BENCH);
    Chassis_BenchControlLoop();
    assert(chassis.state == CHASSIS_BENCH);
    assert(chassis.state_valid == 1U);
    assert(chassis.fault_flags == CHASSIS_FAULT_KINEMATICS);
    assertJointRequestsCalculated();
    assertZeroFinalOutput();
}

static void testInvalidLegFreezesRecovery(void)
{
    float elapsed_before;
    float stable_before;

    Chassis_ControlInit();
    setOnlineFeedback();
    setSingularLegFeedback();
    assert(chassis.leg[CHASSIS_LEFT].valid == 0U);
    assert(chassis.leg[CHASSIS_RIGHT].valid == 0U);

    chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_FALLEN);
    chassis.state_elapsed_s = 0.35f;
    chassis.state_stable_s = 0.12f;
    elapsed_before = chassis.state_elapsed_s;
    stable_before = chassis.state_stable_s;

    Chassis_RecoveryControlLoop();
    assert(chassis.state == CHASSIS_FALLEN);
    assert(chassis.state_elapsed_s == elapsed_before);
    assert(chassis.state_stable_s == stable_before);
    assert(chassis.state_valid == 1U);
    assert(chassis.fault_flags == CHASSIS_FAULT_KINEMATICS);
    assertJointRequestsCalculated();
    assertZeroFinalOutput();
}

static void testMathRecoveryChecksPostureBeforeOutput(void)
{
    Chassis_ControlInit();
    setOnlineFeedback();
    setSingularLegFeedback();
    chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);
    Chassis_ControlLoop();
    assert(chassis.state_valid == 1U);
    assert(chassis.fault_flags == CHASSIS_FAULT_KINEMATICS);
    assertZeroFinalOutput();

    chassis.imu.pitch_rad =
        chassis_config.recovery.standing_pitch_limit_rad + 0.1f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_ZERO_FORCE);
    assert(chassis.fault_flags == CHASSIS_FAULT_CONTROL);
    assertZeroFinalOutput();
}

static void testOutputFaultsKeepCalculation(void)
{
    uint8_t left_front_index;
    uint32_t output_faults;

    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.05f;
    chassis.imu.yaw_total_rad = 0.70f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    left_front_index = chassis_config.leg[CHASSIS_LEFT]
                           .joint[CHASSIS_JOINT_FRONT]
                           .motor_index;
    chassis.enabled = 0U;
    chassis.imu.initialized = 0U;
    chassis.imu.attitude_ready = 0U;
    chassis.imu.error_code = 1U;
    chassis.dm_motor[left_front_index].online = 0U;
    chassis.wheel_motor[CHASSIS_LEFT].online = 0U;
    chassis.can_tx_error_count = APP_CAN_TX_ERROR_MAX + 1U;
    chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);

    Chassis_ControlLoop();
    output_faults = CHASSIS_FAULT_DISABLED |
                    CHASSIS_FAULT_IMU |
                    CHASSIS_FAULT_DM_MOTOR |
                    CHASSIS_FAULT_DJI_MOTOR |
                    CHASSIS_FAULT_CAN;
    assert((chassis.fault_flags & output_faults) == output_faults);
    assert(chassis.state_valid == 1U);
    assert(fabsf(chassis.leg[CHASSIS_LEFT].length_m - 0.25f) <
           TEST_TOLERANCE);
    assert(fabsf(chassis.lqr_state[CHASSIS_STATE_THETA_B] - 0.05f) <
           TEST_TOLERANCE);
    assertControlRequestsCalculated();
    assertZeroFinalOutput();

    chassis.leg_length_pid[CHASSIS_LEFT].integral = 1.0f;
    chassis.roll_pid.integral = 2.0f;
    chassis.joint_angle_pid[left_front_index].integral = 3.0f;
    chassis.speed_kalman.state[0] = 4.0f;
    chassis.forward_position_m = 5.0f;
    chassis.imu.yaw_total_rad = 0.90f;
    chassis.enabled = 1U;
    chassis.imu.initialized = 1U;
    chassis.imu.attitude_ready = 1U;
    chassis.imu.error_code = 0U;
    chassis.dm_motor[left_front_index].online = 1U;
    chassis.wheel_motor[CHASSIS_LEFT].online = 1U;
    chassis.can_tx_error_count = 0U;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);
    assert(chassis.fault_flags == CHASSIS_FAULT_NONE);
    assert(chassis.leg_length_pid[CHASSIS_LEFT].integral == 0.0f);
    assert(chassis.roll_pid.integral == 0.0f);
    assert(chassis.joint_angle_pid[left_front_index].integral == 0.0f);
    assert(chassis.speed_kalman.state[0] == 0.0f);
    assert(chassis.forward_position_m == 0.0f);
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] - 0.90f) <
           TEST_TOLERANCE);

    Chassis_ControlLoop();
    assert(chassis.state_valid == 1U);
    assert(chassis.fault_flags == CHASSIS_FAULT_NONE);
    assertControlRequestsCalculated();
    assertZeroFinalOutput();
}

static void testContinuousPhi0AndYaw(void)
{
    float previous_phi0_total_rad;
    uint32_t side;

    Chassis_ControlInit();
    setOnlineFeedback();
    setSymmetricLegPose(0.25f, CHASSIS_PI - 0.02f);
    previous_phi0_total_rad =
        chassis.leg[CHASSIS_LEFT].phi0_total_rad;
    setSymmetricLegPose(0.25f, -CHASSIS_PI + 0.03f);
    assert(chassis.leg[CHASSIS_LEFT].phi0_rad < 0.0f);
    assert(chassis.leg[CHASSIS_LEFT].phi0_total_rad > CHASSIS_PI);
    assert(fabsf(chassis.leg[CHASSIS_LEFT].phi0_total_rad -
                  previous_phi0_total_rad - 0.05f) < TEST_TOLERANCE);

    chassis.imu.yaw_rad = -CHASSIS_PI + 0.04f;
    chassis.imu.yaw_total_rad = CHASSIS_PI + 0.04f;
    chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_BENCH);
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] -
                  (CHASSIS_PI + 0.04f)) < TEST_TOLERANCE);
    Chassis_BenchControlLoop();
    assert(fabsf(chassis.lqr_state[CHASSIS_STATE_FAI] -
                  (CHASSIS_PI + 0.04f)) < TEST_TOLERANCE);

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        chassis.leg[side].phi0_total_rad += 2.0f * CHASSIS_PI;
    }
    Chassis_ControlLoop();
    assert(chassis.lqr_state[CHASSIS_STATE_THETA_L] >= -CHASSIS_PI);
    assert(chassis.lqr_state[CHASSIS_STATE_THETA_L] < CHASSIS_PI);
    assert(chassis.lqr_state[CHASSIS_STATE_THETA_R] >= -CHASSIS_PI);
    assert(chassis.lqr_state[CHASSIS_STATE_THETA_R] < CHASSIS_PI);
}

static void testRemoteFaultsKeepCalculation(void)
{
    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.04f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_FOLLOW;
    chassis.remote_online = 0U;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);

    Chassis_ControlLoop();
    assert((chassis.fault_flags & CHASSIS_FAULT_REMOTE) != 0U);
    assert(chassis.state_valid == 1U);
    assertControlRequestsCalculated();
    assertZeroFinalOutput();

    chassis.remote_online = 1U;
    chassis.remote_stop = 1U;
    Chassis_ControlUpdateState();
    Chassis_ControlLoop();
    assert((chassis.fault_flags & CHASSIS_FAULT_REMOTE) != 0U);
    assert(chassis.state_valid == 1U);
    assertControlRequestsCalculated();
    assertZeroFinalOutput();

    chassis.remote_stop = 0U;
    Chassis_ControlUpdateState();
    assert(chassis.fault_flags == CHASSIS_FAULT_NONE);
    Chassis_ControlLoop();
    assert(chassis.state_valid == 1U);
    assert(chassis.fault_flags == CHASSIS_FAULT_NONE);
}

static void testRemoteMotionTargets(void)
{
    float initial_yaw_target_rad;
    float initial_leg_target_m;

    Chassis_ControlInit();
    setOnlineFeedback();
    chassis.imu.pitch_rad = 0.04f;
    chassis.imu.yaw_total_rad = CHASSIS_PI + 0.04f;
    setSymmetricLegPose(0.25f, CHASSIS_HALF_PI);
    chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_ControlUpdateState();
    assert(chassis.state == CHASSIS_STANDING);

    initial_yaw_target_rad = chassis.target_state[CHASSIS_STATE_FAI];
    initial_leg_target_m = chassis.target_leg_length_m[CHASSIS_LEFT];
    chassis.remote_target_valid = 1U;
    chassis.motion_command.forward_speed_mps = APP_RC_MAX_VEL;
    chassis.motion_command.yaw_target_rad = initial_yaw_target_rad + 0.5f;
    chassis.motion_command.leg_length_m = APP_RC_LEG_S;
    chassis.forward_position_m = 5.0f;

    Chassis_ControlLoop();

    assert(fabsf(chassis.target_state[CHASSIS_STATE_DOT_S] -
                 APP_RC_VEL_RATE * chassis.control_dt_s) < TEST_TOLERANCE);
    assert(fabsf(chassis.target_state[CHASSIS_STATE_FAI] -
                 (initial_yaw_target_rad +
                  APP_RC_YAW_RATE * chassis.control_dt_s)) < TEST_TOLERANCE);
    assert(chassis.target_state[CHASSIS_STATE_FAI] > CHASSIS_PI);
    assert(fabsf(chassis.target_state[CHASSIS_STATE_DOT_FAI] -
                 APP_RC_YAW_RATE) < TEST_TOLERANCE);
    assert(fabsf(chassis.target_leg_length_m[CHASSIS_LEFT] -
                 (initial_leg_target_m -
                  chassis_config.recovery.standing_length_rate_mps *
                      chassis.control_dt_s)) < TEST_TOLERANCE);
    assert(chassis.forward_position_m == 0.0f);
    assert(chassis.state_valid == 1U);
    assertZeroFinalOutput();
}

int main(void)
{
    testBenchControl();
    testRecoveryHandoff();
    testFallenTimeout();
    testPrepareTimeout();
    testInvalidLegFeedback();
    testInvalidLegFreezesRecovery();
    testMathRecoveryChecksPostureBeforeOutput();
    testOutputFaultsKeepCalculation();
    testContinuousPhi0AndYaw();
    testRemoteFaultsKeepCalculation();
    testRemoteMotionTargets();
    return 0;
}
