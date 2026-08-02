#include "chassis_control.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TEST_TOLERANCE 2.0e-5f

static void set_online_feedback(void)
{
    uint32_t index;

    Chassis.enable_flag = 1U;
    Chassis.dt = Chassis_Config.default_dt;
    Chassis.imu.init_flag = 1U;
    Chassis.imu.attitude_flag = 1U;
    Chassis.imu.error_code = 0U;
    Chassis.remote_online_flag = 1U;
    Chassis.remote_stop_flag = 0U;
    Chassis.can_error_count = 0U;
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        Chassis.dm_motor[index].online_flag = 1U;
        Chassis.dm_motor[index].speed_radps = 0.0f;
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        Chassis.wheel_motor[index].online_flag = 1U;
        Chassis.wheel_motor[index].speed_rpm = 0;
    }
}

static void set_leg_pose(Chassis_Side_t side,
                       float L0,
                       float phi0)
{
    const Chassis_Leg_Config_t *config = &Chassis_Config.leg[side];
    Chassis_Leg_t current_leg = {
        .phi1 = CHASSIS_PI,
        .phi4 = 0.0f,
    };
    VMC_Joint_Target_t target;
    uint8_t front_index = config->joint[CHASSIS_JOINT_FRONT].motor_index;
    uint8_t back_index = config->joint[CHASSIS_JOINT_BACK].motor_index;

    assert(VMC_Inverse_Calc(config,
                               &current_leg,
                               L0,
                               phi0,
                               &target) == 1U);
    Chassis.dm_motor[front_index].position_rad =
        (target.phi1 -
         config->joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config->joint[CHASSIS_JOINT_FRONT].angle_scale;
    Chassis.dm_motor[back_index].position_rad =
        (target.phi4 -
         config->joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config->joint[CHASSIS_JOINT_BACK].angle_scale;
}

static void set_symmetric_leg_pose(float L0, float phi0)
{
    set_leg_pose(CHASSIS_LEFT, L0, phi0);
    set_leg_pose(CHASSIS_RIGHT, L0, phi0);
    Chassis_Leg_Update();
    assert(Chassis.leg[CHASSIS_LEFT].valid_flag == 1U);
    assert(Chassis.leg[CHASSIS_RIGHT].valid_flag == 1U);
}

static void set_singular_leg_feedback(void)
{
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        uint8_t front_index = Chassis_Config.leg[side]
                                  .joint[CHASSIS_JOINT_FRONT]
                                  .motor_index;
        uint8_t back_index = Chassis_Config.leg[side]
                                 .joint[CHASSIS_JOINT_BACK]
                                 .motor_index;

        /* 转换后phi1==phi4且髋轴同心，主动杆端点B/D重合。 */
        Chassis.dm_motor[front_index].position_rad = CHASSIS_PI;
        Chassis.dm_motor[back_index].position_rad = 0.0f;
    }
    Chassis_Leg_Update();
}

static void assert_zero_output(void)
{
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        assert(Chassis.output.T_joint[index] == 0.0f);
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        assert(Chassis.output.I_wheel[index] == 0);
    }
    assert(Chassis.output.safe_flag == 1U);
}

static void assert_zero_wheel_request(void)
{
    uint32_t index;

    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        assert(Chassis.output.I_wheel_req[index] == 0);
    }
}

static void assert_control_request(void)
{
    float maximum_joint_request_nm = 0.0f;
    int32_t maximum_wheel_request = 0;
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(Chassis.output.T_joint_req[index]);

        if (request_nm > maximum_joint_request_nm)
        {
            maximum_joint_request_nm = request_nm;
        }
    }
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        int32_t request = Chassis.output.I_wheel_req[index];

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

static void assert_joint_request(void)
{
    float maximum_joint_request_nm = 0.0f;
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(Chassis.output.T_joint_req[index]);

        if (request_nm > maximum_joint_request_nm)
        {
            maximum_joint_request_nm = request_nm;
        }
    }
    assert(maximum_joint_request_nm > 0.0f);
}

static void test_bench_control(void)
{
    float maximum_request_nm = 0.0f;
    int32_t maximum_wheel_current_request = 0;
    uint32_t index;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.05f;
    Chassis.imu.yaw = 0.70f;
    Chassis.imu.yaw_total = 0.70f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_BENCH);

    Chassis_Bench();
    assert(Chassis.state == CHASSIS_BENCH);
    assert(Chassis.calc_flag == 1U);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - 0.15f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].target_L0 - 0.15f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(Chassis.leg[CHASSIS_LEFT].Tp == 0.0f);
    assert(Chassis.leg[CHASSIS_RIGHT].Tp == 0.0f);
    assert(Chassis.leg[CHASSIS_LEFT].F0 == 0.0f);
    assert(Chassis.leg[CHASSIS_RIGHT].F0 == 0.0f);
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(Chassis.output.T_joint_req[index]);

        assert(request_nm <=
               Chassis_Config.recovery.joint_T_limit +
                   TEST_TOLERANCE);
        if (request_nm > maximum_request_nm)
        {
            maximum_request_nm = request_nm;
        }
    }
    assert(maximum_request_nm > 0.0f);
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        int32_t current_request = Chassis.output.I_wheel_req[index];

        if (current_request < 0)
        {
            current_request = -current_request;
        }
        assert(current_request <= Chassis_Config.wheel.I_limit);
        if (current_request > maximum_wheel_current_request)
        {
            maximum_wheel_current_request = current_request;
        }
    }
    assert(maximum_wheel_current_request > 0);
    assert_zero_output();
}

static void test_recovery_handoff(void)
{
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.0f;
    Chassis.imu.yaw = 0.70f;
    Chassis.imu.yaw_total = 0.70f;
    set_symmetric_leg_pose(Chassis_Config.recovery.bench_L0,
                           Chassis_Config.recovery.bench_phi0);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    for (iteration = 0U; iteration < 150U; iteration++)
    {
        Chassis.output.T_joint[0] = 0.5f;
        Chassis_Leg_Update();
        Chassis_State_Update();
        if ((Chassis.state == CHASSIS_FALLEN) ||
            (Chassis.state == CHASSIS_FALLING_TO_STAND))
        {
            Chassis_Recovery();
        }
        if (Chassis.state == CHASSIS_STANDING)
        {
            break;
        }
    }

    assert(Chassis.state == CHASSIS_STANDING);
    assert_zero_output();
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 -
                 Chassis_Config.recovery.bench_L0) <
           TEST_TOLERANCE);
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis_Control();
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.leg[CHASSIS_LEFT].target_L0 >
           Chassis_Config.recovery.bench_L0);
    assert_zero_output();
}

static void test_fallen_timeout(void)
{
    float left_phi0_rad;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 1.20f;
    set_symmetric_leg_pose(0.30f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    left_phi0_rad = Chassis.leg[CHASSIS_LEFT].phi0;
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLEN);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 -
                 Chassis_Config.recovery.extend_L0) <
           TEST_TOLERANCE);
    assert(Chassis.leg[CHASSIS_LEFT].target_phi0 < left_phi0_rad);
    assert_zero_wheel_request();
    assert_zero_output();

    Chassis.state_time =
        Chassis_Config.recovery.fallen_timeout -
        Chassis.dt * 0.5f;
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assert_zero_output();

    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_RECOVERY_TIMEOUT);
}

static void test_prepare_timeout(void)
{
    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.0f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLING_TO_STAND);
    assert_zero_wheel_request();
    Chassis.state_time =
        Chassis_Config.recovery.prepare_timeout -
        Chassis.dt * 0.5f;
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_RECOVERY_TIMEOUT);
    assert_zero_output();
}

static void test_invalid_leg_feedback(void)
{
    Chassis_Init();
    set_online_feedback();
    set_singular_leg_feedback();
    assert(Chassis.leg[CHASSIS_LEFT].valid_flag == 0U);
    assert(Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U);
    assert(Chassis.leg[CHASSIS_LEFT].phi1 == 0.0f);
    assert(Chassis.leg[CHASSIS_LEFT].phi4 == 0.0f);

    Chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_BENCH);
    Chassis_Bench();
    assert(Chassis.state == CHASSIS_BENCH);
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.fault == CHASSIS_FAULT_KINEMATICS);
    assert_joint_request();
    assert_zero_output();
}

static void test_invalid_leg_recovery(void)
{
    float elapsed_before;
    float stable_before;

    Chassis_Init();
    set_online_feedback();
    set_singular_leg_feedback();
    assert(Chassis.leg[CHASSIS_LEFT].valid_flag == 0U);
    assert(Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U);

    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);
    Chassis.state_time = 0.35f;
    Chassis.stable_time = 0.12f;
    elapsed_before = Chassis.state_time;
    stable_before = Chassis.stable_time;

    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLEN);
    assert(Chassis.state_time == elapsed_before);
    assert(Chassis.stable_time == stable_before);
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.fault == CHASSIS_FAULT_KINEMATICS);
    assert_joint_request();
    assert_zero_output();
}

static void test_recovery_posture(void)
{
    Chassis_Init();
    set_online_feedback();
    set_singular_leg_feedback();
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    Chassis_Control();
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.fault == CHASSIS_FAULT_KINEMATICS);
    assert_zero_output();

    Chassis.imu.pitch =
        Chassis_Config.recovery.pitch_limit + 0.1f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_CONTROL);
    assert_zero_output();
}

static void test_fault_calculation(void)
{
    uint8_t left_front_index;
    uint32_t output_fault;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.05f;
    Chassis.imu.yaw_total = 0.70f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    left_front_index = Chassis_Config.leg[CHASSIS_LEFT]
                           .joint[CHASSIS_JOINT_FRONT]
                           .motor_index;
    Chassis.enable_flag = 0U;
    Chassis.imu.init_flag = 0U;
    Chassis.imu.attitude_flag = 0U;
    Chassis.imu.error_code = 1U;
    Chassis.dm_motor[left_front_index].online_flag = 0U;
    Chassis.wheel_motor[CHASSIS_LEFT].online_flag = 0U;
    Chassis.can_error_count = APP_CAN_TX_ERROR_MAX + 1U;
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis_Control();
    output_fault = CHASSIS_FAULT_DISABLED |
                    CHASSIS_FAULT_IMU |
                    CHASSIS_FAULT_DM_MOTOR |
                    CHASSIS_FAULT_DJI_MOTOR |
                    CHASSIS_FAULT_CAN;
    assert((Chassis.fault & output_fault) == output_fault);
    assert(Chassis.calc_flag == 1U);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].L0 - 0.25f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_B] - 0.05f) <
           TEST_TOLERANCE);
    assert_control_request();
    assert_zero_output();

    Chassis.leg_length_pid[CHASSIS_LEFT].integral = 1.0f;
    Chassis.roll_pid.integral = 2.0f;
    Chassis.joint_angle_pid[left_front_index].integral = 3.0f;
    Chassis.speed_kalman.state[0] = 4.0f;
    Chassis.body.s = 5.0f;
    Chassis.imu.yaw_total = 0.90f;
    Chassis.enable_flag = 1U;
    Chassis.imu.init_flag = 1U;
    Chassis.imu.attitude_flag = 1U;
    Chassis.imu.error_code = 0U;
    Chassis.dm_motor[left_front_index].online_flag = 1U;
    Chassis.wheel_motor[CHASSIS_LEFT].online_flag = 1U;
    Chassis.can_error_count = 0U;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    assert(Chassis.leg_length_pid[CHASSIS_LEFT].integral == 0.0f);
    assert(Chassis.roll_pid.integral == 0.0f);
    assert(Chassis.joint_angle_pid[left_front_index].integral == 0.0f);
    assert(Chassis.speed_kalman.state[0] == 0.0f);
    assert(Chassis.body.s == 0.0f);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - 0.90f) <
           TEST_TOLERANCE);

    Chassis_Control();
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    assert_control_request();
    assert_zero_output();
}

static void test_continuous_angle(void)
{
    float previous_phi0_total_rad;
    uint32_t side;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_PI - 0.02f);
    previous_phi0_total_rad =
        Chassis.leg[CHASSIS_LEFT].phi0_total;
    set_symmetric_leg_pose(0.25f, -CHASSIS_PI + 0.03f);
    assert(Chassis.leg[CHASSIS_LEFT].phi0 < 0.0f);
    assert(Chassis.leg[CHASSIS_LEFT].phi0_total > CHASSIS_PI);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].phi0_total -
                  previous_phi0_total_rad - 0.05f) < TEST_TOLERANCE);

    Chassis.imu.yaw = -CHASSIS_PI + 0.04f;
    Chassis.imu.yaw_total = CHASSIS_PI + 0.04f;
    Chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_BENCH);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                  (CHASSIS_PI + 0.04f)) < TEST_TOLERANCE);
    Chassis_Bench();
    assert(fabsf(Chassis.lqr.x[CHASSIS_STATE_FAI] -
                  (CHASSIS_PI + 0.04f)) < TEST_TOLERANCE);

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].phi0_total += 2.0f * CHASSIS_PI;
    }
    Chassis_Control();
    assert(Chassis.lqr.x[CHASSIS_STATE_THETA_L] >= -CHASSIS_PI);
    assert(Chassis.lqr.x[CHASSIS_STATE_THETA_L] < CHASSIS_PI);
    assert(Chassis.lqr.x[CHASSIS_STATE_THETA_R] >= -CHASSIS_PI);
    assert(Chassis.lqr.x[CHASSIS_STATE_THETA_R] < CHASSIS_PI);
}

static void test_remote_fault(void)
{
    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.04f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis.remote_online_flag = 0U;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis_Control();
    assert((Chassis.fault & CHASSIS_FAULT_REMOTE) != 0U);
    assert(Chassis.calc_flag == 1U);
    assert_control_request();
    assert_zero_output();

    Chassis.remote_online_flag = 1U;
    Chassis.remote_stop_flag = 1U;
    Chassis_State_Update();
    Chassis_Control();
    assert((Chassis.fault & CHASSIS_FAULT_REMOTE) != 0U);
    assert(Chassis.calc_flag == 1U);
    assert_control_request();
    assert_zero_output();

    Chassis.remote_stop_flag = 0U;
    Chassis_State_Update();
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    Chassis_Control();
    assert(Chassis.calc_flag == 1U);
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
}

static void test_remote_goal(void)
{
    float initial_yaw_target_rad;
    float initial_leg_target_m;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.04f;
    Chassis.imu.yaw_total = CHASSIS_PI + 0.04f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    initial_yaw_target_rad = Chassis.lqr.target[CHASSIS_STATE_FAI];
    initial_leg_target_m = Chassis.leg[CHASSIS_LEFT].target_L0;
    Chassis.remote_target_flag = 1U;
    Chassis.goal.d_s = APP_RC_MAX_VEL;
    Chassis.goal.fai = initial_yaw_target_rad + 0.5f;
    Chassis.goal.L0 = APP_RC_LEG_S;
    Chassis.body.s = 5.0f;

    Chassis_Control();

    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_DOT_S] -
                 APP_RC_VEL_RATE * Chassis.dt) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                 (initial_yaw_target_rad +
                  APP_RC_YAW_RATE * Chassis.dt)) < TEST_TOLERANCE);
    assert(Chassis.lqr.target[CHASSIS_STATE_FAI] > CHASSIS_PI);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] -
                 APP_RC_YAW_RATE) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 -
                 (initial_leg_target_m -
                  Chassis_Config.recovery.L0_rate *
                      Chassis.dt)) < TEST_TOLERANCE);
    assert(Chassis.body.s == 0.0f);
    assert(Chassis.calc_flag == 1U);
    assert_zero_output();
}

static void test_top_projection(void)
{
    float yaw_anchor_rad = 0.40f;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.yaw_total = yaw_anchor_rad;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_TOP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    assert(fabsf(Chassis.top_fai - yaw_anchor_rad) <
           TEST_TOLERANCE);

    Chassis.remote_target_flag = 1U;
    Chassis.goal.d_s = 0.25f;
    Chassis.goal.d_y = -0.10f;
    Chassis.goal.d_fai = 2.0f;
    Chassis.imu.yaw_total = yaw_anchor_rad + CHASSIS_HALF_PI;
    Chassis_Control();

    assert(fabsf(Chassis.top_d_s + 0.10f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_DOT_S] +
                 APP_RC_VEL_RATE * Chassis.dt) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_DOT_FAI] -
                 APP_RC_YAW_RATE * Chassis.dt) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                 Chassis.imu.yaw_total) < TEST_TOLERANCE);
    assert(Chassis.lqr.scale[CHASSIS_STATE_S] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_FAI] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_DOT_S] == 1.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_DOT_FAI] == 1.0f);
    assert_zero_output();
}

static void test_step_phases(void)
{
    int32_t maximum_wheel_request = 0;
    uint32_t iteration;
    uint32_t side;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(Chassis_Config.step.approach_L0,
                        CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STEP);
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);

    for (iteration = 0U; iteration < 150U; iteration++)
    {
        Chassis_Step();
        if (Chassis.step_phase == CHASSIS_STEP_APPROACH)
        {
            break;
        }
    }
    assert(Chassis.step_phase == CHASSIS_STEP_APPROACH);

    Chassis.goal.d_s =
        Chassis_Config.step.approach_d_s;
    set_symmetric_leg_pose(Chassis_Config.step.approach_L0,
                        CHASSIS_HALF_PI + 0.80f);
    for (side = 0U; side < APP_WHEEL_COUNT; side++)
    {
        Chassis.wheel_motor[side].current =
            (int16_t)(Chassis_Config.step.contact_T_fb *
                      Chassis_Config.wheel.T_to_I * 1.5f);
    }
    for (iteration = 0U; iteration < 100U; iteration++)
    {
        Chassis_Step();
        if (Chassis.step_phase == CHASSIS_STEP_CLIMB)
        {
            break;
        }
    }
    assert(Chassis.step_contact_latch_flag[CHASSIS_LEFT] == 1U);
    assert(Chassis.step_contact_latch_flag[CHASSIS_RIGHT] == 1U);
    assert(Chassis.step_phase == CHASSIS_STEP_CLIMB);
    for (side = 0U; side < APP_WHEEL_COUNT; side++)
    {
        int32_t request = Chassis.output.I_wheel_req[side];

        if (request < 0)
        {
            request = -request;
        }
        if (request > maximum_wheel_request)
        {
            maximum_wheel_request = request;
        }
        assert(Chassis.output.I_wheel[side] == 0);
    }
    assert(maximum_wheel_request > 0);

    set_symmetric_leg_pose(Chassis_Config.step.retract_L0,
                        CHASSIS_HALF_PI +
                            Chassis_Config.step.peak_theta);
    for (iteration = 0U; iteration < 150U; iteration++)
    {
        Chassis_Step();
        if (Chassis.step_phase == CHASSIS_STEP_RECOVER)
        {
            break;
        }
    }
    assert(Chassis.step_phase == CHASSIS_STEP_RECOVER);
    assert_zero_output();

    set_symmetric_leg_pose(Chassis.goal.L0,
                        CHASSIS_HALF_PI +
                            Chassis_Config.step.recover_theta);
    for (iteration = 0U; iteration < 150U; iteration++)
    {
        Chassis_Step();
        if (Chassis.state == CHASSIS_STANDING)
        {
            break;
        }
    }
    assert(Chassis.state == CHASSIS_STANDING);
    assert_zero_output();
}

static void test_step_timeout(void)
{
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STEP);
    Chassis.state_time =
        Chassis_Config.step.prepare_timeout -
        Chassis.dt * 0.5f;

    Chassis_Step();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_STEP_TIMEOUT);
    assert_zero_output();
}

static void test_output_disabled(void)
{
    assert(APP_CHASSIS_OUTPUT_ENABLE == 0U);
    assert(Chassis_Config.output.joint_flag == 0U);
    assert(Chassis_Config.output.wheel_flag == 0U);
}

int main(void)
{
    test_output_disabled();
    test_bench_control();
    test_recovery_handoff();
    test_fallen_timeout();
    test_prepare_timeout();
    test_invalid_leg_feedback();
    test_invalid_leg_recovery();
    test_recovery_posture();
    test_fault_calculation();
    test_continuous_angle();
    test_remote_fault();
    test_remote_goal();
    test_top_projection();
    test_step_phases();
    test_step_timeout();
    return 0;
}
