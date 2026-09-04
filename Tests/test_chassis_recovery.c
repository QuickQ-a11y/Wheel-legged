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
    uint8_t phi1_index = config->joint[CHASSIS_JOINT_PHI1].motor_index;
    uint8_t phi4_index = config->joint[CHASSIS_JOINT_PHI4].motor_index;

    assert(VMC_Inverse_Calc(config,
                               &current_leg,
                               L0,
                               phi0,
                               &target) == 1U);
    Chassis.dm_motor[phi1_index].position_rad =
        (target.phi1 -
         config->joint[CHASSIS_JOINT_PHI1].angle_offset_rad) /
        (config->joint[CHASSIS_JOINT_PHI1].scale *
         config->joint[CHASSIS_JOINT_PHI1].ratio);
    Chassis.dm_motor[phi4_index].position_rad =
        (target.phi4 -
         config->joint[CHASSIS_JOINT_PHI4].angle_offset_rad) /
        (config->joint[CHASSIS_JOINT_PHI4].scale *
         config->joint[CHASSIS_JOINT_PHI4].ratio);
}

static void set_symmetric_leg_pose(float L0, float phi0)
{
    set_leg_pose(CHASSIS_LEFT, L0, phi0);
    set_leg_pose(CHASSIS_RIGHT, L0, phi0);
    Chassis_Leg_Update();
    assert(Chassis.leg[CHASSIS_LEFT].valid_flag == 1U);
    assert(Chassis.leg[CHASSIS_RIGHT].valid_flag == 1U);
}

/*
 * 按机体真实俯仰角摆好重力矢量。倒地判据吃的是 Chassis.fall_pitch，
 * 它由原始加速度算出来，只设 imu.pitch 已经表达不了"车躺着"了。
 * imu.pitch 按 EKF 的 asinf 特性同步折返，复现真实读数。
 * fall_pitch 的低通要跑上百拍才收敛，这里直接放稳态值。
 */
static void set_fall_pose(float pitch_rad)
{
    const float gravity_mps2 = 9.80665f;

    Chassis.imu.accel_raw[0] =
        gravity_mps2 * sinf(pitch_rad) /
        Chassis_Config.imu.fall_accel_x_scale;
    Chassis.imu.accel_raw[1] = 0.0f;
    Chassis.imu.accel_raw[2] =
        gravity_mps2 * cosf(pitch_rad) /
        Chassis_Config.imu.fall_accel_z_scale;
    Chassis.fall_pitch = pitch_rad;
    Chassis.imu.pitch = asinf(sinf(pitch_rad));
}

static void set_singular_leg_feedback(void)
{
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        uint8_t phi1_index = Chassis_Config.leg[side]
                                  .joint[CHASSIS_JOINT_PHI1]
                                  .motor_index;
        uint8_t phi4_index = Chassis_Config.leg[side]
                                 .joint[CHASSIS_JOINT_PHI4]
                                 .motor_index;

        /* 转换后phi1==phi4且髋轴同心，主动杆端点B/D重合。 */
        Chassis.dm_motor[phi1_index].position_rad =
            (CHASSIS_HALF_PI -
             Chassis_Config.leg[side]
                 .joint[CHASSIS_JOINT_PHI1]
                 .angle_offset_rad) /
            (Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].scale *
             Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].ratio);
        Chassis.dm_motor[phi4_index].position_rad =
            (CHASSIS_HALF_PI -
             Chassis_Config.leg[side]
                 .joint[CHASSIS_JOINT_PHI4]
                 .angle_offset_rad) /
            (Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].scale *
             Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].ratio);
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
    Chassis.imu.yaw_total = 0.70f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_BENCH);

    Chassis_Bench();
    assert(Chassis.state == CHASSIS_BENCH);
    /* 进入板凳锁存当前实际姿态，遥控调节速率为零时目标保持不动。 */
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 -
                 Chassis.leg[CHASSIS_LEFT].L0) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].target_L0 -
                 Chassis.leg[CHASSIS_RIGHT].L0) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - 0.70f) <
           TEST_TOLERANCE);
    assert(Chassis.leg[CHASSIS_LEFT].Tp == 0.0f);
    assert(Chassis.leg[CHASSIS_RIGHT].Tp == 0.0f);
    assert(Chassis.leg[CHASSIS_LEFT].F0 == 0.0f);
    assert(Chassis.leg[CHASSIS_RIGHT].F0 == 0.0f);
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        float request_nm = fabsf(Chassis.output.T_joint_req[index]);

        /* 请求量不再限幅，其量级由串级PID的输出限幅界定。 */
        assert(request_nm <=
               Chassis_Config.recovery.joint_speed_pid.outputLimit +
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
        /* 轮通道唯一限幅在力矩侧，电流上限由换算得到。 */
        assert((float)current_request <=
               Chassis_Config.wheel.T_limit * Chassis_Config.wheel.T_to_I +
                   TEST_TOLERANCE);
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

    /*
     * 交接回站立后腿长目标按L0_rate斜坡走向goal.L0，不再一拍直接接管。
     * 这段落差正是自救结束停在bench_L0、随后收腿到goal.L0的那一下。
     */
    for (iteration = 0U; iteration < 1000U; iteration++)
    {
        float previous_L0_m = Chassis.leg[CHASSIS_LEFT].target_L0;

        Chassis_Control();
        assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - previous_L0_m) <=
               Chassis_Config.recovery.L0_rate * Chassis.dt + TEST_TOLERANCE);
    }
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - Chassis.goal.L0) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].target_L0 - Chassis.goal.L0) <
           TEST_TOLERANCE);
    assert_zero_output();
}

static void test_fallen_timeout(void)
{
    float left_phi0_rad;
    float left_L0_m;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.29f, CHASSIS_HALF_PI);
    set_fall_pose(1.20f);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    left_phi0_rad = Chassis.leg[CHASSIS_LEFT].phi0;
    left_L0_m = Chassis.leg[CHASSIS_LEFT].L0;
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLEN);
    /* 腿长和腿角目标都按速率推进，单拍只走一小步，不再阶跃到位。 */
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 -
                 Chassis_Config.recovery.extend_L0) <
           fabsf(left_L0_m - Chassis_Config.recovery.extend_L0));
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
    assert(Chassis.leg[CHASSIS_LEFT].phi1 == CHASSIS_HALF_PI);
    assert(Chassis.leg[CHASSIS_LEFT].phi4 == CHASSIS_HALF_PI);

    Chassis.mode = CHASSIS_MODE_BENCH;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_BENCH);
    Chassis_Bench();
    assert(Chassis.state == CHASSIS_BENCH);
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

    /* 几何无解时冻结阶段推进和稳定计时，但总计时必须继续走。 */
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLEN);
    assert(Chassis.state_time > elapsed_before);
    assert(Chassis.stable_time == stable_before);
    assert(Chassis.fault == CHASSIS_FAULT_KINEMATICS);
    assert_joint_request();
    assert_zero_output();

    /*
     * 几何长期无解必须能超时退出：否则恢复永不结束，mode锁在SELF_SAVE、
     * state锁在FALLEN，遥控失效只能断电。
     */
    Chassis.state_time = Chassis_Config.recovery.fallen_timeout;
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_ZERO_FORCE);
    assert(Chassis.fault == CHASSIS_FAULT_RECOVERY_TIMEOUT);
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

/*
 * 轮输出关闭时不驱动轮电机，其在线状态不应参与输出封锁，
 * 否则只调髋关节时轮电调未上电就会连带封锁关节力矩。
 */
static void test_wheel_offline_gate(void)
{
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis.wheel_motor[CHASSIS_LEFT].online_flag = 0U;
    Chassis.wheel_motor[CHASSIS_RIGHT].online_flag = 0U;
    Chassis_Control();
    if (Chassis_Config.output.wheel_flag == 0U)
    {
        assert((Chassis.fault & CHASSIS_FAULT_DJI_MOTOR) == 0U);
    }
    else
    {
        assert((Chassis.fault & CHASSIS_FAULT_DJI_MOTOR) != 0U);
    }

    /* DM 离线在任何配置下都必须封锁。 */
    Chassis.dm_motor[0].online_flag = 0U;
    Chassis_Control();
    assert((Chassis.fault & CHASSIS_FAULT_DM_MOTOR) != 0U);
}

static void test_fault_calculation(void)
{
    uint8_t left_phi1_index;
    uint32_t output_fault;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.05f;
    Chassis.imu.yaw_total = 0.70f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    left_phi1_index = Chassis_Config.leg[CHASSIS_LEFT]
                          .joint[CHASSIS_JOINT_PHI1]
                          .motor_index;
    Chassis.enable_flag = 0U;
    Chassis.imu.init_flag = 0U;
    Chassis.imu.attitude_flag = 0U;
    Chassis.imu.error_code = 1U;
    Chassis.dm_motor[left_phi1_index].online_flag = 0U;
    Chassis.wheel_motor[CHASSIS_LEFT].online_flag = 0U;
    Chassis.can_error_count = APP_CAN_TX_ERROR_MAX + 1U;
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis_Control();
    output_fault = CHASSIS_FAULT_DISABLED |
                    CHASSIS_FAULT_IMU |
                    CHASSIS_FAULT_DM_MOTOR |
                    CHASSIS_FAULT_CAN;
    /* 轮离线是否封锁取决于轮输出是否打开。 */
    if (Chassis_Config.output.wheel_flag != 0U)
    {
        output_fault |= CHASSIS_FAULT_DJI_MOTOR;
    }
    assert((Chassis.fault & output_fault) == output_fault);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].L0 - 0.25f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_B] - 0.05f) <
           TEST_TOLERANCE);
    assert_control_request();
    assert_zero_output();

    Chassis.leg_length_pid.integral = 1.0f;
    Chassis.roll_pid.integral = 2.0f;
    Chassis.joint_angle_pid[left_phi1_index].integral = 3.0f;
    Chassis.speed_kalman.state[0] = 4.0f;
    Chassis.body.s = 5.0f;
    Chassis.imu.yaw_total = 0.90f;
    Chassis.enable_flag = 1U;
    Chassis.imu.init_flag = 1U;
    Chassis.imu.attitude_flag = 1U;
    Chassis.imu.error_code = 0U;
    Chassis.dm_motor[left_phi1_index].online_flag = 1U;
    Chassis.wheel_motor[CHASSIS_LEFT].online_flag = 1U;
    Chassis.can_error_count = 0U;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    assert(Chassis.leg_length_pid.integral == 0.0f);
    assert(Chassis.roll_pid.integral == 0.0f);
    assert(Chassis.joint_angle_pid[left_phi1_index].integral == 0.0f);
    assert(Chassis.speed_kalman.state[0] == 0.0f);
    assert(Chassis.body.s == 0.0f);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - 0.90f) <
           TEST_TOLERANCE);

    Chassis_Control();
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    assert_control_request();
    assert_zero_output();
}

static void test_lqr_realtime_leg_length(void)
{
    Chassis_Init();
    set_online_feedback();
    /* 两条腿都落在K的采样范围内时，拟合腿长直接取实时L0，不限幅。 */
    set_leg_pose(CHASSIS_LEFT, 0.14f, CHASSIS_HALF_PI);
    set_leg_pose(CHASSIS_RIGHT, 0.24f, CHASSIS_HALF_PI);
    Chassis_Leg_Update();
    assert(Chassis.leg[CHASSIS_LEFT].valid_flag == 1U);
    assert(Chassis.leg[CHASSIS_RIGHT].valid_flag == 1U);

    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    Chassis_Control();

    assert(fabsf(Chassis.leg[CHASSIS_LEFT].K_L0_fit -
                  Chassis.leg[CHASSIS_LEFT].L0) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].K_L0_fit -
                  Chassis.leg[CHASSIS_RIGHT].L0) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].K_L0_fit -
                  Chassis.leg[CHASSIS_RIGHT].K_L0_fit) > 0.05f);
    assert(Chassis.lqr.limit_flag == 0U);
    assert_zero_output();

    /* 超出采样范围时必须限幅到边界并置位标志，禁止外推。
     * 越界腿长由配置上界推出，换车时不需要改这个测试。 */
    set_leg_pose(CHASSIS_RIGHT,
                 Chassis_Config.lqr.L0_max + 0.05f,
                 CHASSIS_HALF_PI);
    Chassis_Leg_Update();
    Chassis_Control();
    assert(Chassis.leg[CHASSIS_RIGHT].L0 >
           Chassis_Config.lqr.L0_max + TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].K_L0_fit -
                  Chassis_Config.lqr.L0_max) < TEST_TOLERANCE);
    assert(Chassis.lqr.limit_flag == 1U);
    assert_zero_output();
}

static void test_vertical_theta_balance(void)
{
    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.20f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI - 0.20f);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    Chassis_Control();

    assert(fabsf(Chassis.leg[CHASSIS_LEFT].theta) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].theta) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_L]) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_R]) < TEST_TOLERANCE);
    assert_zero_output();
}

static void test_lqr_leg_length_limit(void)
{
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    /* 两端越界腿长都由配置边界推出，换车时不需要改这个测试。 */
    Chassis.leg[CHASSIS_LEFT].L0 = Chassis_Config.lqr.L0_min - 0.02f;
    Chassis.leg[CHASSIS_RIGHT].L0 = Chassis_Config.lqr.L0_max + 0.05f;

    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    Chassis_Control();

    assert(fabsf(Chassis.leg[CHASSIS_LEFT].K_L0_fit -
                 Chassis_Config.lqr.L0_min) < TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].K_L0_fit -
                 Chassis_Config.lqr.L0_max) < TEST_TOLERANCE);
    assert(Chassis.lqr.limit_flag == 1U);
    assert_zero_output();
}

static void test_wheel_leg_speed_estimate(void)
{
    float expected_speed_mps;
    uint32_t side;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();

    Chassis.imu.body_accel[Chassis_Config.imu.forward_accel_axis] = 2.0f;
    Chassis.imu.accel[Chassis_Config.imu.forward_accel_axis] = 100.0f;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].d_phi0 = 1.0f;
        Chassis.leg[side].d_L0 = 0.0f;
    }

    Chassis_Control();

    expected_speed_mps =
        Chassis_Config.wheel.R +
        Chassis.leg[CHASSIS_LEFT].L0;
    assert(fabsf(Chassis.body.side_speed[CHASSIS_LEFT] -
                 expected_speed_mps) < TEST_TOLERANCE);
    assert(fabsf(Chassis.body.side_speed[CHASSIS_RIGHT] -
                 expected_speed_mps) < TEST_TOLERANCE);
    assert(fabsf(Chassis.body.d_s_raw - expected_speed_mps) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.body.dd_s - 2.0f) < TEST_TOLERANCE);
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
    assert_control_request();
    assert_zero_output();

    Chassis.remote_online_flag = 1U;
    Chassis.remote_stop_flag = 1U;
    Chassis_State_Update();
    Chassis_Control();
    assert((Chassis.fault & CHASSIS_FAULT_REMOTE) != 0U);
    assert_control_request();
    assert_zero_output();

    Chassis.remote_stop_flag = 0U;
    Chassis_State_Update();
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
    Chassis_Control();
    assert(Chassis.fault == CHASSIS_FAULT_NONE);
}

static void test_remote_goal(void)
{
    float initial_yaw_target_rad;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.pitch = 0.04f;
    Chassis.imu.yaw_total = CHASSIS_PI + 0.04f;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    initial_yaw_target_rad = Chassis.lqr.target[CHASSIS_STATE_FAI];
    Chassis.goal.d_s = APP_RC_MAX_VEL;
    Chassis.goal.d_fai = APP_RC_MAX_YAW;
    /*
     * 目标腿长写死一个低于播种姿态0.25的值，不取APP_RC_LEG_*：
     * 那三档是随时会调的整定量，一旦某档等于0.25，斜坡就无事可做，
     * 本用例要验的"不再一拍到位"会变成空断言。
     */
    Chassis.goal.L0 = 0.15f;
    Chassis.body.s = 5.0f;

    Chassis_Control();

    /* 速度目标直接取遥控值，不走斜坡。 */
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_S] -
                 APP_RC_MAX_VEL) < TEST_TOLERANCE);
    /*
     * 腿长目标按L0_rate斜坡逼近goal.L0：进STANDING时已锁在当时的实际
     * 腿长0.25，第一拍只朝APP_RC_LEG_S走一个步长，不再一拍到位。
     */
    assert(Chassis.leg[CHASSIS_LEFT].target_L0 < 0.25f);
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - 0.25f) <=
           Chassis_Config.recovery.L0_rate * Chassis.dt + TEST_TOLERANCE);
    /* 有偏航输入时航向目标按角速度积分，角速度目标直接透传。 */
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                 (initial_yaw_target_rad +
                  APP_RC_MAX_YAW * Chassis.dt)) < TEST_TOLERANCE);
    assert(Chassis.lqr.target[CHASSIS_STATE_FAI] > CHASSIS_PI);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] -
                 APP_RC_MAX_YAW) < TEST_TOLERANCE);
    assert(Chassis.yaw_stick_flag == 1U);
    /* 有前进速度指令时位移状态清零，位移项不参与。 */
    assert(Chassis.body.s == 0.0f);
    assert_zero_output();

    /* 松杆瞬间锁存当前航向并保持，偏航角速度目标清零。 */
    Chassis.goal.d_s = 0.0f;
    Chassis.goal.d_fai = 0.0f;
    Chassis_Control();
    assert(Chassis.lqr.target[CHASSIS_STATE_D_S] == 0.0f);
    assert(Chassis.lqr.target[CHASSIS_STATE_D_FAI] == 0.0f);
    assert(Chassis.yaw_stick_flag == 0U);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                 Chassis.body.fai) < TEST_TOLERANCE);
    assert_zero_output();
}

static void test_top_projection(void)
{
    float yaw_anchor_rad = 0.40f;
    uint32_t index;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.yaw_total = yaw_anchor_rad;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_TOP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
    assert(fabsf(Chassis.top_fai - yaw_anchor_rad) <
           TEST_TOLERANCE);

    Chassis.goal.d_s = 0.25f;
    Chassis.goal.d_y = -0.10f;
    /* 固定转速自转：目标转速来自配置，不再由右摇杆给。 */
    Chassis.goal.d_fai = Chassis_Config.top.spin_d_fai;
    Chassis.imu.yaw_total = yaw_anchor_rad + CHASSIS_HALF_PI;
    Chassis_Control();

    assert(fabsf(Chassis.top_d_s + 0.10f) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_S] -
                 Chassis.top_d_s) < TEST_TOLERANCE);
    /* 角速度目标按top.d_fai_rate斜坡逼近，第一拍只走一个步长。 */
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] -
                 Chassis_Config.top.d_fai_rate * Chassis.dt) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] -
                 Chassis.imu.yaw_total) < TEST_TOLERANCE);
    assert(Chassis.lqr.scale[CHASSIS_STATE_S] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_FAI] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_S] == 1.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_FAI] == 1.0f);

    /* 持续运行后收敛到配置转速并停在目标上，不越过。 */
    for (index = 0U; index < 1000U; index++)
    {
        Chassis_Control();
        assert(Chassis.lqr.target[CHASSIS_STATE_D_FAI] <=
               Chassis.goal.d_fai + TEST_TOLERANCE);
    }
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] -
                 Chassis.goal.d_fai) < TEST_TOLERANCE);
    assert_zero_output();
}

/*
 * 退出小陀螺时角速度目标必须按斜坡收敛到摇杆值，不能阶跃。
 * 直接交回摇杆会让目标从spin_d_fai一拍跳到0，LQR随即给出大反向轮力矩硬刹。
 */
static void test_top_exit_ramp(void)
{
    float yaw_anchor_rad = 0.40f;
    float maximum_step;
    float previous_target;
    uint32_t ramp_ticks;
    uint32_t index;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.yaw_total = yaw_anchor_rad;
    set_symmetric_leg_pose(0.25f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_TOP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    /*
     * 走完整条斜坡要 spin_d_fai / (d_fai_rate * dt) 拍，按配置算出来再留
     * 余量。写死次数会在调 spin_d_fai 或 d_fai_rate 之后悄悄不够用。
     */
    ramp_ticks = (uint32_t)(Chassis_Config.top.spin_d_fai /
                            (Chassis_Config.top.d_fai_rate * Chassis.dt)) +
                 2U;

    /* 先让自转转速爬满。 */
    Chassis.goal.d_fai = Chassis_Config.top.spin_d_fai;
    for (index = 0U; index < ramp_ticks; index++)
    {
        Chassis_Control();
    }
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] -
                 Chassis_Config.top.spin_d_fai) < TEST_TOLERANCE);
    assert(Chassis.top_exit_flag == 1U);

    /* 拨回跟随、摇杆回中：角速度目标必须逐拍下降，且每拍不超过一个步长。 */
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis.goal.d_fai = 0.0f;
    Chassis_State_Update();
    maximum_step = Chassis_Config.top.d_fai_rate * Chassis.dt;
    for (index = 0U; index < ramp_ticks; index++)
    {
        previous_target = Chassis.lqr.target[CHASSIS_STATE_D_FAI];
        Chassis_Control();
        assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] -
                     previous_target) <= maximum_step + TEST_TOLERANCE);
        assert(Chassis.lqr.target[CHASSIS_STATE_D_FAI] >= -TEST_TOLERANCE);
    }
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI]) < TEST_TOLERANCE);
    /* 收敛完成后标志清零，控制权交回摇杆。 */
    assert(Chassis.top_exit_flag == 0U);

    /* 交回之后摇杆直通，不再经过斜坡。 */
    Chassis.goal.d_fai = 0.5f;
    Chassis_Control();
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] - 0.5f) <
           TEST_TOLERANCE);
    assert_zero_output();
}

/*
 * 从与approach_L0不同的腿长切入STEP时，腿长目标必须从当前实际腿长接管
 * 并按斜坡走，不允许阶跃。直接赋approach_L0会让kp=800的腿长PID一拍打满
 * outputLimit，实机表现是切到左上瞬间腿弹长、失衡倒地。
 */
static void test_step_enter_L0_ramp(void)
{
    float entry_L0 = 0.15f;
    float maximum_step;
    float previous_target;
    uint32_t index;

    assert(fabsf(entry_L0 - Chassis_Config.step.approach_L0) > 0.05f);

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(entry_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STEP);
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);

    /* 进入那一拍不许把目标直接摁到approach_L0。 */
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - entry_L0) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].target_L0 - entry_L0) <
           TEST_TOLERANCE);

    /* 之后每拍的目标增量都不得超过一个斜坡步长（台阶用自己的L0_rate）。 */
    maximum_step = Chassis_Config.step.L0_rate * Chassis.dt;
    for (index = 0U; index < 50U; index++)
    {
        previous_target = Chassis.leg[CHASSIS_LEFT].target_L0;
        Chassis_Step();
        assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - previous_target) <=
               maximum_step + TEST_TOLERANCE);
    }
    /* 斜坡朝approach_L0走，但50拍远不足以走完。 */
    assert(Chassis.leg[CHASSIS_LEFT].target_L0 > entry_L0);
    assert(Chassis.leg[CHASSIS_LEFT].target_L0 <
           Chassis_Config.step.approach_L0);
}

/*
 * 准备和接近阶段必须能用右摇杆转向，否则对不准台阶；磕上台阶之后
 * (RECOVER)航向锁死，摇杆不得再改变step_fai。
 * MOTION1/MOTION2 走关节位置串级，根本不到航向那段代码。
 */
static void test_step_yaw_control(void)
{
    float yaw_rate = 0.8f;
    float yaw_scale = Chassis_Config.imu.yaw_angle_scale;
    float entry_fai;
    float expected;
    float frozen_fai;

    Chassis_Init();
    set_online_feedback();
    Chassis.imu.yaw_total = 0.20f;
    /*
     * 腿长既不等于approach_L0也不等于goal.L0：前者让状态机留在PREPARE，
     * 后者保证后面手动切到RECOVER时不会当拍就达标跳回PREPARE
     * ——那一跳会做交接复位、把step_fai重锁到当前航向。
     */
    set_symmetric_leg_pose(0.20f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STEP);
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);
    entry_fai = Chassis.step_fai;

    /* 推杆：航向目标按角速度积分，角速度目标直通。 */
    Chassis.goal.d_fai = yaw_rate;
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);
    expected = entry_fai + yaw_rate * Chassis.dt;
    assert(fabsf(Chassis.step_fai - expected) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - expected) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_D_FAI] - yaw_rate) <
           TEST_TOLERANCE);
    assert(Chassis.yaw_stick_flag == 1U);

    /* 松杆：锁存到当前实际航向，角速度目标清零。 */
    Chassis.goal.d_fai = 0.0f;
    Chassis.imu.yaw_total = 0.55f;
    Chassis_Step();
    assert(fabsf(Chassis.step_fai - 0.55f * yaw_scale) < TEST_TOLERANCE);
    assert(Chassis.lqr.target[CHASSIS_STATE_D_FAI] == 0.0f);
    assert(Chassis.yaw_stick_flag == 0U);

    /* RECOVER：轮输出已清零，摇杆不得再改航向目标。 */
    Chassis.step_phase = CHASSIS_STEP_RECOVER;
    frozen_fai = Chassis.step_fai;
    Chassis.goal.d_fai = yaw_rate;
    Chassis.imu.yaw_total = 0.90f;
    Chassis_Step();
    assert(fabsf(Chassis.step_fai - frozen_fai) < TEST_TOLERANCE);
    assert(fabsf(Chassis.lqr.target[CHASSIS_STATE_FAI] - frozen_fai) <
           TEST_TOLERANCE);
    assert(Chassis.lqr.target[CHASSIS_STATE_D_FAI] == 0.0f);
}

static void test_step_phases(void)
{
    const Chassis_Step_Config_t *step = &Chassis_Config.step;
    int32_t maximum_wheel_request = 0;
    uint32_t iteration;
    uint32_t side;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STEP);
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);

    /* 去掉stable_time消抖后，容差满足当拍就切换，不需要再循环等待。 */
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_APPROACH);

    Chassis.goal.d_s = step->approach_d_s;
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI + 0.80f);
    for (side = 0U; side < APP_WHEEL_COUNT; side++)
    {
        Chassis.wheel_motor[side].current =
            (int16_t)(step->contact_T_fb * Chassis_Config.wheel.T_to_I * 1.5f);
    }
    for (iteration = 0U; iteration < 100U; iteration++)
    {
        Chassis_Step();
        if (Chassis.step_phase == CHASSIS_STEP_MOTION1)
        {
            break;
        }
    }
    assert(Chassis.step_contact_latch_flag[CHASSIS_LEFT] == 1U);
    assert(Chassis.step_contact_latch_flag[CHASSIS_RIGHT] == 1U);
    assert(Chassis.step_phase == CHASSIS_STEP_MOTION1);
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

    /* MOTION1：腿摆到swing_phi0且腿长到mid_L0才转MOTION2。 */
    set_symmetric_leg_pose(step->mid_L0,
                           CHASSIS_HALF_PI + step->swing_phi0);
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_MOTION2);
    assert_zero_output();

    /* MOTION2：腿收到retract_L0且转回竖直才转RECOVER。 */
    set_symmetric_leg_pose(step->retract_L0, CHASSIS_HALF_PI);
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_RECOVER);
    assert_zero_output();

    /*
     * RECOVER达标后不再进STANDING，而是循环回PREPARE准备下一级台阶：
     * state仍是CHASSIS_STEP，只有外部mode变化才会真正离开STEP。
     * 回PREPARE的同一拍要做完整交接复位。
     */
    Chassis.imu.yaw_total = 1.30f;
    set_symmetric_leg_pose(Chassis.goal.L0,
                           CHASSIS_HALF_PI + step->recover_theta);
    Chassis_Step();
    assert(Chassis.state == CHASSIS_STEP);
    assert(Chassis.step_phase == CHASSIS_STEP_PREPARE);
    /* 交接复位：航向重锁当前实际值，碰撞锁存清空，位移参考归零。 */
    assert(fabsf(Chassis.step_fai -
                 1.30f * Chassis_Config.imu.yaw_angle_scale) <
           TEST_TOLERANCE);
    assert(Chassis.step_contact_latch_flag[CHASSIS_LEFT] == 0U);
    assert(Chassis.step_contact_latch_flag[CHASSIS_RIGHT] == 0U);
    assert(Chassis.lqr.target[CHASSIS_STATE_S] == 0.0f);
    assert(Chassis.body.s == 0.0f);
    assert_zero_output();

    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);
}

/*
 * MOTION1/MOTION2 是位置型：目标腿角和目标腿长都必须按各自限速逐拍推进，
 * 不能一步阶跃到位，否则关节串级第一拍就满输出。
 */
static void test_step_motion_ramp(void)
{
    const Chassis_Step_Config_t *step = &Chassis_Config.step;
    float entry_phi0;
    float entry_L0;
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    Chassis.step_phase = CHASSIS_STEP_MOTION1;
    Chassis.leg[CHASSIS_LEFT].target_phi0 =
        Chassis.leg[CHASSIS_LEFT].phi0_total;
    Chassis.leg[CHASSIS_RIGHT].target_phi0 =
        Chassis.leg[CHASSIS_RIGHT].phi0_total;
    entry_phi0 = Chassis.leg[CHASSIS_LEFT].target_phi0;
    entry_L0 = Chassis.leg[CHASSIS_LEFT].target_L0;

    for (iteration = 0U; iteration < 50U; iteration++)
    {
        float previous_phi0 = Chassis.leg[CHASSIS_LEFT].target_phi0;
        float previous_L0 = Chassis.leg[CHASSIS_LEFT].target_L0;

        Chassis_Step();
        assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_phi0 - previous_phi0) <=
               (step->swing_phi0_rate * Chassis.dt) + TEST_TOLERANCE);
        assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - previous_L0) <=
               (step->climb_L0_rate * Chassis.dt) + TEST_TOLERANCE);
    }
    /* 后摆方向：目标腿杆角朝 phi0_offset + swing_phi0 增大。 */
    assert(Chassis.leg[CHASSIS_LEFT].target_phi0 > entry_phi0);
    assert(Chassis.leg[CHASSIS_LEFT].target_L0 < entry_L0);
    /* 50拍远不足以走完，仍在推进途中。 */
    assert(Chassis.leg[CHASSIS_LEFT].target_phi0 <
           Chassis_Config.phi0_offset + step->swing_phi0);
    assert(Chassis.step_phase == CHASSIS_STEP_MOTION1);
}

/*
 * MOTION1 的到位判据必须只看机构角，不能掺机体俯仰。
 * ZJU原文比的是含pitch的theta_l，展开等于 |腿跟踪误差 + theta_b|，
 * 爬台阶时前轮搭上台阶沿、机体俯仰本来就大，那样会永远退不出——
 * 他们有3秒模式超时兜底，本工程没有。
 */
static void test_step_motion1_pitch_independent(void)
{
    const Chassis_Step_Config_t *step = &Chassis_Config.step;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    Chassis.step_phase = CHASSIS_STEP_MOTION1;

    /* 机构角正好到位，但机体俯仰远超 swing_phi0_tol。 */
    Chassis.imu.pitch = step->swing_phi0_tol + 0.25f;
    set_symmetric_leg_pose(step->mid_L0,
                           CHASSIS_HALF_PI + step->swing_phi0);
    assert(fabsf(Chassis.imu.pitch) > step->swing_phi0_tol);

    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_MOTION2);
}

/*
 * 姿态判据必须能在轮力矩完全不满足时独立判出撞台阶。ZJU指出轮力矩这类
 * 信号与颠簸、急减速、踩弹丸难以区分，速度跟踪误差才是关键区分。
 */
static void test_step_contact_posture(void)
{
    const Chassis_Step_Config_t *step = &Chassis_Config.step;
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_APPROACH);

    /* 轮电流为零：旧的轮力矩路完全不成立。 */
    Chassis.wheel_motor[CHASSIS_LEFT].current = 0;
    Chassis.wheel_motor[CHASSIS_RIGHT].current = 0;
    /* 机体被顶得低头、腿被迫后摆，指令速度给满而实测几乎为零。 */
    Chassis.imu.pitch = step->contact_pitch + 0.05f;
    set_symmetric_leg_pose(step->approach_L0,
                           CHASSIS_HALF_PI + step->contact_theta_hard + 0.05f);
    Chassis.goal.d_s = step->approach_d_s;

    for (iteration = 0U; iteration < 200U; iteration++)
    {
        Chassis_Step();
        if (Chassis.step_phase == CHASSIS_STEP_MOTION1)
        {
            break;
        }
    }
    assert(Chassis.step_posture_flag == 1U);
    assert(Chassis.step_phase == CHASSIS_STEP_MOTION1);
}

/*
 * 新判据的风险是误判不是漏判：平地正常行进不能被当成撞台阶。
 */
static void test_step_contact_no_false_trigger(void)
{
    const Chassis_Step_Config_t *step = &Chassis_Config.step;
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_STEP;
    Chassis_State_Update();
    Chassis_Step();
    assert(Chassis.step_phase == CHASSIS_STEP_APPROACH);

    /* 俯仰和腿摆都很小，轮电流为零，摇杆不给速度。 */
    Chassis.wheel_motor[CHASSIS_LEFT].current = 0;
    Chassis.wheel_motor[CHASSIS_RIGHT].current = 0;
    Chassis.imu.pitch = 0.02f;
    Chassis.goal.d_s = 0.0f;
    set_symmetric_leg_pose(step->approach_L0, CHASSIS_HALF_PI + 0.02f);

    for (iteration = 0U; iteration < 200U; iteration++)
    {
        Chassis_Step();
        assert(Chassis.step_posture_flag == 0U);
    }
    assert(Chassis.step_phase == CHASSIS_STEP_APPROACH);
    assert(Chassis.step_contact_latch_flag[CHASSIS_LEFT] == 0U);
    assert(Chassis.step_contact_latch_flag[CHASSIS_RIGHT] == 0U);
}

/*
 * 转腿目标必须跟着倒地方向对称。旧代码把 theta_ref 写死在正侧，机体往负
 * 方向倒时腿即使已经摆到位也判不到位，会一直转到超时。
 */
static void test_recovery_direction_symmetry(void)
{
    uint32_t iteration;

    /* 负方向倒，腿已摆到同侧对应位置：应当判到位并交给板凳准备阶段。 */
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, 0.80f);
    set_fall_pose(-0.20f);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    for (iteration = 0U; iteration < 200U; iteration++)
    {
        Chassis_Recovery();
        if (Chassis.state != CHASSIS_FALLEN)
        {
            break;
        }
    }
    /* 参考角在自救第一拍锁存，符号必须落在机体倾倒的同一侧。 */
    assert(Chassis.recovery_theta_ref < 0.0f);
    assert(Chassis.state == CHASSIS_FALLING_TO_STAND);

    /* 镜像姿态必须同样成立，判据不能只在一个倒地方向有效。 */
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, CHASSIS_PI - 0.80f);
    set_fall_pose(0.20f);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    for (iteration = 0U; iteration < 200U; iteration++)
    {
        Chassis_Recovery();
        if (Chassis.state != CHASSIS_FALLEN)
        {
            break;
        }
    }
    /* 参考角在自救第一拍锁存，符号必须落在机体倾倒的同一侧。 */
    assert(Chassis.recovery_theta_ref > 0.0f);
    assert(Chassis.state == CHASSIS_FALLING_TO_STAND);
}

/*
 * 腿转不动时目标一直往同方向推只会顶到超时，连续卡住必须反向扫，
 * 并把目标重锁到当前实际角，否则解卡瞬间目标已经跑远、腿会甩。
 */
static void test_recovery_stuck_reverse(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    uint32_t stuck_tick = (uint32_t)(recovery->stuck_time / APP_CTRL_DT_S);
    float first_direction;
    float phi0_total_rad;
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, CHASSIS_HALF_PI);
    /* 倾角落在转腿窗口之外，腿角速度为零，正是卡死的样子。 */
    set_fall_pose(1.60f);
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);

    Chassis_Recovery();
    first_direction = Chassis.recovery_direction;
    assert(first_direction != 0.0f);
    phi0_total_rad = Chassis.leg[CHASSIS_LEFT].phi0_total;

    /*
     * stuck_time 是可调参数，填 <=0 或大于 fallen_timeout 都表示"关掉卡死反转"。
     * 关掉时本用例改为验证禁用路径：整个自救窗口里方向都不许翻。
     * 不要把它写死成"一定会反转"——那样调参就会挂在这里。
     */
    if ((recovery->stuck_time <= 0.0f) ||
        (recovery->stuck_time > recovery->fallen_timeout))
    {
        for (iteration = 0U;
             iteration < (uint32_t)(recovery->fallen_timeout / APP_CTRL_DT_S);
             iteration++)
        {
            if (Chassis.state != CHASSIS_FALLEN)
            {
                break;
            }
            Chassis_Recovery();
            assert(Chassis.recovery_direction == first_direction);
        }
        return;
    }

    /* 卡死计时未满之前方向不许变，目标被 rotate_lead_max 压住。 */
    for (iteration = 1U; iteration < stuck_tick; iteration++)
    {
        Chassis_Recovery();
        assert(Chassis.recovery_direction == first_direction);
    }
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_phi0 - phi0_total_rad) >
           recovery->rotate_lead_max - TEST_TOLERANCE);

    Chassis_Recovery();
    assert(Chassis.recovery_direction == -first_direction);
    assert(Chassis.recovery_stuck_time < recovery->stuck_time);
    /* 反转的同一拍把目标重锁回实际角，只留下反向的第一步。 */
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_phi0 - phi0_total_rad) <=
           (recovery->rotate_rate * Chassis.dt) + TEST_TOLERANCE);
}

/*
 * 侧躺时腿长腿角照样能到位，只查 pitch 会把车直接交给站立控制，
 * 而那个姿态已经在 LQR 线性化域外。roll 必须一起进到位判据。
 */
static void test_recovery_roll_gate(void)
{
    uint32_t iteration;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(Chassis_Config.recovery.bench_L0,
                           Chassis_Config.recovery.bench_phi0);
    set_fall_pose(0.0f);
    Chassis.imu.roll = Chassis_Config.recovery.ready_roll + 0.2f;
    Chassis.mode = CHASSIS_MODE_SELF_SAVE;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);
    Chassis_Recovery();
    assert(Chassis.state == CHASSIS_FALLING_TO_STAND);

    /* roll 超限时到位计时不许累加，远超 stable_time 也不能交接。 */
    for (iteration = 0U; iteration < 500U; iteration++)
    {
        Chassis_Recovery();
    }
    assert(Chassis.state == CHASSIS_FALLING_TO_STAND);
    assert(Chassis.stable_time == 0.0f);

    /* roll 收回门内之后才允许交接。 */
    Chassis.imu.roll = 0.0f;
    for (iteration = 0U; iteration < 500U; iteration++)
    {
        Chassis_Recovery();
        if (Chassis.state == CHASSIS_STANDING)
        {
            break;
        }
    }
    assert(Chassis.state == CHASSIS_STANDING);
}

/*
 * EKF pitch 是 asinf 出来的，车倒过 90 度以后会折返回小值。拨进 FOLLOW
 * 的姿态门若吃这个折返值，趴着的车会被判成"可以直接站"。
 */
static void test_follow_entry_fallen_pitch(void)
{
    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, CHASSIS_HALF_PI);
    /* 真实俯仰 2.60 rad(149度)，EKF 读数折返到 0.54 rad，落在 direct_pitch 之内。 */
    set_fall_pose(2.60f);
    assert(fabsf(Chassis.imu.pitch) < Chassis_Config.recovery.direct_pitch);
    assert(fabsf(Chassis.fall_pitch) > Chassis_Config.recovery.direct_pitch);

    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_FALLEN);
}

/*
 * 把底盘摆成"整车离地已成立"的状态：站立控制中，两腿支撑力估计压到离地
 * 门限以下并跑够消抖。返回时 all_off_flag 已置位。
 */
static void enter_all_off_ground(void)
{
    uint32_t iteration;
    uint32_t side;

    Chassis_Init();
    set_online_feedback();
    set_symmetric_leg_pose(0.20f, CHASSIS_HALF_PI);
    Chassis.mode = CHASSIS_MODE_FOLLOW;
    Chassis_State_Update();
    assert(Chassis.state == CHASSIS_STANDING);

    /*
     * 由目标F0反推关节反馈力矩，观测器再正解回同一个F0。直接塞裸力矩会被
     * 左右腿不同的 joint.scale 和雅可比映射搅乱，F0未必是负的。
     * F0取负即腿在主动回收，支撑力估计随之掉到离地门限以下。
     */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const Chassis_Leg_Config_t *leg_config = &Chassis_Config.leg[side];
        VMC_Torque_t torque;

        assert(VMC_Torque_Calc(leg_config, &Chassis.leg[side], -30.0f, 0.0f,
                               &torque) == 1U);
        Chassis.dm_motor[leg_config->joint[CHASSIS_JOINT_PHI1].motor_index]
            .torque_nm = torque.T1;
        Chassis.dm_motor[leg_config->joint[CHASSIS_JOINT_PHI4].motor_index]
            .torque_nm = torque.T4;
    }
    for (iteration = 0U; iteration < 200U; iteration++)
    {
        Chassis_Control();
        if (Chassis.ground.all_off_flag != 0U)
        {
            break;
        }
    }
    assert(Chassis.ground.all_off_flag == 1U);
}

/*
 * 整车离地成立时三项动作都要出现：悬空腿多出下压推力、悬空轮力矩清零、
 * LQR掩码只留腿摆通道。
 * 本二进制里 output.off_ground_act_flag 已由构建脚本 sed 成开；实际发布
 * 默认是关的，那一条由 test_chassis_observer 断言（它编的是未改的配置）。
 */
static void test_off_ground_actions(void)
{
    enter_all_off_ground();
    assert(Chassis.ground.fn_comp[CHASSIS_LEFT] > 0.0f);

    Chassis_Control();
    /* 悬空轮不驱动。 */
    assert(Chassis.output.T_wheel[CHASSIS_LEFT] == 0.0f);
    assert(Chassis.output.T_wheel[CHASSIS_RIGHT] == 0.0f);
    /* 掩码：位移、速度、航向、俯仰全屏蔽，只留腿摆角和角速度。 */
    assert(Chassis.lqr.scale[CHASSIS_STATE_S] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_S] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_FAI] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_THETA_B] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_THETA_B] == 0.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_THETA_L] == 1.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_THETA_L] == 1.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_THETA_R] == 1.0f);
    assert(Chassis.lqr.scale[CHASSIS_STATE_D_THETA_R] == 1.0f);
}

/*
 * 单腿离地不是腾空。过坎、压弹丸时单腿短暂卸载是常态，此时不许动控制。
 */
static void test_off_ground_action_needs_both_legs(void)
{
    uint32_t state;

    enter_all_off_ground();

    /* 右腿恢复支撑，整车离地不再成立。 */
    Chassis.ground.off_ground_flag[CHASSIS_RIGHT] = 0U;
    Chassis.ground.all_off_flag = 0U;
    Chassis_Control();
    assert(Chassis.ground.all_off_flag == 0U);
    for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
    {
        assert(Chassis.lqr.scale[state] == 1.0f);
    }
    assert(Chassis.ground.fn_comp[CHASSIS_LEFT] == 0.0f);
}

/*
 * 腿长接近拟合上限后不再叠加下压推力，否则悬空腿会一直顶到机构限位。
 * 用左右腿对照来测：同一拍里左腿超过保护线、右腿没超，两条腿的高度PID
 * (共模)、横滚PID(roll为0时输出为0)和重力前馈(同一个theta)都相同，
 * 于是 F0 之差就等于那份被挡掉的推力本身。单独改一条腿的腿长再前后比较
 * 是测不出来的——腿长一变，高度PID的误差也跟着变。
 */
static void test_off_ground_push_L0_cap(void)
{
    const float margin = Chassis_Config.observer.off_comp_L0_margin;
    float long_L0 = Chassis_Config.lqr.L0_max - (margin * 0.5f);
    float short_L0 = Chassis_Config.lqr.L0_max - (margin * 4.0f);
    float push_n;
    uint32_t side;

    enter_all_off_ground();
    push_n = Chassis.ground.fn_comp[CHASSIS_LEFT];
    assert(push_n > 0.0f);

    /* 左腿越过保护线，右腿留在线内。 */
    set_leg_pose(CHASSIS_LEFT, long_L0, CHASSIS_HALF_PI);
    set_leg_pose(CHASSIS_RIGHT, short_L0, CHASSIS_HALF_PI);
    Chassis_Leg_Update();
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const Chassis_Leg_Config_t *leg_config = &Chassis_Config.leg[side];
        VMC_Torque_t torque;

        assert(VMC_Torque_Calc(leg_config, &Chassis.leg[side], -30.0f, 0.0f,
                               &torque) == 1U);
        Chassis.dm_motor[leg_config->joint[CHASSIS_JOINT_PHI1].motor_index]
            .torque_nm = torque.T1;
        Chassis.dm_motor[leg_config->joint[CHASSIS_JOINT_PHI4].motor_index]
            .torque_nm = torque.T4;
    }
    Chassis_Control();
    assert(Chassis.ground.all_off_flag == 1U);
    assert(Chassis.leg[CHASSIS_LEFT].L0 >
           (Chassis_Config.lqr.L0_max - margin));
    assert(Chassis.leg[CHASSIS_RIGHT].L0 <
           (Chassis_Config.lqr.L0_max - margin));
    /* 只有右腿拿到推力，左腿被保护线挡掉。 */
    assert(fabsf((Chassis.leg[CHASSIS_RIGHT].F0 -
                  Chassis.leg[CHASSIS_LEFT].F0) -
                 Chassis.ground.fn_comp[CHASSIS_RIGHT]) < 1.0e-3f);
}

/*
 * 落地交接：起飞锁存腿长指令，空中被推大，落地必须还原回腾空前的值，
 * 否则等于永久改了车身高度指令。
 */
static void test_off_ground_land_handoff(void)
{
    float latched;

    enter_all_off_ground();
    Chassis_Control();
    latched = Chassis.off_ground_L0_latch[CHASSIS_LEFT];
    assert(latched > 0.0f);

    /* 空中腿长指令被推大。 */
    Chassis.leg[CHASSIS_LEFT].target_L0 = latched + 0.05f;
    Chassis.leg[CHASSIS_RIGHT].target_L0 = latched + 0.05f;

    /* 双腿触地，整车离地结束，落地边沿要把腿长指令还原。 */
    Chassis.ground.off_ground_flag[CHASSIS_LEFT] = 0U;
    Chassis.ground.off_ground_flag[CHASSIS_RIGHT] = 0U;
    Chassis.ground.all_off_flag = 0U;
    Chassis_Control();
    assert(fabsf(Chassis.leg[CHASSIS_LEFT].target_L0 - latched) <
           TEST_TOLERANCE);
    assert(fabsf(Chassis.leg[CHASSIS_RIGHT].target_L0 - latched) <
           TEST_TOLERANCE);
}

static void test_output_disabled(void)
{
    assert(APP_CHASSIS_OUTPUT_ENABLE == 0U);
    assert(Chassis_Config.output.joint_flag == 0U);
    assert(Chassis_Config.output.wheel_flag == 0U);
}

static void test_joint_mapping(void)
{
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .motor_index == 1U);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI4]
               .motor_index == 0U);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI1]
               .motor_index == 3U);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI4]
               .motor_index == 2U);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .scale == 1.0f);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI4]
               .scale == 1.0f);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI1]
               .scale == -1.0f);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI4]
               .scale == -1.0f);
    /*
     * ratio 是每台车的同步带传动比（小轮腿0.75、大轮腿直连1.0），不是不变量。
     * 这里只校验四个关节取值一致且为正，可以抓住只改了其中一个的情况。
     */
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .ratio > 0.0f);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI4]
               .ratio ==
           Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .ratio);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI1]
               .ratio ==
           Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .ratio);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI4]
               .ratio ==
           Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .ratio);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI1]
               .angle_offset_rad == CHASSIS_HALF_PI);
    assert(Chassis_Config.leg[CHASSIS_LEFT]
               .joint[CHASSIS_JOINT_PHI4]
               .angle_offset_rad == CHASSIS_HALF_PI);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI1]
               .angle_offset_rad == CHASSIS_HALF_PI);
    assert(Chassis_Config.leg[CHASSIS_RIGHT]
               .joint[CHASSIS_JOINT_PHI4]
               .angle_offset_rad == CHASSIS_HALF_PI);
    assert(Chassis_Config.phi0_offset == CHASSIS_HALF_PI);
    assert(Chassis_Config.recovery.bench_phi0 == CHASSIS_HALF_PI);
    /*
     * 四道腿杆角门都是实机可调参数，不断言具体数值，只断言它们之间
     * 必须成立的关系（见下）。写死数值会让每次调参都挂在这里。
     */
    /*
     * 入口门必须落在站立保护门之内。入口门更松时，腿角处在两道门之间
     * 会先进STANDING、又在同一次State_Select里被姿态保护打回ZERO_FORCE。
     */
    assert(Chassis_Config.recovery.phi0_min >=
           Chassis_Config.recovery.stand_phi0_min);
    assert(Chassis_Config.recovery.phi0_max <=
           Chassis_Config.recovery.stand_phi0_max);
}

int main(void)
{
    test_output_disabled();
    test_joint_mapping();
    test_off_ground_actions();
    test_off_ground_action_needs_both_legs();
    test_off_ground_push_L0_cap();
    test_off_ground_land_handoff();
    test_bench_control();
    test_recovery_handoff();
    test_recovery_direction_symmetry();
    test_recovery_stuck_reverse();
    test_recovery_roll_gate();
    test_follow_entry_fallen_pitch();
    test_fallen_timeout();
    test_prepare_timeout();
    test_invalid_leg_feedback();
    test_invalid_leg_recovery();
    test_recovery_posture();
    test_wheel_offline_gate();
    test_fault_calculation();
    test_lqr_realtime_leg_length();
    test_vertical_theta_balance();
    test_lqr_leg_length_limit();
    test_wheel_leg_speed_estimate();
    test_continuous_angle();
    test_remote_fault();
    test_remote_goal();
    test_top_projection();
    test_top_exit_ramp();
    test_step_enter_L0_ramp();
    test_step_yaw_control();
    test_step_phases();
    test_step_motion_ramp();
    test_step_motion1_pitch_independent();
    test_step_contact_posture();
    test_step_contact_no_false_trigger();
    return 0;
}
