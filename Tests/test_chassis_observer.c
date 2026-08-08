#include "chassis_control.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TOLERANCE 2.0e-5f

static Chassis_Leg_Config_t make_leg_config(uint8_t motor_offset)
{
    Chassis_Leg_Config_t config = {
        .geometry = {
            .l1 = 0.215f,
            .l2 = 0.258f,
            .l3 = 0.258f,
            .l4 = 0.215f,
            .l5 = 0.0f,
        },
        .joint = {
            [CHASSIS_JOINT_PHI1] = {
                .motor_index = motor_offset,
                .angle_offset_rad = CHASSIS_PI,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
            [CHASSIS_JOINT_PHI4] = {
                .motor_index = motor_offset + 1U,
                .angle_offset_rad = 0.0f,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
        },
    };

    return config;
}

static Chassis_Observer_Config_t make_observer_config(void)
{
    Chassis_Observer_Config_t config = {
        .slip_gate_yaw = 0.90f,
        .slip_gate_v = 0.60f,
        .slip_v_enter = 0.20f,
        .slip_yaw_enter = 0.80f,
        .slip_dv_enter = 0.03f,
        .slip_enter_s = 0.05f,
        .off_force_ratio = 0.20f,
        .land_force_ratio = 0.35f,
        .off_hold_s = 0.03f,
        .land_hold_s = 0.05f,
        .off_F_comp_ratio = 0.28f,
        .turn_v_diff = 0.20f,
        .turn_force_limit_ratio = 0.50f,
        .stuck_T_ratio = 0.75f,
        .stuck_theta_enter = 0.50f,
        .stuck_theta_exit = 0.15f,
        .stuck_time = 0.05f,
        .stuck_F0_coef_ratio = 0.38f,
        .stuck_F0_max_ratio = 0.19f,
        .stuck_L0_coef = 0.06f,
        .stuck_L0_max = 0.04f,
    };

    return config;
}

static void set_feedback_force(const Chassis_Config_t *config,
                             Chassis_t *chassis,
                             float F0)
{
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const Chassis_Leg_Config_t *leg_config = &config->leg[side];
        VMC_Torque_t torque;

        assert(VMC_Torque_Calc(leg_config,
                               &chassis->leg[side],
                               F0,
                               0.0f,
                               &torque) == 1U);
        chassis->dm_motor[leg_config->joint[CHASSIS_JOINT_PHI1].motor_index]
            .torque_nm = torque.T1;
        chassis->dm_motor[leg_config->joint[CHASSIS_JOINT_PHI4].motor_index]
            .torque_nm = torque.T4;
    }
}

/* 与 Chassis_Control() 中相同的调用顺序：四套观测各自先Update再Calc。 */
static void run_observers(const Chassis_Config_t *config, Chassis_t *chassis)
{
    Chassis_Slip_Update(config, chassis);
    Chassis_Slip_Calc(config, chassis);
    Chassis_Ground_Update(config, chassis);
    Chassis_Ground_Calc(config, chassis);
    Chassis_Turn_Update(config, chassis);
    Chassis_Turn_Calc(config, chassis);
    Chassis_Stuck_Update(config, chassis);
    Chassis_Stuck_Calc(config, chassis);
}

int main(void)
{
    Chassis_Config_t config = {0};
    Chassis_t chassis;
    uint32_t iteration;

    memset(&chassis, 0, sizeof(chassis));
    config.observer = make_observer_config();
    /* 观测器按传入配置取IMU轴号，测试必须显式给出与整车一致的轴映射。 */
    config.imu.forward_accel_axis = 0U;
    config.imu.lateral_accel_axis = 1U;
    config.imu.vertical_accel_axis = 2U;
    config.wheel.R = 0.10f;
    config.wheel.half_track = 0.1965f;
    config.wheel.T_limit = 0.16f;
    /* 力类阈值以单腿静载为基准，模型参数必须由测试显式给出。 */
    config.model.gravity = 9.81f;
    config.model.body_mass = 10.0f;
    config.model.leg_mass = 0.5f;
    config.model.wheel_mass = 1.0f;
    config.model.cg_to_hip = 0.04f;
    config.leg[CHASSIS_LEFT] = make_leg_config(0U);
    config.leg[CHASSIS_RIGHT] = make_leg_config(2U);
    chassis.dt = 0.01f;

    VMC_State_Calc(&config.leg[CHASSIS_LEFT],
                   0.0f,
                   0.0f,
                   0.0f,
                   0.0f,
                   &chassis.leg[CHASSIS_LEFT]);
    assert(chassis.leg[CHASSIS_LEFT].valid_flag == 1U);
    chassis.leg[CHASSIS_RIGHT] = chassis.leg[CHASSIS_LEFT];
    Chassis_Slip_Init(&chassis.slip);
    Chassis_Ground_Init(&chassis.ground);
    Chassis_Turn_Init(&chassis.turn);
    Chassis_Stuck_Init(&chassis.stuck);

    chassis.body.side_speed[CHASSIS_LEFT] = 1.0f;
    chassis.body.side_speed[CHASSIS_RIGHT] = 1.0f;
    chassis.imu.body_accel[0] = 20.0f;
    set_feedback_force(&config, &chassis, -80.0f);
    run_observers(&config, &chassis);
    assert(chassis.slip.init_flag == 1U);
    assert(chassis.ground.init_flag == 1U);
    assert(chassis.slip.dv_res[CHASSIS_LEFT] == 0.0f);
    assert(chassis.ground.valid_flag[CHASSIS_LEFT] == 1U);
    assert(fabsf(chassis.ground.force[CHASSIS_LEFT].F0 + 80.0f) <
           TEST_TOLERANCE);

    /* 左轮加速打滑：轮速残差和偏航残差同时超阈值，闸门成立后判定打滑。 */
    chassis.imu.body_accel[0] = 0.0f;
    chassis.body.dd_s = 0.0f;
    chassis.body.side_speed[CHASSIS_LEFT] = 2.0f;
    chassis.body.wheel_speed[CHASSIS_LEFT] = 20.0f;
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(chassis.slip.gate_flag == 1U);
    assert(chassis.slip.slip_flag[CHASSIS_LEFT] == 1U);
    /* 打滑时不给转弯半径，避免用失真轮速反推。 */
    assert(chassis.turn.R_turn == 0.0f);

    /* 轮速回落到起始锁存值以下即退出打滑。 */
    chassis.body.side_speed[CHASSIS_LEFT] = 0.0f;
    chassis.body.side_speed[CHASSIS_RIGHT] = 0.0f;
    chassis.body.wheel_speed[CHASSIS_LEFT] = 0.0f;
    run_observers(&config, &chassis);
    assert(chassis.slip.slip_flag[CHASSIS_LEFT] == 0U);

    /* 小陀螺靠左右轮速差旋转，必须屏蔽打滑判定。 */
    chassis.mode = CHASSIS_MODE_TOP;
    chassis.body.side_speed[CHASSIS_LEFT] = 2.0f;
    chassis.body.wheel_speed[CHASSIS_LEFT] = 20.0f;
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(chassis.slip.gate_flag == 0U);
    assert(chassis.slip.slip_flag[CHASSIS_LEFT] == 0U);
    chassis.mode = CHASSIS_MODE_FOLLOW;

    /* 未打滑且左右轮速差够大时给出转弯半径和离心修正。 */
    chassis.body.side_speed[CHASSIS_LEFT] = 0.2f;
    chassis.body.side_speed[CHASSIS_RIGHT] = 0.2f;
    chassis.body.wheel_speed[CHASSIS_LEFT] = 8.0f;
    chassis.body.wheel_speed[CHASSIS_RIGHT] = 4.0f;
    chassis.body.d_fai = 1.0f;
    chassis.body.dd_s = 0.5f;
    run_observers(&config, &chassis);
    assert(chassis.slip.slip_flag[CHASSIS_LEFT] == 0U);
    assert(chassis.turn.R_turn > 0.0f);
    assert(fabsf(chassis.turn.dd_s_turn -
                 chassis.body.d_fai * chassis.body.d_fai *
                     chassis.turn.R_turn) < TEST_TOLERANCE);
    assert(fabsf(chassis.turn.dd_s_fix -
                 (chassis.body.dd_s + chassis.turn.dd_s_turn)) <
           TEST_TOLERANCE);

    /* 左右轮速一致时不反推转弯半径。 */
    chassis.body.wheel_speed[CHASSIS_RIGHT] = 8.0f;
    run_observers(&config, &chassis);
    assert(chassis.turn.R_turn == 0.0f);
    assert(chassis.turn.dd_s_turn == 0.0f);

    /* 支撑力掉到离地比例以下，双腿离地并给出下压补偿建议值。 */
    set_feedback_force(&config, &chassis, 0.0f);
    for (iteration = 0U; iteration < 4U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(chassis.ground.off_ground_flag[CHASSIS_LEFT] == 1U);
    assert(chassis.ground.off_ground_flag[CHASSIS_RIGHT] == 1U);
    assert(chassis.ground.all_off_flag == 1U);
    assert(fabsf(chassis.ground.fn_comp -
                 chassis.ground.Fn_static * config.observer.off_F_comp_ratio) <
           TEST_TOLERANCE);

    /* 倒地和台阶动作期间腿本来就会脱离地面，整车离地结论不成立。 */
    chassis.state = CHASSIS_STEP;
    run_observers(&config, &chassis);
    assert(chassis.ground.off_ground_flag[CHASSIS_LEFT] == 1U);
    assert(chassis.ground.all_off_flag == 0U);
    assert(chassis.ground.fn_comp == 0.0f);
    chassis.state = CHASSIS_FALLEN;
    run_observers(&config, &chassis);
    assert(chassis.ground.all_off_flag == 0U);
    chassis.state = CHASSIS_STANDING;

    set_feedback_force(&config, &chassis, -80.0f);
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(chassis.ground.all_off_flag == 0U);

    chassis.body.d_s = 2.0f;
    chassis.body.d_fai = 1.0f;
    chassis.imu.body_accel[1] = 2.0f;
    run_observers(&config, &chassis);
    assert(chassis.turn.dF_imu > 0.0f);
    assert(chassis.turn.dF_kin > 0.0f);
    assert(fabsf(chassis.turn.dF_imu_lim) <=
           chassis.ground.Fn_static *
               config.observer.turn_force_limit_ratio +
               TEST_TOLERANCE);

    /* 卡腿：轮力矩和腿摆角同时超阈值并持续，补偿量随时间增长并封顶。 */
    assert(chassis.stuck.stuck_flag[CHASSIS_LEFT] == 0U);
    chassis.output.T_wheel[CHASSIS_LEFT] = 0.20f;
    chassis.output.T_wheel[CHASSIS_RIGHT] = 0.20f;
    chassis.leg[CHASSIS_LEFT].theta = 0.80f;
    chassis.leg[CHASSIS_RIGHT].theta = 0.80f;
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(chassis.stuck.stuck_flag[CHASSIS_LEFT] == 1U);
    assert(chassis.stuck.comp_F0[CHASSIS_LEFT] > 0.0f);
    assert(chassis.stuck.comp_L0[CHASSIS_LEFT] > 0.0f);
    for (iteration = 0U; iteration < 200U; iteration++)
    {
        run_observers(&config, &chassis);
    }
    assert(fabsf(chassis.stuck.comp_F0[CHASSIS_LEFT] -
                 config.observer.stuck_F0_max_ratio *
                     chassis.ground.Fn_static) < TEST_TOLERANCE);
    assert(fabsf(chassis.stuck.comp_L0[CHASSIS_LEFT] -
                 config.observer.stuck_L0_max) < TEST_TOLERANCE);

    /* 腿摆角收回到退出阈值以下即清空卡腿计时和补偿量。 */
    chassis.leg[CHASSIS_LEFT].theta = 0.10f;
    chassis.leg[CHASSIS_RIGHT].theta = 0.10f;
    run_observers(&config, &chassis);
    assert(chassis.stuck.stuck_flag[CHASSIS_LEFT] == 0U);
    assert(chassis.stuck.comp_F0[CHASSIS_LEFT] == 0.0f);
    assert(chassis.stuck.comp_L0[CHASSIS_LEFT] == 0.0f);
    return 0;
}
