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
            [CHASSIS_JOINT_FRONT] = {
                .motor_index = motor_offset,
                .angle_offset_rad = CHASSIS_PI,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
            [CHASSIS_JOINT_BACK] = {
                .motor_index = motor_offset + 1U,
                .angle_offset_rad = 0.0f,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
        },
    };

    return config;
}

static Chassis_Observer_Config_t make_observer_config(void)
{
    Chassis_Observer_Config_t config = {
        .gravity_mps2 = 9.81f,
        .body_mass_kg = 10.0f,
        .leg_mass_kg = 0.5f,
        .wheel_mass_kg = 1.0f,
        .body_cg_to_hip_m = 0.04f,
        .slip_speed_enter_mps = 0.20f,
        .slip_speed_exit_mps = 0.10f,
        .slip_yaw_enter_radps = 0.80f,
        .slip_yaw_exit_radps = 0.40f,
        .slip_delta_enter_mps = 0.03f,
        .slip_delta_exit_mps = 0.015f,
        .slip_enter_s = 0.05f,
        .slip_exit_s = 0.20f,
        .off_force_ratio = 0.20f,
        .land_force_ratio = 0.35f,
        .off_hold_s = 0.03f,
        .land_hold_s = 0.05f,
        .turn_force_limit_ratio = 0.50f,
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
        chassis->dm_motor[leg_config->joint[CHASSIS_JOINT_FRONT].motor_index]
            .torque_nm = torque.T_A;
        chassis->dm_motor[leg_config->joint[CHASSIS_JOINT_BACK].motor_index]
            .torque_nm = torque.T_E;
    }
}

int main(void)
{
    Chassis_Config_t config = {0};
    Chassis_t chassis;
    uint32_t iteration;

    memset(&chassis, 0, sizeof(chassis));
    config.observer = make_observer_config();
    config.wheel.R = 0.10f;
    config.wheel.half_track = 0.1965f;
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
    Chassis_Observer_Init(&chassis.observer);

    chassis.body.side_speed[CHASSIS_LEFT] = 1.0f;
    chassis.body.side_speed[CHASSIS_RIGHT] = 1.0f;
    chassis.imu.body_accel[0] = 20.0f;
    set_feedback_force(&config, &chassis, -80.0f);
    Chassis_Observer_Update(&config, &chassis);
    assert(chassis.observer.init_flag == 1U);
    assert(chassis.observer.delta_residual_mps[CHASSIS_LEFT] == 0.0f);
    assert(chassis.observer.force_valid_flag[CHASSIS_LEFT] == 1U);
    assert(fabsf(chassis.observer.feedback_force[CHASSIS_LEFT].F0 +
                 80.0f) < TEST_TOLERANCE);

    chassis.imu.body_accel[0] = 0.0f;
    chassis.body.side_speed[CHASSIS_LEFT] = 2.0f;
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        Chassis_Observer_Update(&config, &chassis);
    }
    assert(chassis.observer.slip_flag[CHASSIS_LEFT] == 1U);

    chassis.body.side_speed[CHASSIS_LEFT] = 0.0f;
    chassis.body.side_speed[CHASSIS_RIGHT] = 0.0f;
    for (iteration = 0U; iteration < 21U; iteration++)
    {
        Chassis_Observer_Update(&config, &chassis);
    }
    assert(chassis.observer.slip_flag[CHASSIS_LEFT] == 0U);

    set_feedback_force(&config, &chassis, 0.0f);
    for (iteration = 0U; iteration < 4U; iteration++)
    {
        Chassis_Observer_Update(&config, &chassis);
    }
    assert(chassis.observer.off_ground_flag[CHASSIS_LEFT] == 1U);
    assert(chassis.observer.off_ground_flag[CHASSIS_RIGHT] == 1U);
    assert(chassis.observer.all_off_flag == 1U);

    set_feedback_force(&config, &chassis, -80.0f);
    for (iteration = 0U; iteration < 6U; iteration++)
    {
        Chassis_Observer_Update(&config, &chassis);
    }
    assert(chassis.observer.all_off_flag == 0U);

    chassis.body.d_s = 2.0f;
    chassis.body.d_fai = 1.0f;
    chassis.imu.body_accel[1] = 2.0f;
    Chassis_Observer_Update(&config, &chassis);
    assert(chassis.observer.turn_support_imu_raw_n > 0.0f);
    assert(chassis.observer.turn_support_kin_raw_n > 0.0f);
    assert(fabsf(chassis.observer.turn_support_imu_limited_n) <=
           chassis.observer.nominal_static_load_n *
               config.observer.turn_force_limit_ratio +
               TEST_TOLERANCE);
    return 0;
}
