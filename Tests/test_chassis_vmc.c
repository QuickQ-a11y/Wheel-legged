#include "chassis_vmc.h"

#include <assert.h>
#include <math.h>

#define TEST_TOLERANCE 2.0e-5f
#define TEST_DERIVATIVE_TOLERANCE 2.0e-3f

static float motor_position(const Chassis_Leg_Config_t *config,
                            Chassis_Joint_t joint,
                            float phi)
{
    return (phi - config->joint[joint].angle_offset_rad) /
           (config->joint[joint].scale *
            config->joint[joint].ratio);
}

static void calculate_target_pose(const Chassis_Leg_Config_t *config,
                                  float target_L0,
                                  float target_phi0,
                                  Chassis_Leg_t *leg)
{
    Chassis_Leg_t current_leg = {
        .phi1 = 2.7f,
        .phi4 = 0.4f,
    };
    VMC_Joint_Target_t target;
    float phi1_motor_position;
    float phi4_motor_position;

    assert(VMC_Inverse_Calc(config,
                            &current_leg,
                            target_L0,
                            target_phi0,
                            &target) == 1U);
    phi1_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI1, target.phi1);
    phi4_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI4, target.phi4);
    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   0.0f,
                   0.0f,
                   leg);
    assert(leg->valid_flag == 1U);
}

static void test_geometry_convention(const Chassis_Leg_Config_t *base_config)
{
    static const float lengths[] = {0.18f, 0.25f, 0.32f};
    static const float angles[] = {1.25f, CHASSIS_HALF_PI, 1.90f};
    Chassis_Leg_Config_t config = *base_config;
    Chassis_Leg_t leg;
    uint32_t length_index;
    uint32_t angle_index;

    calculate_target_pose(&config, 0.25f, CHASSIS_HALF_PI, &leg);
    assert(leg.phi1 > leg.phi4);
    assert(fabsf(leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);

    config.geometry.l5 = 0.06f;
    for (length_index = 0U;
         length_index < sizeof(lengths) / sizeof(lengths[0]);
         length_index++)
    {
        for (angle_index = 0U;
             angle_index < sizeof(angles) / sizeof(angles[0]);
             angle_index++)
        {
            calculate_target_pose(&config,
                                  lengths[length_index],
                                  angles[angle_index],
                                  &leg);
            assert(leg.phi1 > leg.phi4);
            assert(fabsf(leg.L0 - lengths[length_index]) < TEST_TOLERANCE);
            assert(fabsf(leg.phi0 - angles[angle_index]) < TEST_TOLERANCE);
        }
    }
}

static void test_jacobian_and_virtual_work(
    const Chassis_Leg_Config_t *config)
{
    const float phi1_motor_position = 0.5f;
    const float phi4_motor_position = -0.8f;
    const float phi1_motor_speed = 0.7f;
    const float phi4_motor_speed = -0.4f;
    const float dt = 1.0e-3f;
    const float F0 = -30.0f;
    const float Tp = 1.2f;
    Chassis_Leg_t leg;
    Chassis_Leg_t previous_leg;
    Chassis_Leg_t next_leg;
    VMC_Torque_t torque;
    float numeric_d_L0;
    float numeric_d_phi0;
    float motor_power;
    float virtual_power;

    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &leg);
    VMC_State_Calc(config,
                   phi1_motor_position - phi1_motor_speed * dt,
                   phi4_motor_position - phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &previous_leg);
    VMC_State_Calc(config,
                   phi1_motor_position + phi1_motor_speed * dt,
                   phi4_motor_position + phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &next_leg);
    assert(leg.valid_flag == 1U);
    assert(previous_leg.valid_flag == 1U);
    assert(next_leg.valid_flag == 1U);

    numeric_d_L0 = (next_leg.L0 - previous_leg.L0) / (2.0f * dt);
    numeric_d_phi0 =
        (next_leg.phi0 - previous_leg.phi0) / (2.0f * dt);
    assert(fabsf(numeric_d_L0 - leg.d_L0) <
           TEST_DERIVATIVE_TOLERANCE);
    assert(fabsf(numeric_d_phi0 - leg.d_phi0) <
           TEST_DERIVATIVE_TOLERANCE);

    assert(VMC_Torque_Calc(config, &leg, F0, Tp, &torque) == 1U);
    motor_power = torque.T1 * phi1_motor_speed +
                  torque.T4 * phi4_motor_speed;
    virtual_power = F0 * leg.d_L0 + Tp * leg.d_phi0;
    assert(fabsf(motor_power - virtual_power) < TEST_TOLERANCE);
}

static void test_downward_phi0_branch(
    const Chassis_Leg_Config_t *config)
{
    const float target_L0 = 0.25f;
    const float target_phi0 = CHASSIS_HALF_PI;
    const float phi1_motor_speed = 0.20f;
    const float phi4_motor_speed = -0.15f;
    const float dt = 1.0e-3f;
    const float F0 = -30.0f;
    const float Tp = 1.2f;
    Chassis_Leg_t current_leg = {
        .phi1 = CHASSIS_PI,
        .phi4 = 0.0f,
    };
    VMC_Joint_Target_t target;
    Chassis_Leg_t leg;
    Chassis_Leg_t previous_leg;
    Chassis_Leg_t next_leg;
    VMC_Torque_t torque;
    float phi1_motor_position;
    float phi4_motor_position;
    float numeric_d_L0;
    float numeric_d_phi0;
    float motor_power;
    float virtual_power;

    assert(VMC_Inverse_Calc(config,
                            &current_leg,
                            target_L0,
                            target_phi0,
                            &target) == 1U);
    phi1_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI1, target.phi1);
    phi4_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI4, target.phi4);

    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &leg);
    VMC_State_Calc(config,
                   phi1_motor_position - phi1_motor_speed * dt,
                   phi4_motor_position - phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &previous_leg);
    VMC_State_Calc(config,
                   phi1_motor_position + phi1_motor_speed * dt,
                   phi4_motor_position + phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &next_leg);

    assert(leg.valid_flag == 1U);
    assert(previous_leg.valid_flag == 1U);
    assert(next_leg.valid_flag == 1U);
    assert(leg.phi1 > leg.phi4);
    assert(fabsf(leg.L0 - target_L0) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0 - target_phi0) < TEST_TOLERANCE);
    numeric_d_L0 = (next_leg.L0 - previous_leg.L0) / (2.0f * dt);
    numeric_d_phi0 =
        (next_leg.phi0 - previous_leg.phi0) / (2.0f * dt);
    assert(fabsf(numeric_d_L0 - leg.d_L0) <
           TEST_DERIVATIVE_TOLERANCE);
    assert(fabsf(numeric_d_phi0 - leg.d_phi0) <
           TEST_DERIVATIVE_TOLERANCE);

    assert(VMC_Torque_Calc(config, &leg, F0, Tp, &torque) == 1U);
    motor_power = torque.T1 * phi1_motor_speed +
                  torque.T4 * phi4_motor_speed;
    virtual_power = F0 * leg.d_L0 + Tp * leg.d_phi0;
    assert(fabsf(motor_power - virtual_power) < TEST_TOLERANCE);
}

int main(void)
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
                .angle_offset_rad = CHASSIS_PI,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
            [CHASSIS_JOINT_PHI4] = {
                .angle_offset_rad = 0.0f,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
        },
    };
    Chassis_Leg_t leg;
    Chassis_Leg_t inverse_leg;
    VMC_Torque_t torque;
    VMC_Force_t force;
    VMC_Joint_Target_t target;
    Chassis_Leg_Config_t boundary_config;
    Chassis_Leg_Config_t ratio_config;
    Chassis_Leg_t boundary_leg;
    float phi1_motor_position_rad;
    float phi4_motor_position_rad;

    test_geometry_convention(&config);
    ratio_config = config;
    ratio_config.joint[CHASSIS_JOINT_PHI1].ratio = 0.75f;
    ratio_config.joint[CHASSIS_JOINT_PHI4].ratio = 0.75f;
    test_jacobian_and_virtual_work(&ratio_config);
    test_downward_phi0_branch(&ratio_config);

    VMC_State_Calc(&config, 0.5f, -0.8f, 0.7f, -0.4f, &leg);

    assert(leg.valid_flag == 1U);
    assert(fabsf(leg.phi1 - 2.64159265f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi2 - 0.99543859f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi3 - 2.44615406f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi4 - 0.8f) < TEST_TOLERANCE);
    assert(fabsf(leg.L0 - 0.32316671f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0 - 1.72079633f) < TEST_TOLERANCE);
    assert(fabsf(leg.d_L0 - 0.15758435f) < TEST_TOLERANCE);
    assert(fabsf(leg.d_phi0 + 0.15f) < TEST_TOLERANCE);

    assert(VMC_Torque_Calc(&config, &leg, -30.0f, 1.2f, &torque) == 1U);
    assert(fabsf(torque.T1 + 4.89775487f) < TEST_TOLERANCE);
    assert(fabsf(torque.T4 - 3.69775487f) < TEST_TOLERANCE);
    assert(VMC_Force_Calc(&config,
                                &leg,
                                torque.T1,
                                torque.T4,
                                &force) == 1U);
    assert(fabsf(force.F0 + 30.0f) < TEST_TOLERANCE);
    assert(fabsf(force.Tp - 1.2f) < TEST_TOLERANCE);

    assert(VMC_Inverse_Calc(&config,
                               &leg,
                               0.15f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    phi1_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI1].scale *
         config.joint[CHASSIS_JOINT_PHI1].ratio);
    phi4_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI4].scale *
         config.joint[CHASSIS_JOINT_PHI4].ratio);
    VMC_State_Calc(&config,
                   phi1_motor_position_rad,
                   phi4_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.L0 - 0.15f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);

    /* 0.05 m人为门槛已删除，真实二连杆最小可达距离约为0.043 m。 */
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.045f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.042f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);

    config.geometry.l5 = 0.06f;
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.35f,
                               1.40f,
                               &target) == 1U);
    phi1_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI1].scale *
         config.joint[CHASSIS_JOINT_PHI1].ratio);
    phi4_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI4].scale *
         config.joint[CHASSIS_JOINT_PHI4].ratio);
    VMC_State_Calc(&config,
                   phi1_motor_position_rad,
                   phi4_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.L0 - 0.35f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0 - 1.40f) < TEST_TOLERANCE);

    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.60f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);
    assert(target.phi1 == 0.0f);
    assert(target.phi4 == 0.0f);
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               NAN,
                               CHASSIS_HALF_PI,
                               &target) == 0U);

    /* 相切两圆的位置解存在，但从动杆共线时速度雅可比不可逆。 */
    boundary_config = config;
    boundary_config.geometry.l1 = 0.1f;
    boundary_config.geometry.l2 = 0.1f;
    boundary_config.geometry.l3 = 0.1f;
    boundary_config.geometry.l4 = 0.1f;
    boundary_config.geometry.l5 = 0.2f;
    boundary_config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI1].scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI4].scale = 1.0f;
    VMC_State_Calc(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(fabsf(boundary_leg.L0 - 0.1f) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.d_L0 == 0.0f);
    assert(boundary_leg.d_phi0 == 0.0f);
    assert(VMC_Torque_Calc(&boundary_config,
                          &boundary_leg,
                          -30.0f,
                          1.2f,
                          &torque) == 0U);
    assert(VMC_Force_Calc(&boundary_config,
                                &boundary_leg,
                                1.0f,
                                1.0f,
                                &force) == 0U);
    assert(torque.T1 == 0.0f);
    assert(torque.T4 == 0.0f);

    /* 两圆明显无交点时保留主动杆角，未定义的派生几何量保持零。 */
    boundary_config.geometry.l5 = 0.21f;
    VMC_State_Calc(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(fabsf(boundary_leg.phi1 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi4 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.phi2 == 0.0f);
    assert(boundary_leg.L0 == 0.0f);

    /* 速度非有限不影响已经定义的位置中间量。 */
    config.geometry.l5 = 0.0f;
    VMC_State_Calc(&config, 0.5f, -0.8f, NAN, -0.4f, &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(fabsf(boundary_leg.L0 - leg.L0) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi0 - leg.phi0) < TEST_TOLERANCE);
    assert(boundary_leg.d_L0 == 0.0f);
    assert(boundary_leg.d_phi0 == 0.0f);

    /* scale和ratio共同构成位置、速度和力矩坐标变换。 */
    boundary_config = config;
    boundary_config.joint[CHASSIS_JOINT_PHI1].scale = 0.0f;
    VMC_State_Calc(&boundary_config,
                   0.5f,
                   -0.8f,
                   0.7f,
                   -0.4f,
                   &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(VMC_Torque_Calc(&boundary_config,
                           &leg,
                           -30.0f,
                           1.2f,
                           &torque) == 0U);

    boundary_config = config;
    boundary_config.joint[CHASSIS_JOINT_PHI1].ratio = 0.0f;
    VMC_State_Calc(&boundary_config,
                   0.5f,
                   -0.8f,
                   0.7f,
                   -0.4f,
                   &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(VMC_Torque_Calc(&boundary_config,
                           &leg,
                           -30.0f,
                           1.2f,
                           &torque) == 0U);

    return 0;
}
