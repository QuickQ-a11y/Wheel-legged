#include "chassis_vmc.h"

#include <assert.h>
#include <math.h>

#define TEST_TOLERANCE 2.0e-5f

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
            [CHASSIS_JOINT_FRONT] = {
                .angle_offset_rad = CHASSIS_PI,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
            [CHASSIS_JOINT_BACK] = {
                .angle_offset_rad = 0.0f,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
        },
    };
    Chassis_Leg_t leg;
    Chassis_Leg_t inverse_leg;
    VMC_Torque_t torque;
    VMC_Force_t force;
    VMC_Joint_Target_t target;
    Chassis_Leg_Config_t boundary_config;
    Chassis_Leg_t boundary_leg;
    float front_motor_position_rad;
    float back_motor_position_rad;

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
    assert(fabsf(torque.T_A + 4.89775487f) < TEST_TOLERANCE);
    assert(fabsf(torque.T_E - 3.69775487f) < TEST_TOLERANCE);
    assert(VMC_Force_Calc(&config,
                                &leg,
                                torque.T_A,
                                torque.T_E,
                                &force) == 1U);
    assert(fabsf(force.F0 + 30.0f) < TEST_TOLERANCE);
    assert(fabsf(force.Tp - 1.2f) < TEST_TOLERANCE);

    assert(VMC_Inverse_Calc(&config,
                               &leg,
                               0.15f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    front_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_FRONT].angle_scale;
    back_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_BACK].angle_scale;
    VMC_State_Calc(&config,
                   front_motor_position_rad,
                   back_motor_position_rad,
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
    front_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_FRONT].angle_scale;
    back_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_BACK].angle_scale;
    VMC_State_Calc(&config,
                   front_motor_position_rad,
                   back_motor_position_rad,
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
    boundary_config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_FRONT].angle_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_FRONT].torque_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].angle_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].torque_scale = 1.0f;
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
    assert(torque.T_A == 0.0f);
    assert(torque.T_E == 0.0f);

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

    return 0;
}
