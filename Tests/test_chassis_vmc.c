#include "chassis_vmc.h"

#include <assert.h>
#include <math.h>

#define TEST_TOLERANCE 2.0e-5f

int main(void)
{
    chassis_leg_config_t config = {
        .geometry = {
            .link1_m = 0.215f,
            .link2_m = 0.258f,
            .link3_m = 0.258f,
            .link4_m = 0.215f,
            .frame_joint_distance_m = 0.0f,
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
    chassis_vmc_state_t leg;
    chassis_vmc_state_t inverse_leg;
    chassis_vmc_torque_t torque;
    chassis_vmc_joint_target_t target;
    chassis_leg_config_t boundary_config;
    chassis_vmc_state_t boundary_leg;
    float front_motor_position_rad;
    float back_motor_position_rad;

    VMC_CalcState(&config, 0.5f, -0.8f, 0.7f, -0.4f, &leg);

    assert(leg.valid == 1U);
    assert(fabsf(leg.phi1_rad - 2.64159265f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi2_rad - 0.99543859f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi3_rad - 2.44615406f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi4_rad - 0.8f) < TEST_TOLERANCE);
    assert(fabsf(leg.length_m - 0.32316671f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0_rad - 1.72079633f) < TEST_TOLERANCE);
    assert(fabsf(leg.length_speed_mps - 0.15758435f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0_speed_radps + 0.15f) < TEST_TOLERANCE);

    assert(VMC_CalcTorque(&config, &leg, -30.0f, 1.2f, &torque) == 1U);
    assert(fabsf(torque.front_nm + 4.89775487f) < TEST_TOLERANCE);
    assert(fabsf(torque.back_nm - 3.69775487f) < TEST_TOLERANCE);

    assert(VMC_CalcJointTarget(&config,
                               &leg,
                               0.15f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    front_motor_position_rad =
        (target.phi1_rad -
         config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_FRONT].angle_scale;
    back_motor_position_rad =
        (target.phi4_rad -
         config.joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_BACK].angle_scale;
    VMC_CalcState(&config,
                   front_motor_position_rad,
                   back_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.length_m - 0.15f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0_rad - CHASSIS_HALF_PI) < TEST_TOLERANCE);

    /* 0.05 m人为门槛已删除，真实二连杆最小可达距离约为0.043 m。 */
    assert(VMC_CalcJointTarget(&config,
                               &inverse_leg,
                               0.045f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    assert(VMC_CalcJointTarget(&config,
                               &inverse_leg,
                               0.042f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);

    config.geometry.frame_joint_distance_m = 0.06f;
    assert(VMC_CalcJointTarget(&config,
                               &inverse_leg,
                               0.35f,
                               1.40f,
                               &target) == 1U);
    front_motor_position_rad =
        (target.phi1_rad -
         config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_FRONT].angle_scale;
    back_motor_position_rad =
        (target.phi4_rad -
         config.joint[CHASSIS_JOINT_BACK].angle_offset_rad) /
        config.joint[CHASSIS_JOINT_BACK].angle_scale;
    VMC_CalcState(&config,
                   front_motor_position_rad,
                   back_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.length_m - 0.35f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0_rad - 1.40f) < TEST_TOLERANCE);

    assert(VMC_CalcJointTarget(&config,
                               &inverse_leg,
                               0.60f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);
    assert(target.phi1_rad == 0.0f);
    assert(target.phi4_rad == 0.0f);
    assert(VMC_CalcJointTarget(&config,
                               &inverse_leg,
                               NAN,
                               CHASSIS_HALF_PI,
                               &target) == 0U);

    /* 相切两圆的位置解存在，但从动杆共线时速度雅可比不可逆。 */
    boundary_config = config;
    boundary_config.geometry.link1_m = 0.1f;
    boundary_config.geometry.link2_m = 0.1f;
    boundary_config.geometry.link3_m = 0.1f;
    boundary_config.geometry.link4_m = 0.1f;
    boundary_config.geometry.frame_joint_distance_m = 0.2f;
    boundary_config.joint[CHASSIS_JOINT_FRONT].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_FRONT].angle_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_FRONT].torque_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].angle_scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_BACK].torque_scale = 1.0f;
    VMC_CalcState(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid == 0U);
    assert(fabsf(boundary_leg.length_m - 0.1f) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi0_rad - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.length_speed_mps == 0.0f);
    assert(boundary_leg.phi0_speed_radps == 0.0f);
    assert(VMC_CalcTorque(&boundary_config,
                          &boundary_leg,
                          -30.0f,
                          1.2f,
                          &torque) == 0U);
    assert(torque.front_nm == 0.0f);
    assert(torque.back_nm == 0.0f);

    /* 两圆明显无交点时保留主动杆角，未定义的派生几何量保持零。 */
    boundary_config.geometry.frame_joint_distance_m = 0.21f;
    VMC_CalcState(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid == 0U);
    assert(fabsf(boundary_leg.phi1_rad - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi4_rad - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.phi2_rad == 0.0f);
    assert(boundary_leg.length_m == 0.0f);

    /* 速度非有限不影响已经定义的位置中间量。 */
    config.geometry.frame_joint_distance_m = 0.0f;
    VMC_CalcState(&config, 0.5f, -0.8f, NAN, -0.4f, &boundary_leg);
    assert(boundary_leg.valid == 0U);
    assert(fabsf(boundary_leg.length_m - leg.length_m) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi0_rad - leg.phi0_rad) < TEST_TOLERANCE);
    assert(boundary_leg.length_speed_mps == 0.0f);
    assert(boundary_leg.phi0_speed_radps == 0.0f);

    return 0;
}
