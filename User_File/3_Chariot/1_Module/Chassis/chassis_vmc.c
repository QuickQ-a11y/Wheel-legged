#include "chassis_vmc.h"

#include <math.h>
#include <string.h>

#define CHASSIS_VMC_EPSILON 1.0e-6f

void vmc_calc_state(const chassis_leg_config_t *config,
                    float front_position_rad,
                    float back_position_rad,
                    float front_speed_radps,
                    float back_speed_radps,
                    chassis_vmc_state_t *leg)
{
    const chassis_geometry_config_t *geometry = &config->geometry;
    float point_b_x;
    float point_b_y;
    float point_c_x;
    float point_c_y;
    float point_d_x;
    float point_d_y;
    float point_bd_x;
    float point_bd_y;
    float point_bd_length;
    float equation_a;
    float equation_b;
    float equation_c;
    float sqrt_argument;
    float sqrt_value;
    float phi1_speed_radps;
    float phi4_speed_radps;
    float point_b_speed_x;
    float point_b_speed_y;
    float point_d_speed_x;
    float point_d_speed_y;
    float matrix11;
    float matrix12;
    float matrix21;
    float matrix22;
    float determinant;
    float right_x;
    float right_y;
    float phi2_speed_radps;
    float point_c_speed_x;
    float point_c_speed_y;
    float length_squared;

    memset(leg, 0, sizeof(*leg));

    /* 电机角度先按零位和极性转换成五连杆几何角。 */
    leg->phi1_rad = config->joint[CHASSIS_JOINT_FRONT].angle_offset_rad +
                    config->joint[CHASSIS_JOINT_FRONT].angle_scale *
                        front_position_rad;
    leg->phi4_rad = config->joint[CHASSIS_JOINT_BACK].angle_offset_rad +
                    config->joint[CHASSIS_JOINT_BACK].angle_scale *
                        back_position_rad;

    point_b_x = -geometry->frame_joint_distance_m * 0.5f +
                geometry->link1_m * cosf(leg->phi1_rad);
    point_b_y = geometry->link1_m * sinf(leg->phi1_rad);
    point_d_x = geometry->frame_joint_distance_m * 0.5f +
                geometry->link4_m * cosf(leg->phi4_rad);
    point_d_y = geometry->link4_m * sinf(leg->phi4_rad);

    point_bd_x = point_d_x - point_b_x;
    point_bd_y = point_d_y - point_b_y;
    point_bd_length = sqrtf(point_bd_x * point_bd_x + point_bd_y * point_bd_y);
    if (point_bd_length <= CHASSIS_VMC_EPSILON)
    {
        memset(leg, 0, sizeof(*leg));
        return;
    }

    /* C 点取与当前实机装配一致的两圆交点分支。 */
    equation_a = 2.0f * geometry->link2_m * point_bd_x;
    equation_b = 2.0f * geometry->link2_m * point_bd_y;
    equation_c = geometry->link2_m * geometry->link2_m +
                 point_bd_length * point_bd_length -
                 geometry->link3_m * geometry->link3_m;
    sqrt_argument = equation_a * equation_a + equation_b * equation_b -
                    equation_c * equation_c;
    if (sqrt_argument < -CHASSIS_VMC_EPSILON)
    {
        memset(leg, 0, sizeof(*leg));
        return;
    }
    if (sqrt_argument < 0.0f)
    {
        sqrt_argument = 0.0f;
    }

    sqrt_value = sqrtf(sqrt_argument);
    leg->phi2_rad = 2.0f * atan2f(equation_b + sqrt_value,
                                  equation_a + equation_c);
    point_c_x = -geometry->frame_joint_distance_m * 0.5f +
                geometry->link1_m * cosf(leg->phi1_rad) +
                geometry->link2_m * cosf(leg->phi2_rad);
    point_c_y = geometry->link1_m * sinf(leg->phi1_rad) +
                geometry->link2_m * sinf(leg->phi2_rad);

    leg->phi3_rad = atan2f(point_c_y - point_d_y, point_c_x - point_d_x);
    leg->length_m = sqrtf(point_c_x * point_c_x + point_c_y * point_c_y);
    if ((leg->length_m <= geometry->min_leg_length_m) ||
        (leg->length_m <= CHASSIS_VMC_EPSILON))
    {
        memset(leg, 0, sizeof(*leg));
        return;
    }
    leg->phi0_rad = atan2f(point_c_y, point_c_x);

    /* 对闭链约束求导，得到 C 点速度、腿长速度和 phi0 角速度。 */
    phi1_speed_radps = config->joint[CHASSIS_JOINT_FRONT].angle_scale *
                       front_speed_radps;
    phi4_speed_radps = config->joint[CHASSIS_JOINT_BACK].angle_scale *
                       back_speed_radps;
    point_b_speed_x = -geometry->link1_m * sinf(leg->phi1_rad) *
                      phi1_speed_radps;
    point_b_speed_y = geometry->link1_m * cosf(leg->phi1_rad) *
                      phi1_speed_radps;
    point_d_speed_x = -geometry->link4_m * sinf(leg->phi4_rad) *
                      phi4_speed_radps;
    point_d_speed_y = geometry->link4_m * cosf(leg->phi4_rad) *
                      phi4_speed_radps;

    matrix11 = -geometry->link2_m * sinf(leg->phi2_rad);
    matrix21 = geometry->link2_m * cosf(leg->phi2_rad);
    matrix12 = geometry->link3_m * sinf(leg->phi3_rad);
    matrix22 = -geometry->link3_m * cosf(leg->phi3_rad);
    determinant = matrix11 * matrix22 - matrix12 * matrix21;
    if (fabsf(determinant) <= CHASSIS_VMC_EPSILON)
    {
        memset(leg, 0, sizeof(*leg));
        return;
    }

    right_x = point_d_speed_x - point_b_speed_x;
    right_y = point_d_speed_y - point_b_speed_y;
    phi2_speed_radps = (right_x * matrix22 - matrix12 * right_y) /
                       determinant;
    point_c_speed_x = point_b_speed_x + matrix11 * phi2_speed_radps;
    point_c_speed_y = point_b_speed_y + matrix21 * phi2_speed_radps;

    leg->length_speed_mps =
        (point_c_x * point_c_speed_x + point_c_y * point_c_speed_y) /
        leg->length_m;

    length_squared = leg->length_m * leg->length_m;
    if (fabsf(length_squared) <= CHASSIS_VMC_EPSILON)
    {
        memset(leg, 0, sizeof(*leg));
        return;
    }
    leg->phi0_speed_radps =
        (point_c_x * point_c_speed_y - point_c_y * point_c_speed_x) /
        length_squared;
}

void vmc_calc_torque(const chassis_leg_config_t *config,
                     const chassis_vmc_state_t *leg,
                     float support_force_n,
                     float swing_torque_nm,
                     chassis_vmc_torque_t *torque)
{
    float denominator;
    float front_torque_nm;
    float back_torque_nm;

    memset(torque, 0, sizeof(*torque));

    denominator = sinf(leg->phi2_rad - leg->phi3_rad);
    if ((fabsf(denominator) <= CHASSIS_VMC_EPSILON) ||
        (fabsf(leg->length_m) <= CHASSIS_VMC_EPSILON))
    {
        return;
    }

    front_torque_nm =
        (-config->geometry.link1_m *
         sinf(leg->phi0_rad - leg->phi3_rad) *
         sinf(leg->phi1_rad - leg->phi2_rad) / denominator *
         support_force_n) +
        (-config->geometry.link1_m *
         sinf(leg->phi1_rad - leg->phi2_rad) *
         cosf(leg->phi0_rad - leg->phi3_rad) /
         (leg->length_m * denominator) * swing_torque_nm);

    back_torque_nm =
        (-config->geometry.link4_m *
         sinf(leg->phi0_rad - leg->phi2_rad) *
         sinf(leg->phi3_rad - leg->phi4_rad) / denominator *
         support_force_n) +
        (-config->geometry.link4_m *
         sinf(leg->phi3_rad - leg->phi4_rad) *
         cosf(leg->phi0_rad - leg->phi2_rad) /
         (leg->length_m * denominator) * swing_torque_nm);

    torque->front_nm = front_torque_nm *
                       config->joint[CHASSIS_JOINT_FRONT].torque_scale;
    torque->back_nm = back_torque_nm *
                      config->joint[CHASSIS_JOINT_BACK].torque_scale;
}
