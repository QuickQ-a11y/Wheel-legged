#include "chassis_vmc.h"

#include "Angle.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define VMC_LEN_EPS 1.0e-6f
#define VMC_DET_EPS 1.0e-6f
#define VMC_SIN_EPS 1.0e-6f
#define VMC_SQ_REL (16.0f * FLT_EPSILON)

/** @brief 检查五连杆尺寸是否可用于几何运算。 */
static uint8_t VMC_IsGeometryValid(const chassis_geometry_config_t *geometry)
{
    return ((geometry != NULL) &&
            isfinite(geometry->link1_m) &&
            isfinite(geometry->link2_m) &&
            isfinite(geometry->link3_m) &&
            isfinite(geometry->link4_m) &&
            isfinite(geometry->frame_joint_distance_m) &&
            (geometry->link1_m > 0.0f) &&
            (geometry->link2_m > 0.0f) &&
            (geometry->link3_m > 0.0f) &&
            (geometry->link4_m > 0.0f) &&
            (geometry->frame_joint_distance_m >= 0.0f)) ? 1U : 0U;
}

/** @brief 将反三角函数输入限制到浮点误差允许的[-1, 1]范围。 */
static float VMC_LimitFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/**
 * @brief 根据两个主动关节反馈完成五连杆正运动学和速度解算。
 *
 * B、D是前后主动杆端点，C是两根从动杆交点及轮轴位置。任一运算
 * 无定义时停止后续计算，但保留此前已经得到的有限中间量供Watch观察。
 */
void VMC_CalcState(const chassis_leg_config_t *config,
                    float front_position_rad,
                    float back_position_rad,
                    float front_speed_radps,
                    float back_speed_radps,
                    chassis_vmc_state_t *leg)
{
    const chassis_geometry_config_t *geometry;
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
    float sqrt_scale;
    float sqrt_tolerance;
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
    float length_speed_mps;
    float phi0_speed_radps;

    if (leg == NULL)
    {
        return;
    }
    memset(leg, 0, sizeof(*leg));
    if (config == NULL)
    {
        return;
    }
    geometry = &config->geometry;
    if ((VMC_IsGeometryValid(geometry) == 0U) ||
        (!isfinite(front_position_rad)) ||
        (!isfinite(back_position_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_FRONT].angle_offset_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_FRONT].angle_scale)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_BACK].angle_offset_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_BACK].angle_scale)))
    {
        return;
    }

    /* 电机角度先按零位和极性转换成五连杆几何角。 */
    leg->phi1_rad = config->joint[CHASSIS_JOINT_FRONT].angle_offset_rad +
                    config->joint[CHASSIS_JOINT_FRONT].angle_scale *
                        front_position_rad;
    leg->phi4_rad = config->joint[CHASSIS_JOINT_BACK].angle_offset_rad +
                    config->joint[CHASSIS_JOINT_BACK].angle_scale *
                        back_position_rad;
    if ((!isfinite(leg->phi1_rad)) || (!isfinite(leg->phi4_rad)))
    {
        leg->phi1_rad = 0.0f;
        leg->phi4_rad = 0.0f;
        return;
    }

    /* 以两髋轴中点为原点，先求主动杆端点B和D。 */
    point_b_x = -geometry->frame_joint_distance_m * 0.5f +
                geometry->link1_m * cosf(leg->phi1_rad);
    point_b_y = geometry->link1_m * sinf(leg->phi1_rad);
    point_d_x = geometry->frame_joint_distance_m * 0.5f +
                geometry->link4_m * cosf(leg->phi4_rad);
    point_d_y = geometry->link4_m * sinf(leg->phi4_rad);

    point_bd_x = point_d_x - point_b_x;
    point_bd_y = point_d_y - point_b_y;
    point_bd_length = sqrtf(point_bd_x * point_bd_x + point_bd_y * point_bd_y);
    if ((!isfinite(point_bd_length)) || (point_bd_length <= VMC_LEN_EPS))
    {
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
    sqrt_scale = equation_a * equation_a + equation_b * equation_b +
                 equation_c * equation_c;
    sqrt_tolerance = VMC_SQ_REL * sqrt_scale;
    if ((!isfinite(sqrt_argument)) || (!isfinite(sqrt_tolerance)) ||
        (sqrt_argument < -sqrt_tolerance))
    {
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
    if ((!isfinite(leg->phi2_rad)) || (!isfinite(point_c_x)) ||
        (!isfinite(point_c_y)))
    {
        leg->phi2_rad = 0.0f;
        return;
    }

    leg->phi3_rad = atan2f(point_c_y - point_d_y, point_c_x - point_d_x);
    /* 虚拟腿由髋轴中点指向C点，极坐标即腿长和phi0。 */
    leg->length_m = sqrtf(point_c_x * point_c_x + point_c_y * point_c_y);
    if ((!isfinite(leg->phi3_rad)) || (!isfinite(leg->length_m)) ||
        (leg->length_m <= VMC_LEN_EPS))
    {
        return;
    }
    leg->phi0_rad = atan2f(point_c_y, point_c_x);
    if (!isfinite(leg->phi0_rad))
    {
        leg->phi0_rad = 0.0f;
        return;
    }
    leg->phi0_total_rad = leg->phi0_rad;

    /* 对闭链约束求导，先由主动杆速度解出从动杆phi2速度。 */
    if ((!isfinite(front_speed_radps)) || (!isfinite(back_speed_radps)))
    {
        return;
    }
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
    if ((!isfinite(determinant)) || (fabsf(determinant) <= VMC_DET_EPS))
    {
        return;
    }

    right_x = point_d_speed_x - point_b_speed_x;
    right_y = point_d_speed_y - point_b_speed_y;
    phi2_speed_radps = (right_x * matrix22 - matrix12 * right_y) /
                       determinant;
    if (!isfinite(phi2_speed_radps))
    {
        return;
    }
    point_c_speed_x = point_b_speed_x + matrix11 * phi2_speed_radps;
    point_c_speed_y = point_b_speed_y + matrix21 * phi2_speed_radps;

    /* 将C点笛卡尔速度投影到虚拟腿径向和切向。 */
    length_speed_mps =
        (point_c_x * point_c_speed_x + point_c_y * point_c_speed_y) /
        leg->length_m;

    length_squared = leg->length_m * leg->length_m;
    phi0_speed_radps =
        (point_c_x * point_c_speed_y - point_c_y * point_c_speed_x) /
        length_squared;
    if ((!isfinite(length_speed_mps)) || (!isfinite(phi0_speed_radps)))
    {
        return;
    }
    leg->length_speed_mps = length_speed_mps;
    leg->phi0_speed_radps = phi0_speed_radps;
    leg->valid = 1U;
}

/**
 * @brief 由目标C点位置解算前后主动杆目标角。
 *
 * 先分别检查前后两组二连杆可达性，再选择当前实机装配分支，并把
 * 结果转换为距离当前反馈最近的等价角，避免主动关节跨零跳变。
 */
uint8_t VMC_CalcJointTarget(const chassis_leg_config_t *config,
                              const chassis_vmc_state_t *current_leg,
                              float target_length_m,
                              float target_phi0_rad,
                              chassis_vmc_joint_target_t *target)
{
    const chassis_geometry_config_t *geometry;
    float point_c_x;
    float point_c_y;
    float front_vector_x;
    float back_vector_x;
    float front_distance;
    float back_distance;
    float front_cosine;
    float back_cosine;
    float front_base_angle;
    float back_base_angle;

    if (target != NULL)
    {
        memset(target, 0, sizeof(*target));
    }
    if ((config == NULL) || (current_leg == NULL) || (target == NULL) ||
        (!isfinite(target_length_m)) || (!isfinite(target_phi0_rad)) ||
        (!isfinite(current_leg->phi1_rad)) ||
        (!isfinite(current_leg->phi4_rad)))
    {
        return 0U;
    }

    geometry = &config->geometry;
    /* 机械尺寸或目标腿长不具备物理意义时不进入三角形求解。 */
    if ((VMC_IsGeometryValid(geometry) == 0U) ||
        (target_length_m <= VMC_LEN_EPS))
    {
        return 0U;
    }

    point_c_x = target_length_m * cosf(target_phi0_rad);
    point_c_y = target_length_m * sinf(target_phi0_rad);
    /* C点到前后髋轴的距离必须分别位于对应二连杆可达区间内。 */
    front_vector_x = point_c_x + geometry->frame_joint_distance_m * 0.5f;
    back_vector_x = point_c_x - geometry->frame_joint_distance_m * 0.5f;
    front_distance = sqrtf(front_vector_x * front_vector_x +
                           point_c_y * point_c_y);
    back_distance = sqrtf(back_vector_x * back_vector_x +
                          point_c_y * point_c_y);
    if ((front_distance <= VMC_LEN_EPS) ||
        (back_distance <= VMC_LEN_EPS) ||
        (front_distance > geometry->link1_m + geometry->link2_m +
                              VMC_LEN_EPS) ||
        (front_distance < fabsf(geometry->link1_m - geometry->link2_m) -
                              VMC_LEN_EPS) ||
        (back_distance > geometry->link4_m + geometry->link3_m +
                             VMC_LEN_EPS) ||
        (back_distance < fabsf(geometry->link4_m - geometry->link3_m) -
                             VMC_LEN_EPS))
    {
        return 0U;
    }

    front_cosine =
        (geometry->link1_m * geometry->link1_m +
         front_distance * front_distance -
         geometry->link2_m * geometry->link2_m) /
        (2.0f * geometry->link1_m * front_distance);
    back_cosine =
        (geometry->link4_m * geometry->link4_m +
         back_distance * back_distance -
         geometry->link3_m * geometry->link3_m) /
        (2.0f * geometry->link4_m * back_distance);
    front_base_angle = atan2f(point_c_y, front_vector_x);
    back_base_angle = atan2f(point_c_y, back_vector_x);

    /* 当前实机装配对应前链上分支、后链下分支。 */
    target->phi1_rad = front_base_angle +
                       acosf(VMC_LimitFloat(front_cosine, -1.0f, 1.0f));
    target->phi4_rad = back_base_angle -
                       acosf(VMC_LimitFloat(back_cosine, -1.0f, 1.0f));
    target->phi1_rad =
        Algorithm_AngleNearestEquivalentRad(target->phi1_rad,
                                            current_leg->phi1_rad);
    target->phi4_rad =
        Algorithm_AngleNearestEquivalentRad(target->phi4_rad,
                                            current_leg->phi4_rad);

    if ((!isfinite(target->phi1_rad)) || (!isfinite(target->phi4_rad)))
    {
        memset(target, 0, sizeof(*target));
        return 0U;
    }
    return 1U;
}

/**
 * @brief 使用虚功关系把虚拟腿广义力映射为两个主动关节力矩。
 *
 * support_force_n沿虚拟腿方向，swing_torque_nm绕髋轴中点作用；
 * denominator接近零表示两根从动杆接近奇异，必须保持零输出。
 */
uint8_t VMC_CalcTorque(const chassis_leg_config_t *config,
                       const chassis_vmc_state_t *leg,
                       float support_force_n,
                       float swing_torque_nm,
                       chassis_vmc_torque_t *torque)
{
    float denominator;
    float front_torque_nm;
    float back_torque_nm;

    if (torque == NULL)
    {
        return 0U;
    }
    memset(torque, 0, sizeof(*torque));
    if ((config == NULL) || (leg == NULL) ||
        (VMC_IsGeometryValid(&config->geometry) == 0U) ||
        (!isfinite(leg->phi0_rad)) || (!isfinite(leg->phi1_rad)) ||
        (!isfinite(leg->phi2_rad)) || (!isfinite(leg->phi3_rad)) ||
        (!isfinite(leg->phi4_rad)) || (!isfinite(leg->length_m)) ||
        (!isfinite(support_force_n)) || (!isfinite(swing_torque_nm)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_FRONT].torque_scale)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_BACK].torque_scale)))
    {
        return 0U;
    }

    denominator = sinf(leg->phi2_rad - leg->phi3_rad);
    if ((!isfinite(denominator)) || (fabsf(denominator) <= VMC_SIN_EPS) ||
        (fabsf(leg->length_m) <= VMC_LEN_EPS))
    {
        return 0U;
    }

    /* 雅可比转置展开后的前、后主动关节力矩。 */
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

    if ((!isfinite(front_torque_nm)) || (!isfinite(back_torque_nm)))
    {
        return 0U;
    }

    torque->front_nm = front_torque_nm *
                       config->joint[CHASSIS_JOINT_FRONT].torque_scale;
    torque->back_nm = back_torque_nm *
                      config->joint[CHASSIS_JOINT_BACK].torque_scale;
    if ((!isfinite(torque->front_nm)) || (!isfinite(torque->back_nm)))
    {
        memset(torque, 0, sizeof(*torque));
        return 0U;
    }
    return 1U;
}
