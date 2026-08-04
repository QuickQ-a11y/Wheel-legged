#include "chassis_vmc.h"

#include "Angle.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define VMC_LEN_EPS 1.0e-6f
#define VMC_DET_EPS 1.0e-6f
#define VMC_SIN_EPS 1.0e-6f
#define VMC_SCALE_EPS 1.0e-6f
#define VMC_SQ_REL (16.0f * FLT_EPSILON)

/** @brief 检查五连杆尺寸是否可用于几何运算。 */
static uint8_t VMC_Geometry_Valid(const Chassis_Geometry_Config_t *geometry)
{
    return ((geometry != NULL) &&
            isfinite(geometry->l1) &&
            isfinite(geometry->l2) &&
            isfinite(geometry->l3) &&
            isfinite(geometry->l4) &&
            isfinite(geometry->l5) &&
            (geometry->l1 > 0.0f) &&
            (geometry->l2 > 0.0f) &&
            (geometry->l3 > 0.0f) &&
            (geometry->l4 > 0.0f) &&
            (geometry->l5 >= 0.0f)) ? 1U : 0U;
}

/** @brief 将反三角函数输入限制到浮点误差允许的[-1, 1]范围。 */
static float VMC_Limit(float value, float min_value, float max_value)
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
 * B是phi1主动杆端点，D是phi4主动杆端点，C是从动杆交点及轮轴位置。任一运算
 * 无定义时停止后续计算，但保留此前已经得到的有限中间量供Watch观察。
 */
void VMC_State_Calc(const Chassis_Leg_Config_t *config,
                    float phi1_motor_position_rad,
                    float phi4_motor_position_rad,
                    float phi1_motor_speed_radps,
                    float phi4_motor_speed_radps,
                    Chassis_Leg_t *leg)
{
    const Chassis_Geometry_Config_t *geometry;
    float phi1_pivot_x;
    float phi4_pivot_x;
    float b_x;
    float b_y;
    float c_x;
    float c_y;
    float d_x;
    float d_y;
    float bd_x;
    float bd_y;
    float bd;
    float a;
    float b;
    float c;
    float sqrt_arg;
    float sqrt_scale;
    float sqrt_tol;
    float sqrt_value;
    float phi1_scale;
    float phi4_scale;
    float d_phi1;
    float d_phi4;
    float d_b_x;
    float d_b_y;
    float d_d_x;
    float d_d_y;
    float J11;
    float J12;
    float J21;
    float J22;
    float det;
    float rhs_x;
    float rhs_y;
    float d_phi2;
    float d_c_x;
    float d_c_y;
    float L0_sq;
    float d_L0;
    float d_phi0;

    if (leg == NULL)
    {
        return;
    }
    /* VMC只拥有几何状态，不清除同一腿对象中的控制目标和广义力。 */
    leg->phi0 = 0.0f;
    leg->phi1 = 0.0f;
    leg->phi2 = 0.0f;
    leg->phi3 = 0.0f;
    leg->phi4 = 0.0f;
    leg->L0 = 0.0f;
    leg->d_L0 = 0.0f;
    leg->d_phi0 = 0.0f;
    leg->valid_flag = 0U;
    if (config == NULL)
    {
        return;
    }
    geometry = &config->geometry;
    phi1_scale = config->joint[CHASSIS_JOINT_PHI1].scale *
                 config->joint[CHASSIS_JOINT_PHI1].ratio;
    phi4_scale = config->joint[CHASSIS_JOINT_PHI4].scale *
                 config->joint[CHASSIS_JOINT_PHI4].ratio;
    if ((VMC_Geometry_Valid(geometry) == 0U) ||
        (!isfinite(phi1_motor_position_rad)) ||
        (!isfinite(phi4_motor_position_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI1].angle_offset_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI1].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI1].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(phi1_scale)) ||
        (fabsf(phi1_scale) <= VMC_SCALE_EPS) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI4].angle_offset_rad)) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI4].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI4].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(phi4_scale)) ||
        (fabsf(phi4_scale) <= VMC_SCALE_EPS))
    {
        return;
    }

    /* 电机角度先按零位和极性转换成五连杆几何角。 */
    leg->phi1 = config->joint[CHASSIS_JOINT_PHI1].angle_offset_rad +
                    phi1_scale * phi1_motor_position_rad;
    leg->phi4 = config->joint[CHASSIS_JOINT_PHI4].angle_offset_rad +
                    phi4_scale * phi4_motor_position_rad;
    if ((!isfinite(leg->phi1)) || (!isfinite(leg->phi4)))
    {
        leg->phi1 = 0.0f;
        leg->phi4 = 0.0f;
        return;
    }

    /* 侧视平面使用+x向前、+y向下，平面角正方向对应车体+Y。 */
    phi1_pivot_x = -geometry->l5 * 0.5f;
    phi4_pivot_x = geometry->l5 * 0.5f;
    b_x = phi1_pivot_x + geometry->l1 * cosf(leg->phi1);
    b_y = geometry->l1 * sinf(leg->phi1);
    d_x = phi4_pivot_x + geometry->l4 * cosf(leg->phi4);
    d_y = geometry->l4 * sinf(leg->phi4);

    bd_x = d_x - b_x;
    bd_y = d_y - b_y;
    bd = sqrtf(bd_x * bd_x + bd_y * bd_y);
    if ((!isfinite(bd)) || (bd <= VMC_LEN_EPS))
    {
        return;
    }

    /* 配合phi1/phi4绕车体+Y增大的定义，选择轮轴位于髋关节下方的交点。 */
    a = 2.0f * geometry->l2 * bd_x;
    b = 2.0f * geometry->l2 * bd_y;
    c = geometry->l2 * geometry->l2 +
                 bd * bd -
                 geometry->l3 * geometry->l3;
    sqrt_arg = a * a + b * b -
                    c * c;
    sqrt_scale = a * a + b * b +
                 c * c;
    sqrt_tol = VMC_SQ_REL * sqrt_scale;
    if ((!isfinite(sqrt_arg)) || (!isfinite(sqrt_tol)) ||
        (sqrt_arg < -sqrt_tol))
    {
        return;
    }
    if (sqrt_arg < 0.0f)
    {
        sqrt_arg = 0.0f;
    }

    sqrt_value = sqrtf(sqrt_arg);
    leg->phi2 = 2.0f * atan2f(b + sqrt_value,
                                  a + c);
    c_x = b_x + geometry->l2 * cosf(leg->phi2);
    c_y = b_y + geometry->l2 * sinf(leg->phi2);
    if ((!isfinite(leg->phi2)) || (!isfinite(c_x)) ||
        (!isfinite(c_y)))
    {
        leg->phi2 = 0.0f;
        return;
    }

    leg->phi3 = atan2f(c_y - d_y, c_x - d_x);
    /* 虚拟腿由两个输出铰点的中点指向C点，极坐标即腿长和phi0。 */
    leg->L0 = sqrtf(c_x * c_x + c_y * c_y);
    if ((!isfinite(leg->phi3)) || (!isfinite(leg->L0)) ||
        (leg->L0 <= VMC_LEN_EPS))
    {
        return;
    }
    leg->phi0 = atan2f(c_y, c_x);
    if (!isfinite(leg->phi0))
    {
        leg->phi0 = 0.0f;
        return;
    }
    leg->phi0_total = leg->phi0;

    /* 对闭链约束求导，先由主动杆速度解出从动杆phi2速度。 */
    if ((!isfinite(phi1_motor_speed_radps)) ||
        (!isfinite(phi4_motor_speed_radps)))
    {
        return;
    }
    d_phi1 = phi1_scale * phi1_motor_speed_radps;
    d_phi4 = phi4_scale * phi4_motor_speed_radps;
    d_b_x = -geometry->l1 * sinf(leg->phi1) *
                      d_phi1;
    d_b_y = geometry->l1 * cosf(leg->phi1) *
                      d_phi1;
    d_d_x = -geometry->l4 * sinf(leg->phi4) *
                      d_phi4;
    d_d_y = geometry->l4 * cosf(leg->phi4) *
                      d_phi4;

    J11 = -geometry->l2 * sinf(leg->phi2);
    J21 = geometry->l2 * cosf(leg->phi2);
    J12 = geometry->l3 * sinf(leg->phi3);
    J22 = -geometry->l3 * cosf(leg->phi3);
    det = J11 * J22 - J12 * J21;
    if ((!isfinite(det)) || (fabsf(det) <= VMC_DET_EPS))
    {
        return;
    }

    rhs_x = d_d_x - d_b_x;
    rhs_y = d_d_y - d_b_y;
    d_phi2 = (rhs_x * J22 - J12 * rhs_y) /
                       det;
    if (!isfinite(d_phi2))
    {
        return;
    }
    d_c_x = d_b_x + J11 * d_phi2;
    d_c_y = d_b_y + J21 * d_phi2;

    /* 将C点笛卡尔速度投影到虚拟腿径向和切向。 */
    d_L0 =
        (c_x * d_c_x + c_y * d_c_y) /
        leg->L0;

    L0_sq = leg->L0 * leg->L0;
    d_phi0 =
        (c_x * d_c_y - c_y * d_c_x) /
        L0_sq;
    if ((!isfinite(d_L0)) || (!isfinite(d_phi0)))
    {
        return;
    }
    leg->d_L0 = d_L0;
    leg->d_phi0 = d_phi0;
    leg->valid_flag = 1U;
}

/**
 * @brief 由目标C点位置解算phi1和phi4主动杆目标角。
 *
 * 先分别检查phi1和phi4两组二连杆可达性，再选择当前实机装配分支，并把
 * 结果转换为距离当前反馈最近的等价角，避免主动关节跨零跳变。
 */
uint8_t VMC_Inverse_Calc(const Chassis_Leg_Config_t *config,
                         const Chassis_Leg_t *current_leg,
                         float target_L0,
                         float target_phi0,
                         VMC_Joint_Target_t *target)
{
    const Chassis_Geometry_Config_t *geometry;
    float c_x;
    float c_y;
    float phi1_pivot_x;
    float phi4_pivot_x;
    float phi1_x;
    float phi4_x;
    float phi1_r;
    float phi4_r;
    float phi1_cos;
    float phi4_cos;
    float phi1_angle;
    float phi4_angle;

    if (target != NULL)
    {
        memset(target, 0, sizeof(*target));
    }
    if ((config == NULL) || (current_leg == NULL) || (target == NULL) ||
        (!isfinite(target_L0)) || (!isfinite(target_phi0)) ||
        (!isfinite(current_leg->phi1)) ||
        (!isfinite(current_leg->phi4)))
    {
        return 0U;
    }

    geometry = &config->geometry;
    /* 机械尺寸或目标腿长不具备物理意义时不进入三角形求解。 */
    if ((VMC_Geometry_Valid(geometry) == 0U) ||
        (target_L0 <= VMC_LEN_EPS))
    {
        return 0U;
    }

    c_x = target_L0 * cosf(target_phi0);
    c_y = target_L0 * sinf(target_phi0);
    phi1_pivot_x = -geometry->l5 * 0.5f;
    phi4_pivot_x = geometry->l5 * 0.5f;
    /* C点到两个输出铰点的距离必须位于各自二连杆可达区间内。 */
    phi1_x = c_x - phi1_pivot_x;
    phi4_x = c_x - phi4_pivot_x;
    phi1_r = sqrtf(phi1_x * phi1_x + c_y * c_y);
    phi4_r = sqrtf(phi4_x * phi4_x + c_y * c_y);
    if ((phi1_r <= VMC_LEN_EPS) ||
        (phi4_r <= VMC_LEN_EPS) ||
        (phi1_r > geometry->l1 + geometry->l2 + VMC_LEN_EPS) ||
        (phi1_r < fabsf(geometry->l1 - geometry->l2) -
                   VMC_LEN_EPS) ||
        (phi4_r > geometry->l4 + geometry->l3 + VMC_LEN_EPS) ||
        (phi4_r < fabsf(geometry->l4 - geometry->l3) -
                  VMC_LEN_EPS))
    {
        return 0U;
    }

    phi1_cos =
        (geometry->l1 * geometry->l1 +
         phi1_r * phi1_r -
         geometry->l2 * geometry->l2) /
        (2.0f * geometry->l1 * phi1_r);
    phi4_cos =
        (geometry->l4 * geometry->l4 +
         phi4_r * phi4_r -
         geometry->l3 * geometry->l3) /
        (2.0f * geometry->l4 * phi4_r);
    phi1_angle = atan2f(c_y, phi1_x);
    phi4_angle = atan2f(c_y, phi4_x);

    /* 与正解使用同一装配分支：phi1取上支，phi4取下支。 */
    target->phi1 = phi1_angle +
                   acosf(VMC_Limit(phi1_cos, -1.0f, 1.0f));
    target->phi4 = phi4_angle -
                   acosf(VMC_Limit(phi4_cos, -1.0f, 1.0f));
    target->phi1 =
        Algorithm_AngleNearestEquivalentRad(target->phi1,
                                            current_leg->phi1);
    target->phi4 =
        Algorithm_AngleNearestEquivalentRad(target->phi4,
                                            current_leg->phi4);

    if ((!isfinite(target->phi1)) || (!isfinite(target->phi4)))
    {
        memset(target, 0, sizeof(*target));
        return 0U;
    }
    return 1U;
}

/** @brief 计算虚拟力到几何关节力矩的2x2映射矩阵。 */
static uint8_t VMC_Matrix_Calc(const Chassis_Leg_Config_t *config,
                               const Chassis_Leg_t *leg,
                               float J[4])
{
    float denominator;

    if (J == NULL)
    {
        return 0U;
    }
    memset(J, 0, sizeof(float) * 4U);
    if ((config == NULL) || (leg == NULL) ||
        (VMC_Geometry_Valid(&config->geometry) == 0U) ||
        (!isfinite(leg->phi0)) || (!isfinite(leg->phi1)) ||
        (!isfinite(leg->phi2)) || (!isfinite(leg->phi3)) ||
        (!isfinite(leg->phi4)) || (!isfinite(leg->L0)))
    {
        return 0U;
    }

    denominator = sinf(leg->phi2 - leg->phi3);
    if ((!isfinite(denominator)) || (fabsf(denominator) <= VMC_SIN_EPS) ||
        (fabsf(leg->L0) <= VMC_LEN_EPS))
    {
        return 0U;
    }

    J[0] =
        (-config->geometry.l1 *
         sinf(leg->phi0 - leg->phi3) *
         sinf(leg->phi1 - leg->phi2) / denominator);
    J[1] =
        (-config->geometry.l1 *
         sinf(leg->phi1 - leg->phi2) *
         cosf(leg->phi0 - leg->phi3) /
         (leg->L0 * denominator));
    J[2] =
        (-config->geometry.l4 *
         sinf(leg->phi0 - leg->phi2) *
         sinf(leg->phi3 - leg->phi4) / denominator);
    J[3] =
        (-config->geometry.l4 *
         sinf(leg->phi3 - leg->phi4) *
         cosf(leg->phi0 - leg->phi2) /
         (leg->L0 * denominator));

    if ((!isfinite(J[0])) || (!isfinite(J[1])) ||
        (!isfinite(J[2])) || (!isfinite(J[3])))
    {
        memset(J, 0, sizeof(float) * 4U);
        return 0U;
    }
    return 1U;
}

/**
 * @brief 使用虚功关系把虚拟腿广义力映射为两个主动关节力矩。
 */
uint8_t VMC_Torque_Calc(const Chassis_Leg_Config_t *config,
                        const Chassis_Leg_t *leg,
                        float F0,
                        float Tp,
                        VMC_Torque_t *torque)
{
    float J[4];
    float T1_geometric;
    float T4_geometric;
    float phi1_scale;
    float phi4_scale;

    if (torque == NULL)
    {
        return 0U;
    }
    memset(torque, 0, sizeof(*torque));
    if ((config == NULL) || (!isfinite(F0)) || (!isfinite(Tp)) ||
        (VMC_Matrix_Calc(config, leg, J) == 0U))
    {
        return 0U;
    }
    phi1_scale = config->joint[CHASSIS_JOINT_PHI1].scale *
                 config->joint[CHASSIS_JOINT_PHI1].ratio;
    phi4_scale = config->joint[CHASSIS_JOINT_PHI4].scale *
                 config->joint[CHASSIS_JOINT_PHI4].ratio;
    if ((!isfinite(config->joint[CHASSIS_JOINT_PHI1].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI1].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(phi1_scale)) ||
        (fabsf(phi1_scale) <= VMC_SCALE_EPS) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI4].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI4].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(phi4_scale)) ||
        (fabsf(phi4_scale) <= VMC_SCALE_EPS))
    {
        return 0U;
    }

    T1_geometric = J[0] * F0 + J[1] * Tp;
    T4_geometric = J[2] * F0 + J[3] * Tp;

    if ((!isfinite(T1_geometric)) || (!isfinite(T4_geometric)))
    {
        return 0U;
    }

    torque->T1 = T1_geometric * phi1_scale;
    torque->T4 = T4_geometric * phi4_scale;
    if ((!isfinite(torque->T1)) || (!isfinite(torque->T4)))
    {
        memset(torque, 0, sizeof(*torque));
        return 0U;
    }
    return 1U;
}

uint8_t VMC_Force_Calc(const Chassis_Leg_Config_t *config,
                       const Chassis_Leg_t *leg,
                       float phi1_motor_torque_nm,
                       float phi4_motor_torque_nm,
                       VMC_Force_t *force)
{
    float J[4];
    float det;
    float T1_geometric;
    float T4_geometric;
    float phi1_scale;
    float phi4_scale;

    if (force == NULL)
    {
        return 0U;
    }
    memset(force, 0, sizeof(*force));
    if ((config == NULL) || (!isfinite(phi1_motor_torque_nm)) ||
        (!isfinite(phi4_motor_torque_nm)) ||
        (VMC_Matrix_Calc(config, leg, J) == 0U))
    {
        return 0U;
    }

    phi1_scale = config->joint[CHASSIS_JOINT_PHI1].scale *
                 config->joint[CHASSIS_JOINT_PHI1].ratio;
    phi4_scale = config->joint[CHASSIS_JOINT_PHI4].scale *
                 config->joint[CHASSIS_JOINT_PHI4].ratio;
    if ((!isfinite(config->joint[CHASSIS_JOINT_PHI1].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI1].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(config->joint[CHASSIS_JOINT_PHI4].ratio)) ||
        (config->joint[CHASSIS_JOINT_PHI4].ratio <= VMC_SCALE_EPS) ||
        (!isfinite(phi1_scale)) || (!isfinite(phi4_scale)) ||
        (fabsf(phi1_scale) <= VMC_SCALE_EPS) ||
        (fabsf(phi4_scale) <= VMC_SCALE_EPS))
    {
        return 0U;
    }

    det = J[0] * J[3] - J[1] * J[2];
    if ((!isfinite(det)) || (fabsf(det) <= VMC_DET_EPS))
    {
        return 0U;
    }

    T1_geometric = phi1_motor_torque_nm / phi1_scale;
    T4_geometric = phi4_motor_torque_nm / phi4_scale;
    force->F0 =
        (J[3] * T1_geometric - J[1] * T4_geometric) / det;
    force->Tp =
        (-J[2] * T1_geometric + J[0] * T4_geometric) / det;
    if ((!isfinite(force->F0)) ||
        (!isfinite(force->Tp)))
    {
        memset(force, 0, sizeof(*force));
        return 0U;
    }
    return 1U;
}
