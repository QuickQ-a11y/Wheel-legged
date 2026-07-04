#include "module_chassis_leg.h"

#include <math.h>
#include <string.h>

#define MODULE_CHASSIS_LEG_EPSILON 1.0e-6f

typedef struct
{
    float x;
    float y;
} module_chassis_leg_vector_t;

/**
 * @brief 根据前后主动杆角度计算五连杆几何状态。
 *
 * B、D 是两根主动杆末端点，C 是两根从动杆的交点。
 * 求解失败说明当前关节角不满足五连杆闭链几何，控制器应进入安全输出。
 */
static void Module_Chassis_Leg_CalculateForwardGeometry(
    const module_chassis_leg_geometry_config_t *geometry,
    float phi1Rad,
    float phi4Rad,
    module_chassis_leg_state_t *state,
    module_chassis_leg_vector_t *pointB,
    module_chassis_leg_vector_t *pointC,
    module_chassis_leg_vector_t *pointD)
{
    float pointBDx;
    float pointBDy;
    float pointBDLength;
    float equationA;
    float equationB;
    float equationC;
    float sqrtArgument;
    float sqrtValue;

    /* 先由两个主动杆角度确定 B、D 点，坐标原点取左右固定铰点中点。 */
    pointB->x = (-geometry->frameJointDistanceM * 0.5f) +
                (geometry->link1LengthM * cosf(phi1Rad));
    pointB->y = geometry->link1LengthM * sinf(phi1Rad);
    pointD->x = (geometry->frameJointDistanceM * 0.5f) +
                (geometry->link4LengthM * cosf(phi4Rad));
    pointD->y = geometry->link4LengthM * sinf(phi4Rad);

    pointBDx = pointD->x - pointB->x;
    pointBDy = pointD->y - pointB->y;
    pointBDLength = sqrtf((pointBDx * pointBDx) + (pointBDy * pointBDy));
    if (pointBDLength <= MODULE_CHASSIS_LEG_EPSILON)
    {
        memset(state, 0, sizeof(*state));
        return;
    }

    /*
     * C 点由以 B、D 为圆心的两圆相交得到。
     * sqrtArgument 小于 0 表示两圆无交点；接近 0 时按切点处理。
     */
    equationA = 2.0f * geometry->link2LengthM * pointBDx;
    equationB = 2.0f * geometry->link2LengthM * pointBDy;
    equationC = (geometry->link2LengthM * geometry->link2LengthM) +
                (pointBDLength * pointBDLength) -
                (geometry->link3LengthM * geometry->link3LengthM);
    sqrtArgument = (equationA * equationA) + (equationB * equationB) -
                   (equationC * equationC);
    if (sqrtArgument < -MODULE_CHASSIS_LEG_EPSILON)
    {
        memset(state, 0, sizeof(*state));
        return;
    }
    if (sqrtArgument < 0.0f)
    {
        sqrtArgument = 0.0f;
    }

    sqrtValue = sqrtf(sqrtArgument);
    /* 当前机构只选用与实机装配一致的交点分支，不在运行时切换构型。 */
    state->phi2Rad = 2.0f * atan2f(equationB + sqrtValue,
                                   equationA + equationC);
    pointC->x = (-geometry->frameJointDistanceM * 0.5f) +
                (geometry->link1LengthM * cosf(phi1Rad)) +
                (geometry->link2LengthM * cosf(state->phi2Rad));
    pointC->y = (geometry->link1LengthM * sinf(phi1Rad)) +
                (geometry->link2LengthM * sinf(state->phi2Rad));

    /* C 点确定后可得到后从动杆角度、虚拟腿长和虚拟腿几何角。 */
    state->phi3Rad = atan2f(pointC->y - pointD->y,
                            pointC->x - pointD->x);
    state->legLengthM = sqrtf((pointC->x * pointC->x) + (pointC->y * pointC->y));
    if ((state->legLengthM <= geometry->minLegLengthM) ||
        (state->legLengthM <= MODULE_CHASSIS_LEG_EPSILON))
    {
        memset(state, 0, sizeof(*state));
        return;
    }

    state->phi0Rad = atan2f(pointC->y, pointC->x);
}

/**
 * @brief 根据关节角速度计算虚拟腿长速度和虚拟腿摆角速度。
 *
 * 速度计算使用闭链约束的一阶导数，先解出从动杆角速度，再得到 C 点速度。
 */
static void Module_Chassis_Leg_CalculateVelocity(
    const module_chassis_leg_geometry_config_t *geometry,
    const module_chassis_leg_state_t *state,
    const module_chassis_leg_vector_t *pointB,
    const module_chassis_leg_vector_t *pointC,
    const module_chassis_leg_vector_t *pointD,
    float phi1VelocityRadps,
    float phi4VelocityRadps,
    module_chassis_leg_state_t *outputState)
{
    module_chassis_leg_vector_t pointBVelocity;
    module_chassis_leg_vector_t pointDVelocity;
    module_chassis_leg_vector_t pointCVelocity;
    float matrix11;
    float matrix12;
    float matrix21;
    float matrix22;
    float determinant;
    float rhsX;
    float rhsY;
    float phi2VelocityRadps;
    float legLengthSquared;

    (void)pointB;
    (void)pointD;

    /* 主动杆末端速度由当前角度和主动杆角速度直接得到。 */
    pointBVelocity.x = -geometry->link1LengthM * sinf(state->phi1Rad) *
                       phi1VelocityRadps;
    pointBVelocity.y = geometry->link1LengthM * cosf(state->phi1Rad) *
                       phi1VelocityRadps;
    pointDVelocity.x = -geometry->link4LengthM * sinf(state->phi4Rad) *
                       phi4VelocityRadps;
    pointDVelocity.y = geometry->link4LengthM * cosf(state->phi4Rad) *
                       phi4VelocityRadps;

    /*
     * 对 B-C-D 闭链约束求导，形成 2x2 线性方程。
     * determinant 接近 0 表示当前姿态接近奇异位，速度解不可靠。
     */
    matrix11 = -geometry->link2LengthM * sinf(state->phi2Rad);
    matrix21 = geometry->link2LengthM * cosf(state->phi2Rad);
    matrix12 = geometry->link3LengthM * sinf(state->phi3Rad);
    matrix22 = -geometry->link3LengthM * cosf(state->phi3Rad);
    determinant = (matrix11 * matrix22) - (matrix12 * matrix21);
    if (fabsf(determinant) <= MODULE_CHASSIS_LEG_EPSILON)
    {
        memset(outputState, 0, sizeof(*outputState));
        return;
    }

    rhsX = pointDVelocity.x - pointBVelocity.x;
    rhsY = pointDVelocity.y - pointBVelocity.y;
    phi2VelocityRadps = ((rhsX * matrix22) - (matrix12 * rhsY)) / determinant;

    /* C 点速度用于投影出腿长速度和腿摆角速度。 */
    pointCVelocity.x = pointBVelocity.x +
                       (matrix11 * phi2VelocityRadps);
    pointCVelocity.y = pointBVelocity.y +
                       (matrix21 * phi2VelocityRadps);

    outputState->legLengthVelocityMps =
        ((pointC->x * pointCVelocity.x) + (pointC->y * pointCVelocity.y)) /
        outputState->legLengthM;

    legLengthSquared = outputState->legLengthM * outputState->legLengthM;
    if (fabsf(legLengthSquared) <= MODULE_CHASSIS_LEG_EPSILON)
    {
        memset(outputState, 0, sizeof(*outputState));
        return;
    }

    /*
     * 这里保存 d(phi0)/dt，控制器按 SPR 的 theta 定义再组合机体俯仰角速度。
     */
    outputState->legSwingVelocityRadps =
        ((pointC->x * pointCVelocity.y) - (pointC->y * pointCVelocity.x)) /
        legLengthSquared;
}

void Module_Chassis_Leg_CalculateState(const module_chassis_leg_config_t *config,
                                       float frontPositionRad,
                                       float backPositionRad,
                                       float frontVelocityRadps,
                                       float backVelocityRadps,
                                       module_chassis_leg_state_t *state)
{
    module_chassis_leg_vector_t pointB = {0.0f, 0.0f};
    module_chassis_leg_vector_t pointC = {0.0f, 0.0f};
    module_chassis_leg_vector_t pointD = {0.0f, 0.0f};
    float phi1VelocityRadps;
    float phi4VelocityRadps;

    /* 电机反馈角先按模型配置转换成五连杆几何角，方向和零位都不在控制器里修正。 */
    memset(state, 0, sizeof(*state));
    state->phi1Rad = config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleOffsetRad +
                     (config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleScale *
                      frontPositionRad);
    state->phi4Rad = config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleOffsetRad +
                     (config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleScale *
                      backPositionRad);

    Module_Chassis_Leg_CalculateForwardGeometry(&config->geometry,
                                                state->phi1Rad,
                                                state->phi4Rad,
                                                state,
                                                &pointB,
                                                &pointC,
                                                &pointD);
    if (state->legLengthM <= MODULE_CHASSIS_LEG_EPSILON)
    {
        return;
    }

    /* 角速度同样只在几何层做方向映射，保证状态量单位统一为 rad/s。 */
    phi1VelocityRadps =
        config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleScale *
        frontVelocityRadps;
    phi4VelocityRadps =
        config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleScale *
        backVelocityRadps;

    Module_Chassis_Leg_CalculateVelocity(&config->geometry,
                                         state,
                                         &pointB,
                                         &pointC,
                                         &pointD,
                                         phi1VelocityRadps,
                                         phi4VelocityRadps,
                                         state);
}

void Module_Chassis_Leg_MapVirtualForce(const module_chassis_leg_config_t *config,
                                        const module_chassis_leg_state_t *state,
                                        float supportForceN,
                                        float swingTorqueNm,
                                        module_chassis_leg_joint_torque_t *jointTorque)
{
    float denominator;
    float frontTorqueByGeometry;
    float backTorqueByGeometry;

    memset(jointTorque, 0, sizeof(*jointTorque));

    /*
     * denominator 来自五连杆雅可比矩阵。
     * 接近 0 表示力到关节力矩的映射奇异，继续输出会放大力矩命令。
     */
    denominator = sinf(state->phi2Rad - state->phi3Rad);
    if ((fabsf(denominator) <= MODULE_CHASSIS_LEG_EPSILON) ||
        (fabsf(state->legLengthM) <= MODULE_CHASSIS_LEG_EPSILON))
    {
        return;
    }

    /*
     * swingTorqueNm 是虚拟腿摆广义力矩，正方向跟随 SPR 状态定义。
     * 最终电机方向仍需要通过 torqueScale 和实机低力矩测试确认。
     */
    /* 前髋关节力矩由虚拟支撑力分量和虚拟腿摆力矩分量叠加得到。 */
    frontTorqueByGeometry =
        (-config->geometry.link1LengthM *
         sinf(state->phi0Rad - state->phi3Rad) *
         sinf(state->phi1Rad - state->phi2Rad) / denominator *
         supportForceN) +
        (-config->geometry.link1LengthM *
         sinf(state->phi1Rad - state->phi2Rad) *
         cosf(state->phi0Rad - state->phi3Rad) /
         (state->legLengthM * denominator) *
         swingTorqueNm);

    /* 后髋关节使用同一套 VMC 映射，最终正负号仍由 torqueScale 处理安装方向。 */
    backTorqueByGeometry =
        (-config->geometry.link4LengthM *
         sinf(state->phi0Rad - state->phi2Rad) *
         sinf(state->phi3Rad - state->phi4Rad) / denominator *
         supportForceN) +
        (-config->geometry.link4LengthM *
         sinf(state->phi3Rad - state->phi4Rad) *
         cosf(state->phi0Rad - state->phi2Rad) /
         (state->legLengthM * denominator) *
         swingTorqueNm);

    jointTorque->frontTorqueNm =
        frontTorqueByGeometry *
        config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].torqueScale;
    jointTorque->backTorqueNm =
        backTorqueByGeometry *
        config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].torqueScale;
}
