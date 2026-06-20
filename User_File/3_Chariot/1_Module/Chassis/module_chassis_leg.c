#include "module_chassis_leg.h"

#include <math.h>
#include <string.h>

#define MODULE_CHASSIS_LEG_EPSILON 1.0e-6f

typedef struct
{
    float x;
    float y;
} module_chassis_leg_vector_t;

static uint8_t Module_Chassis_Leg_IsValidDenominator(float value)
{
    return (fabsf(value) > MODULE_CHASSIS_LEG_EPSILON) ? 1U : 0U;
}

static app_status_t Module_Chassis_Leg_CalculateForwardGeometry(
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

    if ((geometry == NULL) || (state == NULL) || (pointB == NULL) ||
        (pointC == NULL) || (pointD == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

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
        return APP_STATUS_ERROR;
    }

    equationA = 2.0f * geometry->link2LengthM * pointBDx;
    equationB = 2.0f * geometry->link2LengthM * pointBDy;
    equationC = (geometry->link2LengthM * geometry->link2LengthM) +
                (pointBDLength * pointBDLength) -
                (geometry->link3LengthM * geometry->link3LengthM);
    sqrtArgument = (equationA * equationA) + (equationB * equationB) -
                   (equationC * equationC);
    if (sqrtArgument < -MODULE_CHASSIS_LEG_EPSILON)
    {
        return APP_STATUS_ERROR;
    }
    if (sqrtArgument < 0.0f)
    {
        sqrtArgument = 0.0f;
    }

    sqrtValue = sqrtf(sqrtArgument);
    state->phi2Rad = 2.0f * atan2f(equationB + sqrtValue,
                                   equationA + equationC);
    pointC->x = (-geometry->frameJointDistanceM * 0.5f) +
                (geometry->link1LengthM * cosf(phi1Rad)) +
                (geometry->link2LengthM * cosf(state->phi2Rad));
    pointC->y = (geometry->link1LengthM * sinf(phi1Rad)) +
                (geometry->link2LengthM * sinf(state->phi2Rad));

    state->phi3Rad = atan2f(pointC->y - pointD->y,
                            pointC->x - pointD->x);
    state->legLengthM = sqrtf((pointC->x * pointC->x) + (pointC->y * pointC->y));
    if ((state->legLengthM <= geometry->minLegLengthM) ||
        (state->legLengthM <= MODULE_CHASSIS_LEG_EPSILON))
    {
        return APP_STATUS_ERROR;
    }

    state->phi0Rad = atan2f(pointC->y, pointC->x);

    return APP_STATUS_OK;
}

static app_status_t Module_Chassis_Leg_CalculateVelocity(
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

    if ((geometry == NULL) || (state == NULL) || (pointB == NULL) ||
        (pointC == NULL) || (pointD == NULL) || (outputState == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    (void)pointB;
    (void)pointD;

    pointBVelocity.x = -geometry->link1LengthM * sinf(state->phi1Rad) *
                       phi1VelocityRadps;
    pointBVelocity.y = geometry->link1LengthM * cosf(state->phi1Rad) *
                       phi1VelocityRadps;
    pointDVelocity.x = -geometry->link4LengthM * sinf(state->phi4Rad) *
                       phi4VelocityRadps;
    pointDVelocity.y = geometry->link4LengthM * cosf(state->phi4Rad) *
                       phi4VelocityRadps;

    matrix11 = -geometry->link2LengthM * sinf(state->phi2Rad);
    matrix21 = geometry->link2LengthM * cosf(state->phi2Rad);
    matrix12 = geometry->link3LengthM * sinf(state->phi3Rad);
    matrix22 = -geometry->link3LengthM * cosf(state->phi3Rad);
    determinant = (matrix11 * matrix22) - (matrix12 * matrix21);
    if (Module_Chassis_Leg_IsValidDenominator(determinant) == 0U)
    {
        return APP_STATUS_ERROR;
    }

    rhsX = pointDVelocity.x - pointBVelocity.x;
    rhsY = pointDVelocity.y - pointBVelocity.y;
    phi2VelocityRadps = ((rhsX * matrix22) - (matrix12 * rhsY)) / determinant;

    pointCVelocity.x = pointBVelocity.x +
                       (matrix11 * phi2VelocityRadps);
    pointCVelocity.y = pointBVelocity.y +
                       (matrix21 * phi2VelocityRadps);

    outputState->legLengthVelocityMps =
        ((pointC->x * pointCVelocity.x) + (pointC->y * pointCVelocity.y)) /
        outputState->legLengthM;

    legLengthSquared = outputState->legLengthM * outputState->legLengthM;
    if (Module_Chassis_Leg_IsValidDenominator(legLengthSquared) == 0U)
    {
        return APP_STATUS_ERROR;
    }

    /*
     * 控制状态使用的是“竖直参考角 - phi0”，因此腿摆速度为 -d(phi0)/dt。
     */
    outputState->legSwingVelocityRadps =
        -(((pointC->x * pointCVelocity.y) - (pointC->y * pointCVelocity.x)) /
          legLengthSquared);

    return APP_STATUS_OK;
}

app_status_t Module_Chassis_Leg_CalculateState(const module_chassis_leg_config_t *config,
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
    app_status_t status;

    if ((config == NULL) || (state == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    memset(state, 0, sizeof(*state));
    state->phi1Rad = config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleOffsetRad +
                     (config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleScale *
                      frontPositionRad);
    state->phi4Rad = config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleOffsetRad +
                     (config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleScale *
                      backPositionRad);

    status = Module_Chassis_Leg_CalculateForwardGeometry(&config->geometry,
                                                         state->phi1Rad,
                                                         state->phi4Rad,
                                                         state,
                                                         &pointB,
                                                         &pointC,
                                                         &pointD);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    phi1VelocityRadps =
        config->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].angleScale *
        frontVelocityRadps;
    phi4VelocityRadps =
        config->joints[MODULE_CHASSIS_LEG_JOINT_BACK].angleScale *
        backVelocityRadps;

    return Module_Chassis_Leg_CalculateVelocity(&config->geometry,
                                                state,
                                                &pointB,
                                                &pointC,
                                                &pointD,
                                                phi1VelocityRadps,
                                                phi4VelocityRadps,
                                                state);
}

app_status_t Module_Chassis_Leg_MapVirtualForce(const module_chassis_leg_config_t *config,
                                                const module_chassis_leg_state_t *state,
                                                float supportForceN,
                                                float swingTorqueNm,
                                                module_chassis_leg_joint_torque_t *jointTorque)
{
    float denominator;
    float frontTorqueByGeometry;
    float backTorqueByGeometry;

    if ((config == NULL) || (state == NULL) || (jointTorque == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    denominator = sinf(state->phi2Rad - state->phi3Rad);
    if ((Module_Chassis_Leg_IsValidDenominator(denominator) == 0U) ||
        (Module_Chassis_Leg_IsValidDenominator(state->legLengthM) == 0U))
    {
        return APP_STATUS_ERROR;
    }

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

    return APP_STATUS_OK;
}
