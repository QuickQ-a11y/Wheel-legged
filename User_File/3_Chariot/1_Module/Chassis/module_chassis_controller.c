#include "module_chassis_controller.h"

#include "algorithm_pid.h"

#include <math.h>
#include <string.h>

#define MODULE_CHASSIS_RPM_TO_RADPS 0.10471975512f

typedef struct
{
    algorithm_pid_state_t legLengthPid[MODULE_CHASSIS_LEG_COUNT];
    algorithm_pid_state_t rollPid;
    module_chassis_controller_debug_t debug;
} module_chassis_controller_state_t;

static module_chassis_controller_state_t chassisControllerState;

static float Module_Chassis_Controller_LimitFloat(float value,
                                                 float minValue,
                                                 float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static float Module_Chassis_Controller_LimitSymmetric(float value, float limit)
{
    float positiveLimit = fabsf(limit);

    if (positiveLimit <= 0.0f)
    {
        return 0.0f;
    }

    return Module_Chassis_Controller_LimitFloat(value,
                                                -positiveLimit,
                                                positiveLimit);
}

static int16_t Module_Chassis_Controller_LimitInt16(float value, int16_t limit)
{
    float limitedValue = Module_Chassis_Controller_LimitSymmetric(value, (float)limit);

    return (int16_t)limitedValue;
}

static uint8_t Module_Chassis_Controller_IsConfigValid(
    const module_chassis_model_config_t *config)
{
    uint32_t sideIndex;
    uint32_t jointIndex;

    if (config == NULL)
    {
        return 0U;
    }

    if ((config->imu.bodyPitchRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.rollRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.yawRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT))
    {
        return 0U;
    }

    for (sideIndex = 0U; sideIndex < MODULE_CHASSIS_LEG_COUNT; sideIndex++)
    {
        const module_chassis_leg_geometry_config_t *geometry =
            &config->legs[sideIndex].geometry;

        if ((geometry->link1LengthM <= 0.0f) ||
            (geometry->link2LengthM <= 0.0f) ||
            (geometry->link3LengthM <= 0.0f) ||
            (geometry->link4LengthM <= 0.0f) ||
            (geometry->minLegLengthM <= 0.0f))
        {
            return 0U;
        }

        for (jointIndex = 0U; jointIndex < MODULE_CHASSIS_LEG_JOINT_COUNT; jointIndex++)
        {
            if (config->legs[sideIndex].joints[jointIndex].motorIndex >=
                APP_CONFIG_DM_MOTOR_COUNT)
            {
                return 0U;
            }
        }
    }

    return 1U;
}

static float Module_Chassis_Controller_GetDtSec(const module_chassis_model_config_t *config,
                                                const module_chassis_input_t *input)
{
    float dtSec;

    if ((config == NULL) || (input == NULL))
    {
        return APP_CONFIG_IMU_DEFAULT_DT_SEC;
    }

    dtSec = input->imu.dtSec;
    if ((dtSec < config->minDtSec) || (dtSec > config->maxDtSec))
    {
        dtSec = config->defaultDtSec;
    }

    if (dtSec <= 0.0f)
    {
        dtSec = APP_CONFIG_IMU_DEFAULT_DT_SEC;
    }

    return dtSec;
}

static app_status_t Module_Chassis_Controller_GetJointState(
    const module_chassis_input_t *input,
    const module_chassis_leg_config_t *legConfig,
    float jointPositionRad[MODULE_CHASSIS_LEG_JOINT_COUNT],
    float jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_COUNT])
{
    uint32_t jointIndex;

    if ((input == NULL) || (legConfig == NULL) ||
        (jointPositionRad == NULL) || (jointVelocityRadps == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    for (jointIndex = 0U; jointIndex < MODULE_CHASSIS_LEG_JOINT_COUNT; jointIndex++)
    {
        uint8_t motorIndex = legConfig->joints[jointIndex].motorIndex;

        if (motorIndex >= APP_CONFIG_DM_MOTOR_COUNT)
        {
            return APP_STATUS_INVALID_PARAM;
        }

        jointPositionRad[jointIndex] = input->dmMotors[motorIndex].positionRad;
        jointVelocityRadps[jointIndex] = input->dmMotors[motorIndex].velocityRadps;
    }

    return APP_STATUS_OK;
}

static app_status_t Module_Chassis_Controller_UpdateLegStates(
    const module_chassis_model_config_t *config,
    const module_chassis_input_t *input,
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT])
{
    uint32_t sideIndex;

    if ((config == NULL) || (input == NULL) || (legStates == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    for (sideIndex = 0U; sideIndex < MODULE_CHASSIS_LEG_COUNT; sideIndex++)
    {
        float jointPositionRad[MODULE_CHASSIS_LEG_JOINT_COUNT] = {0.0f};
        float jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_COUNT] = {0.0f};
        app_status_t status;

        status = Module_Chassis_Controller_GetJointState(input,
                                                         &config->legs[sideIndex],
                                                         jointPositionRad,
                                                         jointVelocityRadps);
        if (status != APP_STATUS_OK)
        {
            return status;
        }

        status = Module_Chassis_Leg_CalculateState(
            &config->legs[sideIndex],
            jointPositionRad[MODULE_CHASSIS_LEG_JOINT_FRONT],
            jointPositionRad[MODULE_CHASSIS_LEG_JOINT_BACK],
            jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_FRONT],
            jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_BACK],
            &legStates[sideIndex]);
        if (status != APP_STATUS_OK)
        {
            return status;
        }
    }

    return APP_STATUS_OK;
}

static void Module_Chassis_Controller_BuildState(
    const module_chassis_model_config_t *config,
    const module_chassis_input_t *input,
    const module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT],
    float state[MODULE_CHASSIS_CONTROL_STATE_COUNT],
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT],
    float *forwardVelocityMps)
{
    float leftWheelVelocityRadps;
    float rightWheelVelocityRadps;
    float bodyPitchRad;
    float bodyPitchRateRadps;
    float leftLegAngleRad;
    float rightLegAngleRad;
    float leftLegAngleRateRadps;
    float rightLegAngleRateRadps;

    memset(state, 0, sizeof(float) * MODULE_CHASSIS_CONTROL_STATE_COUNT);

    leftWheelVelocityRadps =
        (float)input->djiWheels[0].speedRpm *
        MODULE_CHASSIS_RPM_TO_RADPS *
        config->wheel.leftVelocityScale;
    rightWheelVelocityRadps =
        (float)input->djiWheels[1].speedRpm *
        MODULE_CHASSIS_RPM_TO_RADPS *
        config->wheel.rightVelocityScale;

    wheelAngularVelocityRadps[0] = leftWheelVelocityRadps;
    wheelAngularVelocityRadps[1] = rightWheelVelocityRadps;

    bodyPitchRad = input->imu.pitchRad * config->imu.bodyPitchAngleScale;
    bodyPitchRateRadps =
        input->imu.gyroRadps[config->imu.bodyPitchRateGyroIndex] *
        config->imu.bodyPitchRateScale;

    leftLegAngleRad = config->legVerticalAngleOffsetRad -
                      legStates[MODULE_CHASSIS_LEG_LEFT].phi0Rad +
                      bodyPitchRad;
    rightLegAngleRad = config->legVerticalAngleOffsetRad -
                       legStates[MODULE_CHASSIS_LEG_RIGHT].phi0Rad +
                       bodyPitchRad;
    leftLegAngleRateRadps =
        legStates[MODULE_CHASSIS_LEG_LEFT].legSwingVelocityRadps +
        bodyPitchRateRadps;
    rightLegAngleRateRadps =
        legStates[MODULE_CHASSIS_LEG_RIGHT].legSwingVelocityRadps +
        bodyPitchRateRadps;

    *forwardVelocityMps =
        (config->wheel.radiusM * (leftWheelVelocityRadps + rightWheelVelocityRadps) *
         0.5f) +
        (0.5f * ((legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM *
                  leftLegAngleRateRadps * cosf(leftLegAngleRad)) +
                 (legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM *
                  rightLegAngleRateRadps * cosf(rightLegAngleRad)))) +
        (0.5f * ((legStates[MODULE_CHASSIS_LEG_LEFT].legLengthVelocityMps *
                  sinf(leftLegAngleRad)) +
                 (legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthVelocityMps *
                  sinf(rightLegAngleRad))));

    state[MODULE_CHASSIS_STATE_FORWARD_POSITION] = 0.0f;
    state[MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = *forwardVelocityMps;
    state[MODULE_CHASSIS_STATE_YAW] = input->imu.yawRad * config->imu.yawAngleScale;
    state[MODULE_CHASSIS_STATE_YAW_RATE] =
        input->imu.gyroRadps[config->imu.yawRateGyroIndex] * config->imu.yawRateScale;
    state[MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = leftLegAngleRad;
    state[MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = leftLegAngleRateRadps;
    state[MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = rightLegAngleRad;
    state[MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = rightLegAngleRateRadps;
    state[MODULE_CHASSIS_STATE_BODY_PITCH] = bodyPitchRad;
    state[MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = bodyPitchRateRadps;
}

static void Module_Chassis_Controller_CalculateMotionOutput(
    const module_chassis_model_config_t *config,
    const float state[MODULE_CHASSIS_CONTROL_STATE_COUNT],
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT])
{
    uint32_t outputIndex;
    uint32_t stateIndex;

    for (outputIndex = 0U; outputIndex < MODULE_CHASSIS_CONTROL_OUTPUT_COUNT; outputIndex++)
    {
        float outputValue = 0.0f;

        for (stateIndex = 0U; stateIndex < MODULE_CHASSIS_CONTROL_STATE_COUNT; stateIndex++)
        {
            outputValue += config->motionGain[outputIndex][stateIndex] *
                           (config->targetState[stateIndex] - state[stateIndex]);
        }

        motionOutput[outputIndex] = outputValue;
    }
}

static app_status_t Module_Chassis_Controller_CalculateSupportForces(
    const module_chassis_model_config_t *config,
    const module_chassis_input_t *input,
    const module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT],
    float dtSec,
    float supportForcesN[MODULE_CHASSIS_LEG_COUNT])
{
    float rollRad;
    float rollRateRadps;
    float rollCorrectionN;
    float leftLengthCorrectionN;
    float rightLengthCorrectionN;
    float leftPidOutput = 0.0f;
    float rightPidOutput = 0.0f;
    float rollPidOutput = 0.0f;
    app_status_t status;

    rollRad = input->imu.rollRad * config->imu.rollAngleScale;
    rollRateRadps =
        input->imu.gyroRadps[config->imu.rollRateGyroIndex] *
        config->imu.rollRateScale;

    status = Algorithm_PID_UpdateByFeedbackRate(
        &config->legLengthPid,
        &chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_LEFT],
        config->legs[MODULE_CHASSIS_LEG_LEFT].targetLegLengthM,
        legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM,
        legStates[MODULE_CHASSIS_LEG_LEFT].legLengthVelocityMps,
        dtSec,
        &leftPidOutput);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    status = Algorithm_PID_UpdateByFeedbackRate(
        &config->legLengthPid,
        &chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_RIGHT],
        config->legs[MODULE_CHASSIS_LEG_RIGHT].targetLegLengthM,
        legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM,
        legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthVelocityMps,
        dtSec,
        &rightPidOutput);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    status = Algorithm_PID_UpdateByFeedbackRate(&config->rollPid,
                                                &chassisControllerState.rollPid,
                                                config->targetRollRad,
                                                rollRad,
                                                rollRateRadps,
                                                dtSec,
                                                &rollPidOutput);
    if (status != APP_STATUS_OK)
    {
        return status;
    }

    leftLengthCorrectionN = -leftPidOutput;
    rightLengthCorrectionN = -rightPidOutput;
    rollCorrectionN = -rollPidOutput;

    supportForcesN[MODULE_CHASSIS_LEG_LEFT] =
        -rollCorrectionN +
        leftLengthCorrectionN +
        config->baseSupportForceN +
        config->leftSupportForceFeedforwardN;
    supportForcesN[MODULE_CHASSIS_LEG_RIGHT] =
        rollCorrectionN +
        rightLengthCorrectionN +
        config->baseSupportForceN -
        config->rightSupportForceFeedforwardN;

    return APP_STATUS_OK;
}

static void Module_Chassis_Controller_ApplyJointTorques(
    const module_chassis_model_config_t *config,
    const module_chassis_leg_joint_torque_t jointTorques[MODULE_CHASSIS_LEG_COUNT],
    module_chassis_output_t *output)
{
    uint32_t sideIndex;

    if ((config->output.jointTorqueOutputEnabled == 0U) ||
        (APP_CONFIG_CHASSIS_CONTROLLER_OUTPUT_ENABLE == 0U) ||
        (config->output.jointTorqueLimitNm <= 0.0f))
    {
        return;
    }

    for (sideIndex = 0U; sideIndex < MODULE_CHASSIS_LEG_COUNT; sideIndex++)
    {
        const module_chassis_leg_config_t *legConfig = &config->legs[sideIndex];
        uint8_t frontIndex = legConfig->joints[MODULE_CHASSIS_LEG_JOINT_FRONT].motorIndex;
        uint8_t backIndex = legConfig->joints[MODULE_CHASSIS_LEG_JOINT_BACK].motorIndex;

        if ((frontIndex >= APP_CONFIG_DM_MOTOR_COUNT) ||
            (backIndex >= APP_CONFIG_DM_MOTOR_COUNT))
        {
            continue;
        }

        output->dmCommands[frontIndex].torqueNm =
            Module_Chassis_Controller_LimitSymmetric(jointTorques[sideIndex].frontTorqueNm,
                                                     config->output.jointTorqueLimitNm);
        output->dmCommands[backIndex].torqueNm =
            Module_Chassis_Controller_LimitSymmetric(jointTorques[sideIndex].backTorqueNm,
                                                     config->output.jointTorqueLimitNm);
    }
}

static void Module_Chassis_Controller_ApplyWheelCurrents(
    const module_chassis_model_config_t *config,
    const float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT],
    module_chassis_output_t *output)
{
    if ((config->output.wheelCurrentOutputEnabled == 0U) ||
        (APP_CONFIG_CHASSIS_CONTROLLER_OUTPUT_ENABLE == 0U) ||
        (config->wheel.torqueLimitNm <= 0.0f) ||
        (config->wheel.torqueToCurrentRaw == 0.0f) ||
        (config->wheel.currentLimitRaw <= 0))
    {
        return;
    }

    output->djiCurrents[0] = Module_Chassis_Controller_LimitInt16(
        Module_Chassis_Controller_LimitSymmetric(
            motionOutput[MODULE_CHASSIS_CONTROL_LEFT_WHEEL_TORQUE],
            config->wheel.torqueLimitNm) *
            config->wheel.torqueToCurrentRaw,
        config->wheel.currentLimitRaw);
    output->djiCurrents[1] = Module_Chassis_Controller_LimitInt16(
        Module_Chassis_Controller_LimitSymmetric(
            motionOutput[MODULE_CHASSIS_CONTROL_RIGHT_WHEEL_TORQUE],
            config->wheel.torqueLimitNm) *
            config->wheel.torqueToCurrentRaw,
        config->wheel.currentLimitRaw);
}

void Module_Chassis_Controller_Init(const module_chassis_model_config_t *config)
{
    (void)config;
    memset(&chassisControllerState, 0, sizeof(chassisControllerState));
    (void)Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_LEFT]);
    (void)Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_RIGHT]);
    (void)Algorithm_PID_Init(&chassisControllerState.rollPid);
}

app_status_t Module_Chassis_Controller_Update(const module_chassis_model_config_t *config,
                                              const module_chassis_input_t *input,
                                              module_chassis_output_t *output)
{
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT];
    module_chassis_leg_joint_torque_t jointTorques[MODULE_CHASSIS_LEG_COUNT];
    float controlState[MODULE_CHASSIS_CONTROL_STATE_COUNT] = {0.0f};
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT] = {0.0f};
    float supportForcesN[MODULE_CHASSIS_LEG_COUNT] = {0.0f};
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT] = {0.0f};
    float forwardVelocityMps = 0.0f;
    float dtSec;
    app_status_t status;

    if ((config == NULL) || (input == NULL) || (output == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    if (Module_Chassis_Controller_IsConfigValid(config) == 0U)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return APP_STATUS_INVALID_PARAM;
    }

    memset(legStates, 0, sizeof(legStates));
    memset(jointTorques, 0, sizeof(jointTorques));

    status = Module_Chassis_Controller_UpdateLegStates(config, input, legStates);
    if (status != APP_STATUS_OK)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return status;
    }

    dtSec = Module_Chassis_Controller_GetDtSec(config, input);
    Module_Chassis_Controller_BuildState(config,
                                         input,
                                         legStates,
                                         controlState,
                                         wheelAngularVelocityRadps,
                                         &forwardVelocityMps);
    Module_Chassis_Controller_CalculateMotionOutput(config,
                                                    controlState,
                                                    motionOutput);
    status = Module_Chassis_Controller_CalculateSupportForces(config,
                                                              input,
                                                              legStates,
                                                              dtSec,
                                                              supportForcesN);
    if (status != APP_STATUS_OK)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return status;
    }

    status = Module_Chassis_Leg_MapVirtualForce(
        &config->legs[MODULE_CHASSIS_LEG_LEFT],
        &legStates[MODULE_CHASSIS_LEG_LEFT],
        supportForcesN[MODULE_CHASSIS_LEG_LEFT],
        motionOutput[MODULE_CHASSIS_CONTROL_LEFT_LEG_TORQUE],
        &jointTorques[MODULE_CHASSIS_LEG_LEFT]);
    if (status != APP_STATUS_OK)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return status;
    }

    status = Module_Chassis_Leg_MapVirtualForce(
        &config->legs[MODULE_CHASSIS_LEG_RIGHT],
        &legStates[MODULE_CHASSIS_LEG_RIGHT],
        supportForcesN[MODULE_CHASSIS_LEG_RIGHT],
        motionOutput[MODULE_CHASSIS_CONTROL_RIGHT_LEG_TORQUE],
        &jointTorques[MODULE_CHASSIS_LEG_RIGHT]);
    if (status != APP_STATUS_OK)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return status;
    }

    Module_Chassis_Controller_ApplyJointTorques(config, jointTorques, output);
    Module_Chassis_Controller_ApplyWheelCurrents(config, motionOutput, output);

    memcpy(chassisControllerState.debug.legStates, legStates, sizeof(legStates));
    memcpy(chassisControllerState.debug.controlState, controlState, sizeof(controlState));
    memcpy(chassisControllerState.debug.motionOutput, motionOutput, sizeof(motionOutput));
    memcpy(chassisControllerState.debug.supportForcesN, supportForcesN, sizeof(supportForcesN));
    memcpy(chassisControllerState.debug.wheelAngularVelocityRadps,
           wheelAngularVelocityRadps,
           sizeof(wheelAngularVelocityRadps));
    chassisControllerState.debug.forwardVelocityMps = forwardVelocityMps;
    chassisControllerState.debug.isStateValid = 1U;

    return APP_STATUS_OK;
}

app_status_t Module_Chassis_Controller_GetDebug(module_chassis_controller_debug_t *debug)
{
    if (debug == NULL)
    {
        return APP_STATUS_INVALID_PARAM;
    }

    *debug = chassisControllerState.debug;

    return APP_STATUS_OK;
}
