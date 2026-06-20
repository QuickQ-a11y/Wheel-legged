#include "module_chassis_model.h"

static const module_chassis_model_config_t chassisDefaultModelConfig = {
    .legs = {
        [MODULE_CHASSIS_LEG_LEFT] = {
            .geometry = {
                .link1LengthM = 0.215f,
                .link2LengthM = 0.258f,
                .link3LengthM = 0.258f,
                .link4LengthM = 0.215f,
                .frameJointDistanceM = 0.0f,
                .minLegLengthM = 0.05f,
            },
            .joints = {
                [MODULE_CHASSIS_LEG_JOINT_FRONT] = {
                    .motorIndex = 0U,
                    .angleOffsetRad = MODULE_CHASSIS_MODEL_PI,
                    .angleScale = -1.0f,
                    .torqueScale = -1.0f,
                },
                [MODULE_CHASSIS_LEG_JOINT_BACK] = {
                    .motorIndex = 1U,
                    .angleOffsetRad = 0.0f,
                    .angleScale = -1.0f,
                    .torqueScale = -1.0f,
                },
            },
            .targetLegLengthM = 0.25f,
        },
        [MODULE_CHASSIS_LEG_RIGHT] = {
            .geometry = {
                .link1LengthM = 0.215f,
                .link2LengthM = 0.258f,
                .link3LengthM = 0.258f,
                .link4LengthM = 0.215f,
                .frameJointDistanceM = 0.0f,
                .minLegLengthM = 0.05f,
            },
            .joints = {
                [MODULE_CHASSIS_LEG_JOINT_FRONT] = {
                    .motorIndex = 2U,
                    .angleOffsetRad = MODULE_CHASSIS_MODEL_PI,
                    .angleScale = -1.0f,
                    .torqueScale = -1.0f,
                },
                [MODULE_CHASSIS_LEG_JOINT_BACK] = {
                    .motorIndex = 3U,
                    .angleOffsetRad = 0.0f,
                    .angleScale = -1.0f,
                    .torqueScale = -1.0f,
                },
            },
            .targetLegLengthM = 0.25f,
        },
    },
    .imu = {
        .bodyPitchRateGyroIndex = 1U,
        .rollRateGyroIndex = 0U,
        .yawRateGyroIndex = 2U,
        .bodyPitchAngleScale = 1.0f,
        .bodyPitchRateScale = 1.0f,
        .rollAngleScale = 1.0f,
        .rollRateScale = 1.0f,
        .yawAngleScale = 1.0f,
        .yawRateScale = 1.0f,
    },
    .wheel = {
        .radiusM = 0.10f,
        .halfTrackM = 0.1965f,
        .leftVelocityScale = 1.0f,
        .rightVelocityScale = 1.0f,
        .torqueLimitNm = 0.0f,
        .torqueToCurrentRaw = 0.0f,
        .currentLimitRaw = 0,
    },
    .legLengthPid = {
        .kp = 400.0f,
        .ki = 2.0f,
        .kd = 8000.0f,
        .integralLimit = 50.0f,
        .outputLimit = 300.0f,
    },
    .rollPid = {
        .kp = 3000.0f,
        .ki = 1.0f,
        .kd = 100.0f,
        .integralLimit = 30.0f,
        .outputLimit = 300.0f,
    },
    .output = {
        .jointTorqueOutputEnabled = 0U,
        .wheelCurrentOutputEnabled = 0U,
        .jointTorqueLimitNm = 0.0f,
    },
    .legVerticalAngleOffsetRad = MODULE_CHASSIS_MODEL_HALF_PI,
    .targetRollRad = 0.0f,
    .baseSupportForceN = -30.0f,
    .leftSupportForceFeedforwardN = 0.0f,
    .rightSupportForceFeedforwardN = 0.0f,
    .defaultDtSec = APP_CONFIG_IMU_DEFAULT_DT_SEC,
    .minDtSec = 0.0002f,
    .maxDtSec = 0.02f,
    .targetState = {0.0f},
    .motionGain = {{0.0f}},
};

const module_chassis_model_config_t *Module_Chassis_Model_GetDefaultConfig(void)
{
    return &chassisDefaultModelConfig;
}
