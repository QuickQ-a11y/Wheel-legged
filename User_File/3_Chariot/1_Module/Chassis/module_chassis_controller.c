#include "module_chassis_controller.h"

#include "Kalman.h"
#include "LQR.h"
#include "PID.h"

#include <math.h>
#include <string.h>

#define MODULE_CHASSIS_RPM_TO_RADPS 0.10471975512f
#define MODULE_CHASSIS_CONTROLLER_EPSILON 1.0e-6f

/**
 * @brief 底盘控制器内部状态。
 *
 * PID 状态只放在控制器内部，避免任务层或设备层维护控制算法细节。
 * debug 保存最近一次中间量，便于调参和 review 控制链路。
 */
typedef struct
{
    algorithm_kalman_t velocityKalman;
    algorithm_pid_state_t legLengthPid[MODULE_CHASSIS_LEG_COUNT];
    algorithm_pid_state_t rollPid;
    float forwardPositionM;
} module_chassis_controller_state_t;

static module_chassis_controller_state_t chassisControllerState;
module_chassis_controller_debug_t chassisControllerDebug;

/**
 * @brief 将浮点值限制在指定上下界内。
 */
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

/**
 * @brief 对称限幅，常用于力矩、电流和 PID 输出。
 *
 * limit 小于等于 0 时视为不允许输出，直接返回 0。
 */
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

/**
 * @brief 将浮点命令限幅后转换为 int16_t 电流命令。
 *
 * DJI 电机发送接口使用原始电流值，最终输出需要收敛到 int16_t。
 */
static int16_t Module_Chassis_Controller_LimitInt16(float value, int16_t limit)
{
    float limitedValue = Module_Chassis_Controller_LimitSymmetric(value, (float)limit);

    return (int16_t)limitedValue;
}

/**
 * @brief 获取本轮控制计算使用的采样周期。
 *
 * IMU 任务提供的 dtSec 可能因初始化、调试暂停或调度抖动异常。
 * 超出配置范围时使用默认周期，避免 PID 积分和微分项被异常周期放大。
 */
static float Module_Chassis_Controller_SelectDtSec(const module_chassis_model_config_t *config,
                                                   const module_chassis_input_t *input)
{
    float dtSec;

    dtSec = input->imu.dtSec;

    /* 控制器只接受合理的采样周期，异常周期会放大 PID 积分项。 */
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

/**
 * @brief 清空速度卡尔曼和前进位移积分。
 */
void Module_Chassis_Controller_ResetMotionState(const module_chassis_model_config_t *config)
{
    algorithm_kalman_t *velocityKalman = &chassisControllerState.velocityKalman;

    /*
     * 速度融合状态固定为 x = [dot_s, ddot_s]。
     * P/Q/R 参数来自 module_chassis_model.c，后续按实机噪声只改配置表。
     */
    Algorithm_Kalman_Init(velocityKalman, 2U, 2U);
    memcpy(velocityKalman->covariance,
           config->velocityKalman.initialCovariance,
           sizeof(config->velocityKalman.initialCovariance));
    memcpy(velocityKalman->processNoise,
           config->velocityKalman.processNoise,
           sizeof(config->velocityKalman.processNoise));
    memcpy(velocityKalman->measurementNoise,
           config->velocityKalman.measurementNoise,
           sizeof(config->velocityKalman.measurementNoise));

    velocityKalman->stateTransition[0] = 1.0f;
    velocityKalman->stateTransition[1] = config->defaultDtSec;
    velocityKalman->stateTransition[2] = 0.0f;
    velocityKalman->stateTransition[3] = 1.0f;

    velocityKalman->measurementMatrix[0] = 1.0f;
    velocityKalman->measurementMatrix[1] = 0.0f;
    velocityKalman->measurementMatrix[2] = 0.0f;
    velocityKalman->measurementMatrix[3] = 1.0f;

    chassisControllerState.forwardPositionM = 0.0f;
}

/**
 * @brief 从 DM 电机反馈中取出一条腿的前后髋关节状态。
 *
 * legConfig 只保存机械映射下标，真实反馈值来自任务层汇总后的 input。
 */
static void Module_Chassis_Controller_ReadJointState(
    const module_chassis_input_t *input,
    const module_chassis_leg_config_t *legConfig,
    float jointPositionRad[MODULE_CHASSIS_LEG_JOINT_COUNT],
    float jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_COUNT])
{
    uint32_t jointIndex;

    for (jointIndex = 0U; jointIndex < MODULE_CHASSIS_LEG_JOINT_COUNT; jointIndex++)
    {
        uint8_t motorIndex = legConfig->joints[jointIndex].motorIndex;

        /* 这里只取设备层维护的真实反馈，机械零位和方向在腿部几何层统一处理。 */
        jointPositionRad[jointIndex] = input->dmMotors[motorIndex].positionRad;
        jointVelocityRadps[jointIndex] = input->dmMotors[motorIndex].velocityRadps;
    }
}

/**
 * @brief 根据两条腿的髋关节反馈计算五连杆虚拟腿状态。
 *
 * 输出的 legStates 后续会同时用于状态向量构造和 VMC 力矩映射。
 */
static void Module_Chassis_Controller_BuildLegStates(
    const module_chassis_model_config_t *config,
    const module_chassis_input_t *input,
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT])
{
    uint32_t sideIndex;

    for (sideIndex = 0U; sideIndex < MODULE_CHASSIS_LEG_COUNT; sideIndex++)
    {
        float jointPositionRad[MODULE_CHASSIS_LEG_JOINT_COUNT] = {0.0f};
        float jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_COUNT] = {0.0f};
        /* 先按配置表把前后髋关节反馈取出，保持控制器不关心具体电机编号。 */
        Module_Chassis_Controller_ReadJointState(input,
                                                 &config->legs[sideIndex],
                                                 jointPositionRad,
                                                 jointVelocityRadps);

        /* 五连杆状态由腿部模块负责，控制器只消费腿长、腿角和速度结果。 */
        Module_Chassis_Leg_CalculateState(
            &config->legs[sideIndex],
            jointPositionRad[MODULE_CHASSIS_LEG_JOINT_FRONT],
            jointPositionRad[MODULE_CHASSIS_LEG_JOINT_BACK],
            jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_FRONT],
            jointVelocityRadps[MODULE_CHASSIS_LEG_JOINT_BACK],
            &legStates[sideIndex]);
    }
}

/**
 * @brief 构造轮腿平衡控制十维状态向量。
 *
 * 轮速、腿部几何和 IMU 先被整理成局部物理量；最后用一个连续代码块写入
 * state[0..9]，避免十维状态分散赋值导致 review 时误判数据来源。
 */
static void Module_Chassis_Controller_BuildState(
    const module_chassis_model_config_t *config,
    const module_chassis_input_t *input,
    const module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT],
    float dtSec,
    float state[MODULE_CHASSIS_CONTROL_STATE_COUNT],
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT],
    float *rawForwardVelocityMps,
    float *forwardAccelerationMps2,
    float *fusedForwardAccelerationMps2)
{
    algorithm_kalman_t *velocityKalman = &chassisControllerState.velocityKalman;
    float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT] = {0.0f};
    float leftWheelVelocityRadps;
    float rightWheelVelocityRadps;
    float bodyPitchRad;
    float bodyPitchRateRadps;
    float leftLegAngleRad;
    float rightLegAngleRad;
    float leftLegAngleRateRadps;
    float rightLegAngleRateRadps;
    float fusedForwardVelocityMps;
    float yawRad;
    float yawRateRadps;

    memset(state, 0, sizeof(float) * MODULE_CHASSIS_CONTROL_STATE_COUNT);

    /* DJI 反馈为 rpm，先换算成轮端角速度，再按左右安装方向修正符号。 */
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

    /* IMU 轴向和符号由模型配置决定，避免在控制器里硬编码板子安装方向。 */
    bodyPitchRad = input->imu.pitchRad * config->imu.bodyPitchAngleScale;
    bodyPitchRateRadps =
        input->imu.gyroRadps[config->imu.bodyPitchRateGyroIndex] *
        config->imu.bodyPitchRateScale;

    /*
     * 腿角参考 SPR 的 chassis_feedback_update：
     * theta = phi0 - 竖直参考角 - pitch。
     * 这里的符号必须和离线求 K 时的状态定义保持一致。
     */
    leftLegAngleRad = legStates[MODULE_CHASSIS_LEG_LEFT].phi0Rad -
                      config->legVerticalAngleOffsetRad -
                      bodyPitchRad;
    rightLegAngleRad = legStates[MODULE_CHASSIS_LEG_RIGHT].phi0Rad -
                       config->legVerticalAngleOffsetRad -
                       bodyPitchRad;
    leftLegAngleRateRadps =
        legStates[MODULE_CHASSIS_LEG_LEFT].legSwingVelocityRadps -
        bodyPitchRateRadps;
    rightLegAngleRateRadps =
        legStates[MODULE_CHASSIS_LEG_RIGHT].legSwingVelocityRadps -
        bodyPitchRateRadps;

    /*
     * 前进速度由轮速、腿摆角速度和腿长变化速度共同估计。
     * 该值作为卡尔曼测量中的原始速度，不在这里做滤波或积分。
     */
    *rawForwardVelocityMps =
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

    *forwardAccelerationMps2 =
        input->imu.motionAccMps2[config->imu.forwardAccelerationIndex] *
        config->imu.forwardAccelerationScale;

    /*
     * 速度融合参考 SPR：x = [dot_s, ddot_s]，
     * z = [原始前进速度, IMU 前向运动加速度]。
     */
    if (config->velocityKalman.enabled != 0U)
    {
        velocityKalman->stateTransition[0] = 1.0f;
        velocityKalman->stateTransition[1] = dtSec;
        velocityKalman->stateTransition[2] = 0.0f;
        velocityKalman->stateTransition[3] = 1.0f;

        measurement[0] = *rawForwardVelocityMps;
        measurement[1] = *forwardAccelerationMps2;
        Algorithm_Kalman_Update(velocityKalman, measurement);

        fusedForwardVelocityMps = velocityKalman->state[0];
        *fusedForwardAccelerationMps2 = velocityKalman->state[1];
    }
    else
    {
        fusedForwardVelocityMps = *rawForwardVelocityMps;
        *fusedForwardAccelerationMps2 = *forwardAccelerationMps2;
        velocityKalman->state[0] = fusedForwardVelocityMps;
        velocityKalman->state[1] = *fusedForwardAccelerationMps2;
    }

    /*
     * 位置状态沿用 SPR 的调试策略：速度较小时积分，速度过大时清零。
     * 这样早期未闭环速度目标时，s 不会在快速滑动或搬动机器人时持续漂移。
     */
    if ((config->velocityKalman.positionIntegralVelocityLimitMps > 0.0f) &&
        (fabsf(fusedForwardVelocityMps) <=
         config->velocityKalman.positionIntegralVelocityLimitMps))
    {
        chassisControllerState.forwardPositionM += fusedForwardVelocityMps * dtSec;
    }
    else
    {
        chassisControllerState.forwardPositionM = 0.0f;
    }

    yawRad = input->imu.yawRad * config->imu.yawAngleScale;
    yawRateRadps =
        input->imu.gyroRadps[config->imu.yawRateGyroIndex] *
        config->imu.yawRateScale;

    /* 十维状态向量只在这里集中赋值，顺序必须和离线求 K 时完全一致。 */
    state[MODULE_CHASSIS_STATE_FORWARD_POSITION] =
        chassisControllerState.forwardPositionM;
    state[MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = fusedForwardVelocityMps;
    state[MODULE_CHASSIS_STATE_YAW] = yawRad;
    state[MODULE_CHASSIS_STATE_YAW_RATE] = yawRateRadps;
    state[MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = leftLegAngleRad;
    state[MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = leftLegAngleRateRadps;
    state[MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = rightLegAngleRad;
    state[MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = rightLegAngleRateRadps;
    state[MODULE_CHASSIS_STATE_BODY_PITCH] = bodyPitchRad;
    state[MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = bodyPitchRateRadps;
}

/**
 * @brief 根据配置选择本轮 LQR 拟合使用的左右腿长。
 *
 * 定腿长模式用于早期固定姿态和电机方向调试；实测模式用于后续变腿长控制。
 */
static void Module_Chassis_Controller_SelectLqrLegLength(
    const module_chassis_model_config_t *config,
    const module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT],
    float lqrInputLegLengthM[MODULE_CHASSIS_LEG_COUNT])
{
    if (config->lqr.lqrKLengthSource == MODULE_CHASSIS_LQR_K_LENGTH_MEASURED)
    {
        lqrInputLegLengthM[MODULE_CHASSIS_LEG_LEFT] =
            legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM;
        lqrInputLegLengthM[MODULE_CHASSIS_LEG_RIGHT] =
            legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM;
        return;
    }

    lqrInputLegLengthM[MODULE_CHASSIS_LEG_LEFT] = config->lqr.fixedLeftLegLengthM;
    lqrInputLegLengthM[MODULE_CHASSIS_LEG_RIGHT] = config->lqr.fixedRightLegLengthM;
}

/**
 * @brief 生成本轮运动控制实际使用的 K 矩阵。
 *
 * LQR 拟合关闭时直接使用配置中的 fixedLqrK；开启时使用同一套 poly22 拟合算法，
 * 只通过 lqrKLengthSource 切换固定腿长或实时腿长输入。
 */
static void Module_Chassis_Controller_BuildLqrK(
    const module_chassis_model_config_t *config,
    const module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT],
    float lqrK[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT],
    float lqrInputLegLengthM[MODULE_CHASSIS_LEG_COUNT],
    float lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_COUNT],
    uint8_t *isLqrKFitEnabled,
    uint8_t *isLqrKLengthLimited)
{
    uint8_t isInputLimited = 0U;

    *isLqrKFitEnabled = 0U;
    *isLqrKLengthLimited = 0U;
    lqrInputLegLengthM[MODULE_CHASSIS_LEG_LEFT] = 0.0f;
    lqrInputLegLengthM[MODULE_CHASSIS_LEG_RIGHT] = 0.0f;
    lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_LEFT] = 0.0f;
    lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_RIGHT] = 0.0f;

    if (config->lqr.enabled == 0U)
    {
        memcpy(lqrK, config->fixedLqrK, sizeof(config->fixedLqrK));
        return;
    }

    Module_Chassis_Controller_SelectLqrLegLength(config, legStates, lqrInputLegLengthM);

    Algorithm_LQR_FitLqrKPoly22(
        &config->lqr.lqrKFitCoefficients[0U][0U][0U],
        MODULE_CHASSIS_CONTROL_OUTPUT_COUNT,
        MODULE_CHASSIS_CONTROL_STATE_COUNT,
        lqrInputLegLengthM[MODULE_CHASSIS_LEG_LEFT],
        lqrInputLegLengthM[MODULE_CHASSIS_LEG_RIGHT],
        config->lqr.minFitLegLengthM,
        config->lqr.maxFitLegLengthM,
        &lqrK[0U][0U],
        &lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_LEFT],
        &lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_RIGHT],
        &isInputLimited);

    *isLqrKFitEnabled = 1U;
    *isLqrKLengthLimited = isInputLimited;
}

/**
 * @brief 根据状态反馈计算 LQR/MPC 风格的运动控制输出。
 *
 * lqrK 是本轮实际使用的输出到状态增益矩阵，来源可以是固定矩阵，也可以是腿长拟合 K。
 * 输出顺序为左右轮力矩、左右腿摆虚拟力矩。
 * 左右腿摆虚拟力矩方向跟随 SPR 状态定义，电机最终正负号由模型配置处理。
 */
static void Module_Chassis_Controller_CalculateMotionOutput(
    const module_chassis_model_config_t *config,
    const float lqrK[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT],
    const float state[MODULE_CHASSIS_CONTROL_STATE_COUNT],
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT])
{
    uint32_t outputIndex;
    uint32_t stateIndex;

    for (outputIndex = 0U; outputIndex < MODULE_CHASSIS_CONTROL_OUTPUT_COUNT; outputIndex++)
    {
        float outputValue = 0.0f;

        /* 每一行增益对应一个广义输出：左轮、右轮、左腿摆、右腿摆。 */
        for (stateIndex = 0U; stateIndex < MODULE_CHASSIS_CONTROL_STATE_COUNT; stateIndex++)
        {
            outputValue += lqrK[outputIndex][stateIndex] *
                           (config->targetState[stateIndex] - state[stateIndex]);
        }

        motionOutput[outputIndex] = outputValue;
    }
}

/**
 * @brief 计算左右腿虚拟支撑力。
 *
 * 腿长 PID 负责让虚拟腿收敛到目标腿长，横滚 PID 通过左右腿支撑力差修正横滚。
 * 输出支撑力会交给腿部 VMC 映射为前后髋关节力矩。
 */
static void Module_Chassis_Controller_CalculateSupportForces(
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

    rollRad = input->imu.rollRad * config->imu.rollAngleScale;
    rollRateRadps =
        input->imu.gyroRadps[config->imu.rollRateGyroIndex] *
        config->imu.rollRateScale;

    /* 腿长环输出为虚拟支撑力修正量，反馈速度直接作为阻尼项。 */
    Algorithm_PID_UpdateByFeedbackRate(
        &config->legLengthPid,
        &chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_LEFT],
        config->legs[MODULE_CHASSIS_LEG_LEFT].targetLegLengthM,
        legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM,
        legStates[MODULE_CHASSIS_LEG_LEFT].legLengthVelocityMps,
        dtSec,
        &leftPidOutput);
    Algorithm_PID_UpdateByFeedbackRate(
        &config->legLengthPid,
        &chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_RIGHT],
        config->legs[MODULE_CHASSIS_LEG_RIGHT].targetLegLengthM,
        legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM,
        legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthVelocityMps,
        dtSec,
        &rightPidOutput);
    /* 横滚环输出为左右腿支撑力差，修正方向由下方左右腿合成关系统一处理。 */
    Algorithm_PID_UpdateByFeedbackRate(&config->rollPid,
                                       &chassisControllerState.rollPid,
                                       config->targetRollRad,
                                       rollRad,
                                       rollRateRadps,
                                       dtSec,
                                       &rollPidOutput);

    leftLengthCorrectionN = -leftPidOutput;
    rightLengthCorrectionN = -rightPidOutput;
    rollCorrectionN = -rollPidOutput;

    /*
     * 支撑力由三部分组成：基础支撑力、腿长闭环修正、横滚闭环左右差动。
     * 左右横滚项符号相反，使横滚误差通过两腿支撑力差被修正。
     */
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
}

/**
 * @brief 将腿部 VMC 输出写入 DM 髋关节力矩命令。
 *
 * 只有总输出开关、DM 力矩输出开关和力矩限幅全部有效时才允许非零输出。
 * 这里不直接发送 CAN，只填充 output，由任务层统一下发到设备层。
 */
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

    /* 输出结构在上层每帧先清零；开关未打开时保持 DM 零力矩。 */
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

/**
 * @brief 将左右轮力矩输出转换为 DJI 原始电流命令。
 *
 * 轮电机使用 DJI 电流帧，控制器输出的物理力矩需要通过配置系数换算。
 */
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

    /* 轮端物理力矩先限幅，再按配置系数转换为 DJI 电流原始值。 */
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

/**
 * @brief 初始化底盘控制器状态和 PID 状态。
 *
 * config 当前不需要持久化到控制器内部，保留参数是为了和模块公共接口保持一致。
 */
void Module_Chassis_Controller_Init(const module_chassis_model_config_t *config)
{
    memset(&chassisControllerState, 0, sizeof(chassisControllerState));
    memset(&chassisControllerDebug, 0, sizeof(chassisControllerDebug));
    Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_LEFT]);
    Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_RIGHT]);
    Algorithm_PID_Init(&chassisControllerState.rollPid);
    Module_Chassis_Controller_ResetMotionState(config);
}

/**
 * @brief 执行一次底盘控制环。
 *
 * 主流程对齐 SPR 的 chassis_control_loop：先由反馈得到腿部和机体状态，
 * 再计算支撑力、K 矩阵和 LQR 广义输出，最后通过 VMC 写入电机命令。
 * 输入 input 来自任务层汇总后的 IMU、DM、DJI 和 CAN 状态；
 * 输出 output 写回底盘模块，再由任务层统一下发到电机设备层。
 * 当前只保留站立控制主链路，未接入遥控状态机、自起身、跳跃、KNN 或功率限制。
 */
void Module_Chassis_Controller_RunControlLoop(const module_chassis_model_config_t *config,
                                              const module_chassis_input_t *input,
                                              module_chassis_output_t *output)
{
    module_chassis_leg_state_t legStates[MODULE_CHASSIS_LEG_COUNT];
    module_chassis_leg_joint_torque_t jointTorques[MODULE_CHASSIS_LEG_COUNT];
    float controlState[MODULE_CHASSIS_CONTROL_STATE_COUNT] = {0.0f};
    float lqrK[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT] = {{0.0f}};
    float motionOutput[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT] = {0.0f};
    float supportForcesN[MODULE_CHASSIS_LEG_COUNT] = {0.0f};
    float lqrInputLegLengthM[MODULE_CHASSIS_LEG_COUNT] = {0.0f};
    float lqrLimitedLegLengthM[MODULE_CHASSIS_LEG_COUNT] = {0.0f};
    float wheelAngularVelocityRadps[APP_CONFIG_DJI_WHEEL_COUNT] = {0.0f};
    float rawForwardVelocityMps = 0.0f;
    float forwardAccelerationMps2 = 0.0f;
    float fusedForwardAccelerationMps2 = 0.0f;
    float dtSec;
    uint8_t isLqrKFitEnabled = 0U;
    uint8_t isLqrKLengthLimited = 0U;
    uint32_t sideIndex;
    uint32_t jointIndex;

    output->faultFlags = MODULE_CHASSIS_FAULT_CONTROLLER;
    chassisControllerDebug.isStateValid = 0U;

    memset(legStates, 0, sizeof(legStates));
    memset(jointTorques, 0, sizeof(jointTorques));

    /*
     * 1. 控制环入口只保留真实安全边界。
     * 输入：config 中的 IMU 轴向、腿部几何、DM 电机映射和 lqr 配置。
     * 来源：config 来自 module_chassis_model.c 的默认模型配置。
     * 输出：本阶段不产生控制量；失败时直接返回，output 保持 module_chassis.c
     *       预先写入的安全零输出和 MODULE_CHASSIS_FAULT_CONTROLLER。
     * 去向：边界通过后，config 才允许进入后续反馈、K 矩阵和 VMC 计算。
     */
    if ((config->imu.bodyPitchRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.rollRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.yawRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.forwardAccelerationIndex >= APP_CONFIG_IMU_AXIS_COUNT))
    {
        return;
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
            return;
        }

        for (jointIndex = 0U; jointIndex < MODULE_CHASSIS_LEG_JOINT_COUNT; jointIndex++)
        {
            if (config->legs[sideIndex].joints[jointIndex].motorIndex >=
                APP_CONFIG_DM_MOTOR_COUNT)
            {
                return;
            }
        }
    }

    if ((config->lqr.enabled != 0U) &&
        ((config->lqr.minFitLegLengthM <= 0.0f) ||
         (config->lqr.maxFitLegLengthM < config->lqr.minFitLegLengthM) ||
         ((config->lqr.lqrKLengthSource != MODULE_CHASSIS_LQR_K_LENGTH_FIXED) &&
          (config->lqr.lqrKLengthSource != MODULE_CHASSIS_LQR_K_LENGTH_MEASURED)) ||
         ((config->lqr.lqrKLengthSource == MODULE_CHASSIS_LQR_K_LENGTH_FIXED) &&
          ((config->lqr.fixedLeftLegLengthM <= 0.0f) ||
           (config->lqr.fixedRightLegLengthM <= 0.0f)))))
    {
        return;
    }

    /*
     * 2. 反馈处理：四个髋关节反馈先转换成两条虚拟腿状态，
     * 再和 IMU、轮速一起组成 LQR 状态向量。
     * 输入：input->dmMotors、input->djiWheels、input->imu 和 config 的机械/轴向映射。
     * 来源：input 由 chassis_task.c 从设备层反馈整理后传入。
     * 输出：legStates、controlState、wheelAngularVelocityRadps、rawForwardVelocityMps、
     *       forwardAccelerationMps2、fusedForwardAccelerationMps2 和 dtSec。
     * 去向：BuildState 内部完成原始速度计算、速度融合和十维状态集中赋值；
     *       最终 controlState 供 LQR 输出使用，轮速和速度融合中间量进入 debug。
     */
    Module_Chassis_Controller_BuildLegStates(config, input, legStates);
    if ((legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM <=
         config->legs[MODULE_CHASSIS_LEG_LEFT].geometry.minLegLengthM) ||
        (legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM <=
         config->legs[MODULE_CHASSIS_LEG_RIGHT].geometry.minLegLengthM))
    {
        return;
    }

    dtSec = Module_Chassis_Controller_SelectDtSec(config, input);
    Module_Chassis_Controller_BuildState(config,
                                         input,
                                         legStates,
                                         dtSec,
                                         controlState,
                                         wheelAngularVelocityRadps,
                                         &rawForwardVelocityMps,
                                         &forwardAccelerationMps2,
                                         &fusedForwardAccelerationMps2);

    /*
     * 3. 支撑力：先算 VMC 使用的左右腿虚拟支撑力。
     * 当前只包含腿长 PID、支撑力前馈和横滚补偿。
     * 输入：config 的腿长目标、PID 参数、支撑力前馈和 roll 目标；
     *       input->imu 的 roll/roll rate；legStates 的腿长和腿长速度；dtSec。
     * 来源：目标和 PID 来自 module_chassis_model.c，IMU 来自任务层输入，
     *       腿部状态来自阶段 2。
     * 输出：supportForcesN[左腿/右腿]，单位 N。
     * 去向：supportForcesN 在阶段 6 进入 VMC，和腿摆力矩一起映射到髋关节力矩。
     */
    Module_Chassis_Controller_CalculateSupportForces(config,
                                                     input,
                                                     legStates,
                                                     dtSec,
                                                     supportForcesN);

    /*
     * 4. K 矩阵：定腿长和变腿长只在这里选择输入腿长，
     * 后面的 LQR 输出、VMC 和电机命令共用同一套代码。
     * 输入：config->lqr 的拟合开关、腿长来源、拟合范围和 K 拟合系数；
     *       legStates 的实时腿长。
     * 来源：lqr 配置集中在 module_chassis_model.c，实时腿长来自阶段 2。
     * 输出：lqrK[4][10]、lqrInputLegLengthM、lqrLimitedLegLengthM、
     *       isLqrKFitEnabled 和 isLqrKLengthLimited。
     * 去向：lqrK 在阶段 5 计算轮/腿广义输出，其余量写入 debug 方便确认拟合输入。
     */
    Module_Chassis_Controller_BuildLqrK(config,
                                        legStates,
                                        lqrK,
                                        lqrInputLegLengthM,
                                        lqrLimitedLegLengthM,
                                        &isLqrKFitEnabled,
                                        &isLqrKLengthLimited);

    /*
     * 5. LQR 输出：motionOutput 顺序为左轮、右轮、左腿摆、右腿摆，
     * 这里仍是控制层广义力矩，不是电机原始命令。
     * 输入：lqrK、controlState 和 config->targetState。
     * 来源：lqrK 来自阶段 4，controlState 来自阶段 2，targetState 来自模型配置。
     * 输出：motionOutput[左轮力矩、右轮力矩、左腿摆力矩、右腿摆力矩]。
     * 去向：轮力矩在阶段 7 转换为 DJI 电流；腿摆力矩在阶段 6 进入 VMC。
     */
    Module_Chassis_Controller_CalculateMotionOutput(config,
                                                    lqrK,
                                                    controlState,
                                                    motionOutput);

    /*
     * 6. VMC 映射：进入雅可比映射前再次检查几何有效性。
     * 奇异位姿下不做关节力矩映射，避免雅可比导致力矩异常放大。
     * 输入：config 的腿部几何和关节力矩方向，legStates 的五连杆姿态，
     *       supportForcesN 和 motionOutput 中的左右腿摆力矩。
     * 来源：几何配置来自 module_chassis_model.c，腿部状态来自阶段 2，
     *       支撑力来自阶段 3，腿摆力矩来自阶段 5。
     * 输出：jointTorques[左腿/右腿] 的前后髋关节力矩，单位 N*m。
     * 去向：jointTorques 在阶段 7 写入 output->dmCommands。
     */
    if ((fabsf(legStates[MODULE_CHASSIS_LEG_LEFT].legLengthM) <=
         MODULE_CHASSIS_CONTROLLER_EPSILON) ||
        (fabsf(legStates[MODULE_CHASSIS_LEG_RIGHT].legLengthM) <=
         MODULE_CHASSIS_CONTROLLER_EPSILON) ||
        (fabsf(sinf(legStates[MODULE_CHASSIS_LEG_LEFT].phi2Rad -
                    legStates[MODULE_CHASSIS_LEG_LEFT].phi3Rad)) <=
         MODULE_CHASSIS_CONTROLLER_EPSILON) ||
        (fabsf(sinf(legStates[MODULE_CHASSIS_LEG_RIGHT].phi2Rad -
                    legStates[MODULE_CHASSIS_LEG_RIGHT].phi3Rad)) <=
         MODULE_CHASSIS_CONTROLLER_EPSILON))
    {
        return;
    }

    Module_Chassis_Leg_MapVirtualForce(
        &config->legs[MODULE_CHASSIS_LEG_LEFT],
        &legStates[MODULE_CHASSIS_LEG_LEFT],
        supportForcesN[MODULE_CHASSIS_LEG_LEFT],
        motionOutput[MODULE_CHASSIS_CONTROL_LEFT_LEG_TORQUE],
        &jointTorques[MODULE_CHASSIS_LEG_LEFT]);

    Module_Chassis_Leg_MapVirtualForce(
        &config->legs[MODULE_CHASSIS_LEG_RIGHT],
        &legStates[MODULE_CHASSIS_LEG_RIGHT],
        supportForcesN[MODULE_CHASSIS_LEG_RIGHT],
        motionOutput[MODULE_CHASSIS_CONTROL_RIGHT_LEG_TORQUE],
        &jointTorques[MODULE_CHASSIS_LEG_RIGHT]);

    /*
     * 7. 输出赋值：控制器只写本轮 output，CAN 发送仍由任务层统一完成。
     * 默认输出开关继续封锁非零命令。
     * 输入：config 的输出开关和限幅参数、jointTorques、motionOutput 中的轮力矩。
     * 来源：输出开关和限幅来自 module_chassis_model.c，jointTorques 来自阶段 6，
     *       轮力矩来自阶段 5。
     * 输出：output->dmCommands、output->djiCurrents 和 output->faultFlags。
     * 去向：output 返回 module_chassis.c，再由 chassis_task.c 写入 DM 命令、
     *       DJI 电流发送缓存和 CAN 发送请求。
     */
    Module_Chassis_Controller_ApplyJointTorques(config, jointTorques, output);
    Module_Chassis_Controller_ApplyWheelCurrents(config, motionOutput, output);
    output->faultFlags = MODULE_CHASSIS_FAULT_NONE;

    /*
     * 8. 调试快照。
     * 输入：本轮控制环所有关键中间量。
     * 来源：阶段 2 到阶段 7 的局部变量。
     * 输出：chassisControllerDebug。
     * 去向：调试器、串口日志或 review 时直接观察，不再额外封装状态读取接口。
     */
    memcpy(chassisControllerDebug.legStates, legStates, sizeof(legStates));
    memcpy(chassisControllerDebug.controlState, controlState, sizeof(controlState));
    memcpy(chassisControllerDebug.lqrK,
           lqrK,
           sizeof(lqrK));
    memcpy(chassisControllerDebug.motionOutput, motionOutput, sizeof(motionOutput));
    memcpy(chassisControllerDebug.supportForcesN, supportForcesN, sizeof(supportForcesN));
    memcpy(chassisControllerDebug.lqrInputLegLengthM,
           lqrInputLegLengthM,
           sizeof(lqrInputLegLengthM));
    memcpy(chassisControllerDebug.lqrLimitedLegLengthM,
           lqrLimitedLegLengthM,
           sizeof(lqrLimitedLegLengthM));
    memcpy(chassisControllerDebug.wheelAngularVelocityRadps,
           wheelAngularVelocityRadps,
           sizeof(wheelAngularVelocityRadps));
    chassisControllerDebug.rawForwardVelocityMps = rawForwardVelocityMps;
    chassisControllerDebug.forwardVelocityMps =
        controlState[MODULE_CHASSIS_STATE_FORWARD_VELOCITY];
    chassisControllerDebug.forwardAccelerationMps2 = forwardAccelerationMps2;
    chassisControllerDebug.fusedForwardAccelerationMps2 = fusedForwardAccelerationMps2;
    chassisControllerDebug.forwardPositionM =
        controlState[MODULE_CHASSIS_STATE_FORWARD_POSITION];
    chassisControllerDebug.isLqrKFitEnabled = isLqrKFitEnabled;
    chassisControllerDebug.isLqrKLengthLimited = isLqrKLengthLimited;
    chassisControllerDebug.isStateValid = 1U;
}
