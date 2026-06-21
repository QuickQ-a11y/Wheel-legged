#include "module_chassis_controller.h"

#include "PID.h"

#include <math.h>
#include <string.h>

#define MODULE_CHASSIS_RPM_TO_RADPS 0.10471975512f

/**
 * @brief 底盘控制器内部状态。
 *
 * PID 状态只放在控制器内部，避免任务层或设备层维护控制算法细节。
 * debug 保存最近一次中间量，便于调参和 review 控制链路。
 */
typedef struct
{
    algorithm_pid_state_t legLengthPid[MODULE_CHASSIS_LEG_COUNT];
    algorithm_pid_state_t rollPid;
    module_chassis_controller_debug_t debug;
} module_chassis_controller_state_t;

static module_chassis_controller_state_t chassisControllerState;

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
 * @brief 检查控制器配置是否满足本次计算的最低要求。
 *
 * 这里只检查会导致数组越界、几何无效或输出映射错误的硬约束。
 * 具体参数是否需要重新调参由模型配置和实机测试决定。
 */
static uint8_t Module_Chassis_Controller_IsConfigValid(
    const module_chassis_model_config_t *config)
{
    uint32_t sideIndex;
    uint32_t jointIndex;

    if (config == NULL)
    {
        return 0U;
    }

    /* IMU 轴向下标来自模型配置，越界会直接读错姿态角速度。 */
    if ((config->imu.bodyPitchRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.rollRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT) ||
        (config->imu.yawRateGyroIndex >= APP_CONFIG_IMU_AXIS_COUNT))
    {
        return 0U;
    }

    /* 腿部几何和 DM 电机映射是后续正运动学与输出映射的硬边界。 */
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

/**
 * @brief 获取本轮控制计算使用的采样周期。
 *
 * IMU 任务提供的 dtSec 可能因初始化、调试暂停或调度抖动异常。
 * 超出配置范围时使用默认周期，避免 PID 积分和微分项被异常周期放大。
 */
static float Module_Chassis_Controller_GetDtSec(const module_chassis_model_config_t *config,
                                                const module_chassis_input_t *input)
{
    float dtSec;

    if ((config == NULL) || (input == NULL))
    {
        return APP_CONFIG_IMU_DEFAULT_DT_SEC;
    }

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
 * @brief 从 DM 电机反馈中取出一条腿的前后髋关节状态。
 *
 * legConfig 只保存机械映射下标，真实反馈值来自任务层汇总后的 input。
 */
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

        /* 这里只取设备层维护的真实反馈，机械零位和方向在腿部几何层统一处理。 */
        jointPositionRad[jointIndex] = input->dmMotors[motorIndex].positionRad;
        jointVelocityRadps[jointIndex] = input->dmMotors[motorIndex].velocityRadps;
    }

    return APP_STATUS_OK;
}

/**
 * @brief 根据两条腿的髋关节反馈计算五连杆虚拟腿状态。
 *
 * 输出的 legStates 后续会同时用于状态向量构造和 VMC 力矩映射。
 */
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

        /* 先按配置表把前后髋关节反馈取出，保持控制器不关心具体电机编号。 */
        status = Module_Chassis_Controller_GetJointState(input,
                                                         &config->legs[sideIndex],
                                                         jointPositionRad,
                                                         jointVelocityRadps);
        if (status != APP_STATUS_OK)
        {
            return status;
        }

        /* 五连杆状态由腿部模块负责，控制器只消费腿长、腿角和速度结果。 */
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

/**
 * @brief 构造轮腿平衡控制状态向量。
 *
 * 当前状态顺序由 module_chassis_model.h 的枚举固定：
 * 前进位置、前进速度、偏航角、偏航角速度、左右腿摆角及角速度、机体俯仰角及角速度。
 * 前进位置暂时置 0，后续加入里程计或融合速度积分时再统一接入。
 */
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
     * 控制腿角沿用 Webots 定义：theta = 竖直参考角 - phi0 + pitch。
     * theta > 0 已确认对应虚拟腿向机身后方倾斜。
     */
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

    /*
     * 前进速度由轮速、腿摆角速度和腿长变化速度共同估计。
     * 该值是控制器当前的速度反馈，不在这里做滤波或积分。
     */
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

    /* 状态向量顺序必须和离线求 K 时的状态定义完全一致。 */
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

/**
 * @brief 根据状态反馈计算 LQR/MPC 风格的运动控制输出。
 *
 * motionGain 是输出到状态的增益矩阵，目标状态与当前状态的误差按行累加。
 * 输出顺序为左右轮力矩、左右腿摆虚拟力矩。
 * 左右腿摆虚拟力矩为正时，分别使 theta_l/theta_r 增大，即驱动虚拟腿向机身后方摆动。
 */
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

        /* 每一行增益对应一个广义输出：左轮、右轮、左腿摆、右腿摆。 */
        for (stateIndex = 0U; stateIndex < MODULE_CHASSIS_CONTROL_STATE_COUNT; stateIndex++)
        {
            outputValue += config->motionGain[outputIndex][stateIndex] *
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

    /* 腿长环输出为虚拟支撑力修正量，反馈速度直接作为阻尼项。 */
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

    /* 横滚环输出为左右腿支撑力差，修正方向由下方左右腿合成关系统一处理。 */
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

    return APP_STATUS_OK;
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
    (void)config;
    memset(&chassisControllerState, 0, sizeof(chassisControllerState));
    (void)Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_LEFT]);
    (void)Algorithm_PID_Init(&chassisControllerState.legLengthPid[MODULE_CHASSIS_LEG_RIGHT]);
    (void)Algorithm_PID_Init(&chassisControllerState.rollPid);
}

/**
 * @brief 执行一次底盘控制器计算。
 *
 * 调用链为：配置检查、腿部几何状态计算、控制状态构造、运动控制输出、
 * 支撑力计算、VMC 力矩映射、输出限幅和调试快照更新。
 */
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

    /* 阶段 1：检查配置硬边界，失败时本轮调试数据标记为无效。 */
    if (Module_Chassis_Controller_IsConfigValid(config) == 0U)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return APP_STATUS_INVALID_PARAM;
    }

    memset(legStates, 0, sizeof(legStates));
    memset(jointTorques, 0, sizeof(jointTorques));

    /* 阶段 2：把四个髋关节反馈转换为两条虚拟腿的几何状态。 */
    status = Module_Chassis_Controller_UpdateLegStates(config, input, legStates);
    if (status != APP_STATUS_OK)
    {
        chassisControllerState.debug.isStateValid = 0U;
        return status;
    }

    /*
     * 阶段 3：构造控制状态并计算运动控制输出。
     * motionOutput 是广义输出，还不是最终电机命令。
     */
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
    /*
     * 阶段 4：腿长和横滚闭环生成左右虚拟支撑力。
     * 支撑力后续和腿摆力矩一起交给 VMC 做关节力矩映射。
     */
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

    /* 阶段 5：左腿 VMC，把支撑力和左腿摆力矩映射到前后髋关节。 */
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

    /* 阶段 6：右腿 VMC，符号约定与左腿一致，电机安装方向由配置处理。 */
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

    /* 阶段 7：把广义控制量写入输出结构，实际 CAN 下发由任务层完成。 */
    Module_Chassis_Controller_ApplyJointTorques(config, jointTorques, output);
    Module_Chassis_Controller_ApplyWheelCurrents(config, motionOutput, output);

    /* 阶段 8：保存本轮中间量，供串口、调试器或日志 review 控制链路。 */
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

/**
 * @brief 读取最近一次控制器中间量。
 *
 * debug 中的 isStateValid 为 0 时，说明最近一次控制计算未完整通过。
 */
app_status_t Module_Chassis_Controller_GetDebug(module_chassis_controller_debug_t *debug)
{
    if (debug == NULL)
    {
        return APP_STATUS_INVALID_PARAM;
    }

    *debug = chassisControllerState.debug;

    return APP_STATUS_OK;
}
