#include "task_imu.h"

#include "app_config.h"
#include "main.h"
#include "Limit.h"
#include "PID.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"

#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

#define TASK_IMU_GRAVITY_MPS2 9.80665f

static osThreadId_t imuTaskHandle;
static osMutexId_t imuStateMutex;
static bmi088_t imuBmi088;
task_imu_state_t imuTaskDebugState;

static const algorithm_pid_config_t imuTemperaturePidConfig = {
    .kp = APP_IMU_TEMP_KP,
    .ki = APP_IMU_TEMP_KI,
    .kd = APP_IMU_TEMP_KD,
    .integralLimit = APP_IMU_TEMP_I_LIMIT,
    .outputLimit = APP_IMU_TEMP_PWM_MAX,
};

static const osThreadAttr_t imuTaskAttributes = {
    .name = "ImuTask",
    .stack_size = 768U * 4U,
    .priority = (osPriority_t)osPriorityRealtime,
};

static const osMutexAttr_t imuStateMutexAttributes = {
    .name = "ImuStateMutex",
};

/**
 * @brief 生成 IMU 姿态 EKF 参数。
 *
 * 参数集中来自 app_config.h；后续调实车时优先改配置，不在滤波流程里散落常数。
 */
static algorithm_quaternion_ekf_config_t IMU_Task_GetAttitudeFilterConfig(void)
{
    algorithm_quaternion_ekf_config_t config = {
        .quaternionProcessNoise = APP_IMU_EKF_QUAT_NOISE,
        .gyroBiasProcessNoise = APP_IMU_EKF_BIAS_NOISE,
        .accelMeasurementNoise = APP_IMU_EKF_ACCEL_NOISE,
        .quaternionInitialCovariance =
            APP_IMU_EKF_QUAT_COV,
        .gyroBiasInitialCovariance =
            APP_IMU_EKF_BIAS_COV,
        .accelLpfTimeSec = APP_IMU_EKF_ACCEL_LPF_S,
        .accelNormMinMps2 = APP_IMU_EKF_ACCEL_MIN_MPS2,
        .accelNormMaxMps2 = APP_IMU_EKF_ACCEL_MAX_MPS2,
        .gyroStableThresholdRadps =
            APP_IMU_EKF_GYRO_STABLE_RADPS,
        .gyroBiasCorrectionLimitRadps =
            APP_IMU_EKF_BIAS_CORR_RADPS,
    };

    return config;
}

/**
 * @brief 累计启动阶段陀螺零偏。
 *
 * 当前只在上电静止假设下估计零偏，完成前不标记姿态可用。
 */
static void IMU_Task_UpdateGyroBias(task_imu_state_t *state,
                                    float gyroSum[BMI088_AXIS_COUNT],
                                    uint32_t *biasSampleCount)
{
    uint32_t axis;

    if (*biasSampleCount >= APP_IMU_BIAS_SAMPLES)
    {
        return;
    }

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        gyroSum[axis] += state->bmi088Data.gyroRadps[axis];
        state->gyroBiasRadps[axis] = gyroSum[axis] / (float)(*biasSampleCount + 1U);
    }

    (*biasSampleCount)++;
    state->isAttitudeReady =
        (*biasSampleCount >= APP_IMU_BIAS_SAMPLES) ? 1U : 0U;
}

/**
 * @brief 根据 SPR 的方式控制 BMI088 加热 PWM。
 *
 * 输入来自 BMI088 温度读数和 app_config.h 中的温控参数；输出写入 TIM3_CH4，
 * 同时把目标温度、误差、PID 输出和 PWM 比较值保存在 imuTaskDebugState 快照中。
 */
static void IMU_Task_UpdateTemperatureControl(task_imu_state_t *state,
                                              algorithm_pid_state_t *temperaturePid,
                                              float dtSec)
{
    float pidOutput = 0.0f;
    uint32_t pwmCompare;

    state->temperatureTargetCelsius = APP_IMU_TEMP_TARGET_C;
    state->temperatureErrorCelsius =
        APP_IMU_TEMP_TARGET_C - state->bmi088Data.temperatureCelsius;
    state->isTemperatureStable =
        (fabsf(state->temperatureErrorCelsius) <=
         APP_IMU_TEMP_STABLE_C)
            ? 1U
            : 0U;

    Algorithm_PID_UpdateByFeedbackRate(&imuTemperaturePidConfig,
                                       temperaturePid,
                                       APP_IMU_TEMP_TARGET_C,
                                       state->bmi088Data.temperatureCelsius,
                                       0.0f,
                                       dtSec,
                                       &pidOutput);

    pidOutput = Algorithm_LimitRange(pidOutput,
                                     0.0f,
                                     APP_IMU_TEMP_PWM_MAX);
    state->isTemperatureProtected =
        (state->bmi088Data.temperatureCelsius >=
         APP_IMU_TEMP_PROTECT_C)
            ? 1U
            : 0U;
    if (state->isTemperatureProtected != 0U)
    {
        pidOutput = 0.0f;
        Algorithm_PID_Init(temperaturePid);
    }

    pwmCompare = (uint32_t)pidOutput;
    state->temperaturePidOutput = pidOutput;
    state->temperaturePwmCompare = pwmCompare;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmCompare);
}

/**
 * @brief 用四元数 EKF 更新姿态。
 *
 * 输入来自 BMI088 原始角速度/加速度和启动阶段静态零偏；输出写回 state 的姿态、
 * 四元数、EKF 残余零偏和滤波后角速度。Yaw 没有外部观测，仍不能作为长期绝对航向。
 */
static void IMU_Task_UpdateAttitude(task_imu_state_t *state,
                                    algorithm_quaternion_ekf_t *attitudeFilter,
                                    float dtSec)
{
    const bmi088_data_t *data = &state->bmi088Data;
    float gyroRadps[BMI088_AXIS_COUNT];
    uint32_t axis;

    if (state->isAttitudeReady == 0U)
    {
        return;
    }

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        gyroRadps[axis] = data->gyroRadps[axis] - state->gyroBiasRadps[axis];
    }

    Algorithm_QuaternionEKF_Update(attitudeFilter,
                                   gyroRadps,
                                   data->accMps2,
                                   dtSec);

    memcpy(state->quaternion,
           attitudeFilter->quaternion,
           sizeof(state->quaternion));
    memcpy(state->gyroBiasEkfRadps,
           attitudeFilter->gyroBiasRadps,
           sizeof(state->gyroBiasEkfRadps));
    memcpy(state->filteredGyroRadps,
           attitudeFilter->filteredGyroRadps,
           sizeof(state->filteredGyroRadps));
    state->rollRad = attitudeFilter->rollRad;
    state->pitchRad = attitudeFilter->pitchRad;
    state->yawRad = attitudeFilter->yawRad;
    state->yawTotalRad = attitudeFilter->yawTotalRad;
}

/**
 * @brief 在静止且温度稳定时慢速修正 Z 轴陀螺零偏。
 *
 * SPR 的四元数 EKF 不观测 Z 轴零偏，因为加速度无法约束 yaw；这里不把 Z 轴塞进 EKF，
 * 只在调试和站稳条件明确满足时学习 gyroBiasRadps[2]，降低静止时 yaw 持续漂移。
 */
static void IMU_Task_UpdateStableZBias(task_imu_state_t *state)
{
    float zBiasFilterRatio;

    state->accNormMps2 = sqrtf(
        (state->bmi088Data.accMps2[0] * state->bmi088Data.accMps2[0]) +
        (state->bmi088Data.accMps2[1] * state->bmi088Data.accMps2[1]) +
        (state->bmi088Data.accMps2[2] * state->bmi088Data.accMps2[2]));
    state->gyroNormRadps = sqrtf(
        (state->filteredGyroRadps[0] * state->filteredGyroRadps[0]) +
        (state->filteredGyroRadps[1] * state->filteredGyroRadps[1]) +
        (state->filteredGyroRadps[2] * state->filteredGyroRadps[2]));
    state->zGyroResidualRadps = state->filteredGyroRadps[2];
    state->isZBiasUpdated = 0U;

    if (state->isAttitudeReady == 0U)
    {
        return;
    }

    if ((state->isTemperatureStable == 0U) ||
        (state->accNormMps2 < APP_IMU_EKF_ACCEL_MIN_MPS2) ||
        (state->accNormMps2 > APP_IMU_EKF_ACCEL_MAX_MPS2) ||
        (state->gyroNormRadps > APP_IMU_Z_BIAS_GYRO_MAX_RADPS))
    {
        return;
    }

    /*
     * Z 轴零偏只做慢速一阶学习，不参与姿态 EKF 观测更新。
     * 更新后的零偏会在下一轮姿态积分前扣除。
     */
    zBiasFilterRatio =
        state->dtSec / (APP_IMU_Z_BIAS_LPF_S + state->dtSec);
    zBiasFilterRatio = Algorithm_LimitRange(zBiasFilterRatio, 0.0f, 1.0f);
    state->gyroBiasRadps[2] += zBiasFilterRatio * state->zGyroResidualRadps;
    state->zBiasUpdateCount++;
    state->isZBiasUpdated = 1U;
}

/**
 * @brief 计算 IMU 内部坐标下的运动加速度。
 *
 * 输入来自 BMI088 原始加速度和当前 EKF 四元数；输出写入 state->motionAccMps2。
 * 流程参考 SPR：先用姿态估计出机体系重力分量，扣除重力得到机体运动加速度，
 * 再旋转到内部导航坐标；保存快照前会统一转换为整车右手系。
 */
static void IMU_Task_UpdateMotionAcceleration(task_imu_state_t *state)
{
    const bmi088_data_t *data = &state->bmi088Data;
    const float gravityEarth[BMI088_AXIS_COUNT] = {0.0f, 0.0f, TASK_IMU_GRAVITY_MPS2};
    float gravityBody[BMI088_AXIS_COUNT];
    float motionAccBody[BMI088_AXIS_COUNT];
    float motionAccEarth[BMI088_AXIS_COUNT];
    float filterRatio;
    uint32_t axis;

    if (state->isAttitudeReady == 0U)
    {
        return;
    }

    if ((APP_IMU_ACCEL_LPF_S <= 0.0f) ||
        (state->dtSec <= 0.0f))
    {
        filterRatio = 1.0f;
    }
    else
    {
        filterRatio =
            state->dtSec / (APP_IMU_ACCEL_LPF_S + state->dtSec);
    }

    Algorithm_QuaternionEKF_EarthToBody(gravityEarth,
                                         state->quaternion,
                                         gravityBody);

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        motionAccBody[axis] = data->accMps2[axis] - gravityBody[axis];
        state->bodyMotionAccMps2[axis] =
            (filterRatio * motionAccBody[axis]) +
            ((1.0f - filterRatio) * state->bodyMotionAccMps2[axis]);
    }

    /*
     * 运动加速度先转到滤波器内部导航坐标，保存给业务层前再转成整车右手系。
     * 当前 yaw 仍由陀螺积分主导，长时间运行时 X/Y 方向会随 yaw 漂移。
     */
    Algorithm_QuaternionEKF_BodyToEarth(motionAccBody,
                                         state->quaternion,
                                         motionAccEarth);

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        state->motionAccMps2[axis] =
            (filterRatio * motionAccEarth[axis]) +
            ((1.0f - filterRatio) * state->motionAccMps2[axis]);
    }
}

/**
 * @brief 把 IMU 内部坐标快照转换为整车右手系并发布给业务层。
 *
 * BMI088 原始数据保留传感器坐标，便于排查硬件；姿态角、角速度、运动加速度
 * 和零偏作为业务层输入，统一转换到 X 前、Y 左、Z 上的整车右手系。
 */
static void IMU_Task_PublishState(const task_imu_state_t *internalState)
{
    task_imu_state_t output;
    task_imu_state_t *outputState = &output;

    *outputState = *internalState;

    /* Z 轴镜像时，线加速度是普通向量，仅 Z 分量反号。 */
    outputState->bodyMotionAccMps2[2] =
        -outputState->bodyMotionAccMps2[2];
    outputState->motionAccMps2[2] = -outputState->motionAccMps2[2];

    /*
     * 角速度、欧拉角和陀螺零偏是旋转量；Z 轴镜像后绕 X/Y 轴的正方向取反。
     * quaternion 同步转换为 roll'=-roll, pitch'=-pitch, yaw'=yaw 对应的表示。
     */
    outputState->gyroBiasRadps[0] = -outputState->gyroBiasRadps[0];
    outputState->gyroBiasRadps[1] = -outputState->gyroBiasRadps[1];
    outputState->gyroBiasEkfRadps[0] = -outputState->gyroBiasEkfRadps[0];
    outputState->gyroBiasEkfRadps[1] = -outputState->gyroBiasEkfRadps[1];
    outputState->filteredGyroRadps[0] = -outputState->filteredGyroRadps[0];
    outputState->filteredGyroRadps[1] = -outputState->filteredGyroRadps[1];
    outputState->quaternion[1] = -outputState->quaternion[1];
    outputState->quaternion[2] = -outputState->quaternion[2];
    outputState->rollRad = -outputState->rollRad;
    outputState->pitchRad = -outputState->pitchRad;

    (void)osMutexAcquire(imuStateMutex, osWaitForever);
    imuTaskDebugState = output;
    (void)osMutexRelease(imuStateMutex);
}

/**
 * @brief 生成当前硬件上的 BMI088 配置。
 */
static bmi088_config_t IMU_Task_GetBmi088Config(void)
{
    bmi088_config_t config = {
        .spiHandle = &hspi2,
        .accChipSelect = {
            .gpioPort = ACC_CS_GPIO_Port,
            .gpioPin = ACC_CS_Pin,
        },
        .gyroChipSelect = {
            .gpioPort = GYRO_CS_GPIO_Port,
            .gpioPin = GYRO_CS_Pin,
        },
        .timeoutMs = APP_IMU_SPI_TIMEOUT_MS,
    };

    return config;
}

static void IMU_Task_Entry(void *argument)
{
    const bmi088_config_t bmi088Config = IMU_Task_GetBmi088Config();
    const algorithm_quaternion_ekf_config_t attitudeFilterConfig =
        IMU_Task_GetAttitudeFilterConfig();
    algorithm_quaternion_ekf_t attitudeFilter = {0};
    algorithm_pid_state_t temperaturePid = {0};
    task_imu_state_t localState = {0};
    float gyroSum[BMI088_AXIS_COUNT] = {0.0f};
    const float tickSec = 1.0f / (float)osKernelGetTickFreq();
    float dtSec = APP_CTRL_DT_S;
    uint32_t biasSampleCount = 0U;
    uint32_t sampleLastTick = 0U;
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        if (localState.isInitialized == 0U)
        {
            BMI088_Init(&imuBmi088, &bmi088Config);
            localState.lastErrorCode = imuBmi088.lastErrorCode;
            /*
             * 初始化失败时也保留实际读回的芯片 ID，便于 Watch 判断供电、
             * SPI 片选或读写时序问题。
             */
            localState.bmi088Data = imuBmi088.data;
            if (localState.lastErrorCode != BMI088_ERROR_NONE)
            {
                localState.initErrorCount++;
                IMU_Task_PublishState(&localState);
                (void)osDelay(APP_IMU_INIT_RETRY_TICKS);
                wakeTick = osKernelGetTickCount();
                continue;
            }

            localState.isInitialized = 1U;
            localState.lastErrorCode = BMI088_ERROR_NONE;
            localState.quaternion[0] = 1.0f;
            Algorithm_QuaternionEKF_Init(&attitudeFilter, &attitudeFilterConfig);
            Algorithm_PID_Init(&temperaturePid);
            (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
            memset(gyroSum, 0, sizeof(gyroSum));
            biasSampleCount = 0U;
            sampleLastTick = 0U;
        }

        BMI088_Read(&imuBmi088, &localState.bmi088Data);
        {
            uint32_t sampleTick = osKernelGetTickCount();

            /* IMU滤波、温控PID和零偏更新共用本轮采样的实际dt。 */
            if (sampleLastTick == 0U)
            {
                dtSec = APP_CTRL_DT_S;
            }
            else
            {
                dtSec = (float)(sampleTick - sampleLastTick) * tickSec;
                if ((dtSec <= 0.0f) || (dtSec > 0.05f))
                {
                    dtSec = APP_CTRL_DT_S;
                }
            }
            sampleLastTick = sampleTick;
            localState.dtSec = dtSec;

            IMU_Task_UpdateTemperatureControl(&localState,
                                              &temperaturePid,
                                              dtSec);
            IMU_Task_UpdateGyroBias(&localState, gyroSum, &biasSampleCount);
            IMU_Task_UpdateAttitude(&localState,
                                    &attitudeFilter,
                                    dtSec);
            IMU_Task_UpdateStableZBias(&localState);
            IMU_Task_UpdateMotionAcceleration(&localState);
        }

        IMU_Task_PublishState(&localState);

        wakeTick += APP_CTRL_TICKS;
        if ((int32_t)(osKernelGetTickCount() - wakeTick) >= 0)
        {
            wakeTick = osKernelGetTickCount() + APP_CTRL_TICKS;
        }
        (void)osDelayUntil(wakeTick);
    }
}

void IMU_Task_Init(void)
{
    imuStateMutex = osMutexNew(&imuStateMutexAttributes);
    memset(&imuTaskDebugState, 0, sizeof(imuTaskDebugState));
    imuTaskHandle = osThreadNew(IMU_Task_Entry, NULL, &imuTaskAttributes);
}

void IMU_Task_GetState(task_imu_state_t *state)
{
    (void)osMutexAcquire(imuStateMutex, osWaitForever);

    *state = imuTaskDebugState;
    (void)osMutexRelease(imuStateMutex);
}
