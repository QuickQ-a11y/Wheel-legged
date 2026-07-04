#include "task_imu.h"

#include "app_config.h"
#include "main.h"
#include "spi.h"

#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

#define TASK_IMU_GRAVITY_MPS2 9.80665f

static osThreadId_t imuTaskHandle;
static osMutexId_t imuStateMutex;
static bmi088_t imuBmi088;
task_imu_state_t imuTaskDebugState;

static const osThreadAttr_t imuTaskAttributes = {
    .name = "ImuTask",
    .stack_size = 768U * 4U,
    .priority = (osPriority_t)osPriorityRealtime,
};

static const osMutexAttr_t imuStateMutexAttributes = {
    .name = "ImuStateMutex",
};

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

    if ((state == NULL) || (gyroSum == NULL) || (biasSampleCount == NULL))
    {
        return;
    }

    if (*biasSampleCount >= APP_CONFIG_IMU_BIAS_SAMPLE_COUNT)
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
        (*biasSampleCount >= APP_CONFIG_IMU_BIAS_SAMPLE_COUNT) ? 1U : 0U;
}

/**
 * @brief 用陀螺积分和加速度低频修正更新基础姿态。
 *
 * yaw 当前没有磁力计或外部观测，只做陀螺积分，不能作为长期绝对航向。
 */
static void IMU_Task_UpdateAttitude(task_imu_state_t *state,
                                    uint32_t *lastUpdateTick)
{
    const bmi088_data_t *data;
    uint32_t nowTick;
    float dtSec;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accRoll;
    float accPitch;

    if ((state == NULL) || (lastUpdateTick == NULL) || (state->isAttitudeReady == 0U))
    {
        return;
    }

    data = &state->bmi088Data;
    nowTick = data->lastUpdateTick;
    if (*lastUpdateTick == 0U)
    {
        *lastUpdateTick = nowTick;
        state->dtSec = APP_CONFIG_IMU_DEFAULT_DT_SEC;
        return;
    }

    dtSec = (float)(nowTick - *lastUpdateTick) * 0.001f;
    if ((dtSec <= 0.0f) || (dtSec > 0.05f))
    {
        dtSec = APP_CONFIG_IMU_DEFAULT_DT_SEC;
    }
    *lastUpdateTick = nowTick;
    state->dtSec = dtSec;

    gyroX = data->gyroRadps[0] - state->gyroBiasRadps[0];
    gyroY = data->gyroRadps[1] - state->gyroBiasRadps[1];
    gyroZ = data->gyroRadps[2] - state->gyroBiasRadps[2];

    accRoll = atan2f(data->accMps2[1], data->accMps2[2]);
    accPitch = atan2f(-data->accMps2[0],
                      sqrtf((data->accMps2[1] * data->accMps2[1]) +
                            (data->accMps2[2] * data->accMps2[2])));

    state->rollRad =
        (APP_CONFIG_IMU_COMPLEMENTARY_ALPHA * (state->rollRad + gyroX * dtSec)) +
        ((1.0f - APP_CONFIG_IMU_COMPLEMENTARY_ALPHA) * accRoll);
    state->pitchRad =
        (APP_CONFIG_IMU_COMPLEMENTARY_ALPHA * (state->pitchRad + gyroY * dtSec)) +
        ((1.0f - APP_CONFIG_IMU_COMPLEMENTARY_ALPHA) * accPitch);
    state->yawRad += gyroZ * dtSec;
}

/**
 * @brief 计算自然坐标系下的运动加速度。
 *
 * 输入来自 BMI088 原始加速度和当前互补滤波姿态；输出写入 state->motionAccMps2。
 * 流程参考 SPR：先用姿态估计出机体系重力分量，扣除重力得到机体运动加速度，
 * 再旋转到自然坐标系，供底盘卡尔曼速度融合使用。
 */
static void IMU_Task_UpdateMotionAcceleration(task_imu_state_t *state)
{
    const bmi088_data_t *data = &state->bmi088Data;
    float sinRoll;
    float cosRoll;
    float sinPitch;
    float cosPitch;
    float sinYaw;
    float cosYaw;
    float gravityBody[BMI088_AXIS_COUNT];
    float motionAccBody[BMI088_AXIS_COUNT];
    float motionAccEarth[BMI088_AXIS_COUNT];
    float filterRatio;
    uint32_t axis;

    if (state->isAttitudeReady == 0U)
    {
        return;
    }

    sinRoll = sinf(state->rollRad);
    cosRoll = cosf(state->rollRad);
    sinPitch = sinf(state->pitchRad);
    cosPitch = cosf(state->pitchRad);
    sinYaw = sinf(state->yawRad);
    cosYaw = cosf(state->yawRad);

    /*
     * 静止时加速度计主要测到重力，当前姿态定义下：
     * accX = -g * sin(pitch)
     * accY =  g * sin(roll) * cos(pitch)
     * accZ =  g * cos(roll) * cos(pitch)
     */
    gravityBody[0] = -TASK_IMU_GRAVITY_MPS2 * sinPitch;
    gravityBody[1] = TASK_IMU_GRAVITY_MPS2 * sinRoll * cosPitch;
    gravityBody[2] = TASK_IMU_GRAVITY_MPS2 * cosRoll * cosPitch;

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        motionAccBody[axis] = data->accMps2[axis] - gravityBody[axis];
    }

    /*
     * R = Rz(yaw) * Ry(pitch) * Rx(roll)，把机体系运动加速度转到自然坐标系。
     * 当前 yaw 只由陀螺积分得到，长期会漂移；调试时如只关心机体前向，
     * 可在 module_chassis_model.c 中调整前向加速度的轴向映射。
     */
    motionAccEarth[0] =
        (cosYaw * cosPitch * motionAccBody[0]) +
        (((cosYaw * sinPitch * sinRoll) - (sinYaw * cosRoll)) *
         motionAccBody[1]) +
        (((cosYaw * sinPitch * cosRoll) + (sinYaw * sinRoll)) *
         motionAccBody[2]);
    motionAccEarth[1] =
        (sinYaw * cosPitch * motionAccBody[0]) +
        (((sinYaw * sinPitch * sinRoll) + (cosYaw * cosRoll)) *
         motionAccBody[1]) +
        (((sinYaw * sinPitch * cosRoll) - (cosYaw * sinRoll)) *
         motionAccBody[2]);
    motionAccEarth[2] =
        (-sinPitch * motionAccBody[0]) +
        (cosPitch * sinRoll * motionAccBody[1]) +
        (cosPitch * cosRoll * motionAccBody[2]);

    if ((APP_CONFIG_IMU_MOTION_ACCEL_LPF_TIME_SEC <= 0.0f) ||
        (state->dtSec <= 0.0f))
    {
        filterRatio = 1.0f;
    }
    else
    {
        filterRatio =
            state->dtSec / (APP_CONFIG_IMU_MOTION_ACCEL_LPF_TIME_SEC + state->dtSec);
    }

    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        state->motionAccMps2[axis] =
            (filterRatio * motionAccEarth[axis]) +
            ((1.0f - filterRatio) * state->motionAccMps2[axis]);
    }
}

/**
 * @brief 保存 IMU 状态快照。
 */
static void IMU_Task_SaveState(const task_imu_state_t *state)
{
    if ((state == NULL) || (imuStateMutex == NULL))
    {
        return;
    }

    if (osMutexAcquire(imuStateMutex, osWaitForever) != osOK)
    {
        return;
    }

    imuTaskDebugState = *state;
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
        .timeoutMs = APP_CONFIG_IMU_SPI_TIMEOUT_MS,
    };

    return config;
}

static void ImuTask(void *argument)
{
    const bmi088_config_t bmi088Config = IMU_Task_GetBmi088Config();
    task_imu_state_t localState = {0};
    float gyroSum[BMI088_AXIS_COUNT] = {0.0f};
    uint32_t biasSampleCount = 0U;
    uint32_t attitudeLastTick = 0U;
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        if (localState.isInitialized == 0U)
        {
            BMI088_Init(&imuBmi088, &bmi088Config);
            localState.lastErrorCode = BMI088_GetErrorCode(&imuBmi088);
            /*
             * 初始化失败时也保留实际读回的芯片 ID，便于 Watch 判断供电、
             * SPI 片选或读写时序问题。
             */
            localState.bmi088Data = imuBmi088.data;
            if (localState.lastErrorCode != BMI088_ERROR_NONE)
            {
                localState.initErrorCount++;
                IMU_Task_SaveState(&localState);
                (void)osDelay(APP_CONFIG_IMU_INIT_RETRY_TICKS);
                wakeTick = osKernelGetTickCount();
                continue;
            }

            localState.isInitialized = 1U;
            localState.lastErrorCode = BMI088_ERROR_NONE;
            memset(gyroSum, 0, sizeof(gyroSum));
            biasSampleCount = 0U;
            attitudeLastTick = 0U;
        }

        BMI088_Read(&imuBmi088, &localState.bmi088Data);
        localState.lastErrorCode = BMI088_GetErrorCode(&imuBmi088);
        if (localState.lastErrorCode != BMI088_ERROR_NONE)
        {
            localState.readErrorCount++;
        }
        else
        {
            IMU_Task_UpdateGyroBias(&localState, gyroSum, &biasSampleCount);
            IMU_Task_UpdateAttitude(&localState, &attitudeLastTick);
            IMU_Task_UpdateMotionAcceleration(&localState);
        }

        IMU_Task_SaveState(&localState);

        wakeTick += APP_CONFIG_IMU_TASK_PERIOD_TICKS;
        (void)osDelayUntil(wakeTick);
    }
}

void IMU_Task_Init(void)
{
    imuStateMutex = osMutexNew(&imuStateMutexAttributes);
    if (imuStateMutex == NULL)
    {
        return;
    }

    memset(&imuTaskDebugState, 0, sizeof(imuTaskDebugState));
    imuTaskHandle = osThreadNew(ImuTask, NULL, &imuTaskAttributes);
}

void IMU_Task_GetState(task_imu_state_t *state)
{
    if ((state == NULL) || (imuStateMutex == NULL))
    {
        return;
    }

    if (osMutexAcquire(imuStateMutex, osWaitForever) != osOK)
    {
        return;
    }

    *state = imuTaskDebugState;
    (void)osMutexRelease(imuStateMutex);
}
