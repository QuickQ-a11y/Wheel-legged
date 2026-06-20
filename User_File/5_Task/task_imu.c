#include "task_imu.h"

#include "app_config.h"
#include "main.h"
#include "spi.h"

#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

static osThreadId_t imuTaskHandle;
static osMutexId_t imuStateMutex;
static bmi088_t imuBmi088;
static task_imu_state_t imuTaskState;

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

    imuTaskState = *state;
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
            if (BMI088_Init(&imuBmi088, &bmi088Config) != APP_STATUS_OK)
            {
                localState.initErrorCount++;
                localState.lastErrorCode = BMI088_GetErrorCode(&imuBmi088);
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

        if (BMI088_Read(&imuBmi088, &localState.bmi088Data) != APP_STATUS_OK)
        {
            localState.readErrorCount++;
            localState.lastErrorCode = BMI088_GetErrorCode(&imuBmi088);
        }
        else
        {
            localState.lastErrorCode = BMI088_ERROR_NONE;
            IMU_Task_UpdateGyroBias(&localState, gyroSum, &biasSampleCount);
            IMU_Task_UpdateAttitude(&localState, &attitudeLastTick);
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

    memset(&imuTaskState, 0, sizeof(imuTaskState));
    imuTaskHandle = osThreadNew(ImuTask, NULL, &imuTaskAttributes);
}

app_status_t IMU_Task_GetState(task_imu_state_t *state)
{
    if ((state == NULL) || (imuStateMutex == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    if (osMutexAcquire(imuStateMutex, osWaitForever) != osOK)
    {
        return APP_STATUS_BUSY;
    }

    *state = imuTaskState;
    (void)osMutexRelease(imuStateMutex);

    return APP_STATUS_OK;
}
