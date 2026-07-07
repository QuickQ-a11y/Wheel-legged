#include "QuaternionEKF.h"

#include <math.h>
#include <string.h>

#define ALGORITHM_QUATERNION_EKF_PI 3.14159265358979323846f
#define ALGORITHM_QUATERNION_EKF_TWO_PI 6.28318530717958647692f
#define ALGORITHM_QUATERNION_EKF_EPSILON 1.0e-6f

/**
 * @brief 限制浮点值范围。
 */
static float Algorithm_QuaternionEKF_Limit(float value, float minValue, float maxValue)
{
    if (value > maxValue)
    {
        return maxValue;
    }

    if (value < minValue)
    {
        return minValue;
    }

    return value;
}

/**
 * @brief 把矩阵清零并写入单位阵。
 */
static void Algorithm_QuaternionEKF_SetIdentity(float *matrix, uint32_t matrixSize)
{
    uint32_t rowIndex;
    uint32_t columnIndex;

    memset(matrix, 0, sizeof(float) * matrixSize * matrixSize);
    for (rowIndex = 0U; rowIndex < matrixSize; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < matrixSize; columnIndex++)
        {
            if (rowIndex == columnIndex)
            {
                matrix[(rowIndex * matrixSize) + columnIndex] = 1.0f;
            }
        }
    }
}

/**
 * @brief 四元数归一化，避免积分和量测更新后模长偏离 1。
 */
static void Algorithm_QuaternionEKF_NormalizeQuaternion(float quaternion[4])
{
    float norm;
    float invNorm;
    uint32_t index;

    norm = sqrtf((quaternion[0] * quaternion[0]) +
                 (quaternion[1] * quaternion[1]) +
                 (quaternion[2] * quaternion[2]) +
                 (quaternion[3] * quaternion[3]));
    if (norm <= ALGORITHM_QUATERNION_EKF_EPSILON)
    {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }

    invNorm = 1.0f / norm;
    for (index = 0U; index < 4U; index++)
    {
        quaternion[index] *= invNorm;
    }
}

/**
 * @brief 用初始 roll/pitch/yaw 生成四元数。
 */
static void Algorithm_QuaternionEKF_EulerToQuaternion(float rollRad,
                                                      float pitchRad,
                                                      float yawRad,
                                                      float quaternion[4])
{
    float halfRoll = rollRad * 0.5f;
    float halfPitch = pitchRad * 0.5f;
    float halfYaw = yawRad * 0.5f;
    float cosRoll = cosf(halfRoll);
    float sinRoll = sinf(halfRoll);
    float cosPitch = cosf(halfPitch);
    float sinPitch = sinf(halfPitch);
    float cosYaw = cosf(halfYaw);
    float sinYaw = sinf(halfYaw);

    quaternion[0] = (cosRoll * cosPitch * cosYaw) +
                    (sinRoll * sinPitch * sinYaw);
    quaternion[1] = (sinRoll * cosPitch * cosYaw) -
                    (cosRoll * sinPitch * sinYaw);
    quaternion[2] = (cosRoll * sinPitch * cosYaw) +
                    (sinRoll * cosPitch * sinYaw);
    quaternion[3] = (cosRoll * cosPitch * sinYaw) -
                    (sinRoll * sinPitch * cosYaw);
    Algorithm_QuaternionEKF_NormalizeQuaternion(quaternion);
}

/**
 * @brief 第一帧有效加速度给 roll/pitch 初值，yaw 没有绝对观测所以置 0。
 */
static void Algorithm_QuaternionEKF_InitAttitudeFromAccel(
    algorithm_quaternion_ekf_t *filter,
    const float accMps2[3])
{
    float rollRad;
    float pitchRad;

    rollRad = atan2f(accMps2[1], accMps2[2]);
    pitchRad = atan2f(-accMps2[0],
                      sqrtf((accMps2[1] * accMps2[1]) +
                            (accMps2[2] * accMps2[2])));
    Algorithm_QuaternionEKF_EulerToQuaternion(rollRad,
                                              pitchRad,
                                              0.0f,
                                              filter->quaternion);
    memcpy(filter->filteredAccMps2,
           accMps2,
           sizeof(filter->filteredAccMps2));
    filter->rollRad = rollRad;
    filter->pitchRad = pitchRad;
    filter->yawRad = 0.0f;
    filter->yawTotalRad = 0.0f;
    filter->yawLastRad = 0.0f;
    filter->yawRoundCount = 0;
    filter->isInitialized = 1U;
}

/**
 * @brief 按 SPR 的做法建立 6 状态转移矩阵。
 *
 * 状态为 q0,q1,q2,q3,biasX,biasY。Z 轴零偏无法由加速度观测，故不进入状态量。
 */
static void Algorithm_QuaternionEKF_BuildStateTransition(
    algorithm_quaternion_ekf_t *filter,
    float dtSec)
{
    const float *quaternion = filter->quaternion;
    const float *gyro = filter->filteredGyroRadps;
    float halfGxDt = 0.5f * gyro[0] * dtSec;
    float halfGyDt = 0.5f * gyro[1] * dtSec;
    float halfGzDt = 0.5f * gyro[2] * dtSec;
    float halfDt = 0.5f * dtSec;
    float *stateTransition = filter->stateTransition;

    Algorithm_QuaternionEKF_SetIdentity(stateTransition,
                                        ALGORITHM_QUATERNION_EKF_STATE_COUNT);

    stateTransition[1] = -halfGxDt;
    stateTransition[2] = -halfGyDt;
    stateTransition[3] = -halfGzDt;

    stateTransition[6] = halfGxDt;
    stateTransition[8] = halfGzDt;
    stateTransition[9] = -halfGyDt;

    stateTransition[12] = halfGyDt;
    stateTransition[13] = -halfGzDt;
    stateTransition[15] = halfGxDt;

    stateTransition[18] = halfGzDt;
    stateTransition[19] = halfGyDt;
    stateTransition[20] = -halfGxDt;

    stateTransition[4] = quaternion[1] * halfDt;
    stateTransition[5] = quaternion[2] * halfDt;
    stateTransition[10] = -quaternion[0] * halfDt;
    stateTransition[11] = quaternion[3] * halfDt;
    stateTransition[16] = -quaternion[3] * halfDt;
    stateTransition[17] = -quaternion[0] * halfDt;
    stateTransition[22] = quaternion[2] * halfDt;
    stateTransition[23] = -quaternion[1] * halfDt;
}

/**
 * @brief 预测四元数状态。
 */
static void Algorithm_QuaternionEKF_PredictQuaternion(
    algorithm_quaternion_ekf_t *filter,
    const float gyroRadps[3],
    float dtSec)
{
    float q0 = filter->quaternion[0];
    float q1 = filter->quaternion[1];
    float q2 = filter->quaternion[2];
    float q3 = filter->quaternion[3];

    filter->gyroBiasRadps[2] = 0.0f;
    filter->filteredGyroRadps[0] = gyroRadps[0] - filter->gyroBiasRadps[0];
    filter->filteredGyroRadps[1] = gyroRadps[1] - filter->gyroBiasRadps[1];
    filter->filteredGyroRadps[2] = gyroRadps[2];

    Algorithm_QuaternionEKF_BuildStateTransition(filter, dtSec);

    filter->quaternion[0] +=
        0.5f * (-q1 * filter->filteredGyroRadps[0] -
                q2 * filter->filteredGyroRadps[1] -
                q3 * filter->filteredGyroRadps[2]) * dtSec;
    filter->quaternion[1] +=
        0.5f * (q0 * filter->filteredGyroRadps[0] +
                q2 * filter->filteredGyroRadps[2] -
                q3 * filter->filteredGyroRadps[1]) * dtSec;
    filter->quaternion[2] +=
        0.5f * (q0 * filter->filteredGyroRadps[1] -
                q1 * filter->filteredGyroRadps[2] +
                q3 * filter->filteredGyroRadps[0]) * dtSec;
    filter->quaternion[3] +=
        0.5f * (q0 * filter->filteredGyroRadps[2] +
                q1 * filter->filteredGyroRadps[1] -
                q2 * filter->filteredGyroRadps[0]) * dtSec;
    Algorithm_QuaternionEKF_NormalizeQuaternion(filter->quaternion);
}

/**
 * @brief 预测协方差 P' = FPF^T + Q。
 */
static void Algorithm_QuaternionEKF_PredictCovariance(
    algorithm_quaternion_ekf_t *filter,
    float dtSec)
{
    uint32_t rowIndex;
    uint32_t columnIndex;
    uint32_t innerIndex;
    float processNoise;
    float sum;

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
            {
                sum += filter->stateTransition[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                       filter->covariance[(innerIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex];
            }
            filter->tempStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] = sum;
        }
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
            {
                sum += filter->tempStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                       filter->stateTransition[(columnIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex];
            }

            processNoise = 0.0f;
            if (rowIndex == columnIndex)
            {
                processNoise = (rowIndex < ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT) ?
                                   filter->config.quaternionProcessNoise * dtSec :
                                   filter->config.gyroBiasProcessNoise * dtSec;
            }
            filter->covariancePredict[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] =
                sum + processNoise;
        }
    }
}

/**
 * @brief 根据四元数计算预测的单位重力方向和 H 矩阵。
 */
static void Algorithm_QuaternionEKF_BuildMeasurementModel(
    algorithm_quaternion_ekf_t *filter)
{
    float q0 = filter->quaternion[0];
    float q1 = filter->quaternion[1];
    float q2 = filter->quaternion[2];
    float q3 = filter->quaternion[3];

    filter->predictedMeasurement[0] = 2.0f * ((q1 * q3) - (q0 * q2));
    filter->predictedMeasurement[1] = 2.0f * ((q0 * q1) + (q2 * q3));
    filter->predictedMeasurement[2] =
        (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

    memset(filter->measurementMatrix, 0, sizeof(filter->measurementMatrix));
    filter->measurementMatrix[0] = -2.0f * q2;
    filter->measurementMatrix[1] = 2.0f * q3;
    filter->measurementMatrix[2] = -2.0f * q0;
    filter->measurementMatrix[3] = 2.0f * q1;

    filter->measurementMatrix[6] = 2.0f * q1;
    filter->measurementMatrix[7] = 2.0f * q0;
    filter->measurementMatrix[8] = 2.0f * q3;
    filter->measurementMatrix[9] = 2.0f * q2;

    filter->measurementMatrix[12] = 2.0f * q0;
    filter->measurementMatrix[13] = -2.0f * q1;
    filter->measurementMatrix[14] = -2.0f * q2;
    filter->measurementMatrix[15] = 2.0f * q3;
}

/**
 * @brief 计算 3x3 矩阵逆矩阵。
 */
static void Algorithm_QuaternionEKF_Invert3x3(const float matrix[9],
                                              float inverse[9],
                                              float *determinant)
{
    *determinant =
        (matrix[0] * ((matrix[4] * matrix[8]) - (matrix[5] * matrix[7]))) -
        (matrix[1] * ((matrix[3] * matrix[8]) - (matrix[5] * matrix[6]))) +
        (matrix[2] * ((matrix[3] * matrix[7]) - (matrix[4] * matrix[6])));

    if (fabsf(*determinant) <= ALGORITHM_QUATERNION_EKF_EPSILON)
    {
        memset(inverse, 0, sizeof(float) * 9U);
        return;
    }

    inverse[0] = ((matrix[4] * matrix[8]) - (matrix[5] * matrix[7])) / *determinant;
    inverse[1] = ((matrix[2] * matrix[7]) - (matrix[1] * matrix[8])) / *determinant;
    inverse[2] = ((matrix[1] * matrix[5]) - (matrix[2] * matrix[4])) / *determinant;
    inverse[3] = ((matrix[5] * matrix[6]) - (matrix[3] * matrix[8])) / *determinant;
    inverse[4] = ((matrix[0] * matrix[8]) - (matrix[2] * matrix[6])) / *determinant;
    inverse[5] = ((matrix[2] * matrix[3]) - (matrix[0] * matrix[5])) / *determinant;
    inverse[6] = ((matrix[3] * matrix[7]) - (matrix[4] * matrix[6])) / *determinant;
    inverse[7] = ((matrix[1] * matrix[6]) - (matrix[0] * matrix[7])) / *determinant;
    inverse[8] = ((matrix[0] * matrix[4]) - (matrix[1] * matrix[3])) / *determinant;
}

/**
 * @brief 使用加速度方向进行 EKF 量测更新。
 */
static void Algorithm_QuaternionEKF_CorrectByAccel(algorithm_quaternion_ekf_t *filter,
                                                   float gyroNormRadps,
                                                   float accelNormMps2)
{
    uint32_t rowIndex;
    uint32_t columnIndex;
    uint32_t innerIndex;
    uint32_t measurementIndex;
    float determinant;
    float baseGain;
    float stateDelta[ALGORITHM_QUATERNION_EKF_STATE_COUNT] = {0.0f};
    float correctionLimit;
    float sum;

    if ((accelNormMps2 < filter->config.accelNormMinMps2) ||
        (accelNormMps2 > filter->config.accelNormMaxMps2))
    {
        memcpy(filter->covariance,
               filter->covariancePredict,
               sizeof(filter->covariance));
        return;
    }

    Algorithm_QuaternionEKF_BuildMeasurementModel(filter);

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
            {
                sum += filter->measurementMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                       filter->covariancePredict[(innerIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex];
            }
            filter->tempMeasurementStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] = sum;
        }
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
            {
                sum += filter->tempMeasurementStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                       filter->measurementMatrix[(columnIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex];
            }
            if (rowIndex == columnIndex)
            {
                sum += filter->config.accelMeasurementNoise;
            }
            filter->innovationCovariance[(rowIndex * ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT) + columnIndex] = sum;
        }
    }

    Algorithm_QuaternionEKF_Invert3x3(filter->innovationCovariance,
                                      filter->innovationCovarianceInverse,
                                      &determinant);
    if (fabsf(determinant) <= ALGORITHM_QUATERNION_EKF_EPSILON)
    {
        memcpy(filter->covariance,
               filter->covariancePredict,
               sizeof(filter->covariance));
        return;
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (measurementIndex = 0U; measurementIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; measurementIndex++)
        {
            sum = 0.0f;
            for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; columnIndex++)
            {
                baseGain = 0.0f;
                for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
                {
                    baseGain += filter->covariancePredict[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                                filter->measurementMatrix[(columnIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex];
                }
                sum += baseGain *
                       filter->innovationCovarianceInverse[(columnIndex * ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT) + measurementIndex];
            }
            filter->kalmanGain[(rowIndex * ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT) + measurementIndex] = sum;
        }
    }

    for (measurementIndex = 0U; measurementIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; measurementIndex++)
    {
        filter->innovation[measurementIndex] =
            filter->measurement[measurementIndex] -
            filter->predictedMeasurement[measurementIndex];
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (measurementIndex = 0U; measurementIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; measurementIndex++)
        {
            stateDelta[rowIndex] +=
                filter->kalmanGain[(rowIndex * ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT) + measurementIndex] *
                filter->innovation[measurementIndex];
        }
    }

    /*
     * 加速度只提供重力方向，不能观测绝对 yaw。
     * 这里沿用 SPR 的处理：不让量测更新直接修正 q3 和 Z 轴零偏。
     */
    stateDelta[3] = 0.0f;
    if (gyroNormRadps > filter->config.gyroStableThresholdRadps)
    {
        stateDelta[4] = 0.0f;
        stateDelta[5] = 0.0f;
    }
    else
    {
        correctionLimit = filter->config.gyroBiasCorrectionLimitRadps;
        stateDelta[4] =
            Algorithm_QuaternionEKF_Limit(stateDelta[4],
                                          -correctionLimit,
                                          correctionLimit);
        stateDelta[5] =
            Algorithm_QuaternionEKF_Limit(stateDelta[5],
                                          -correctionLimit,
                                          correctionLimit);
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT; rowIndex++)
    {
        filter->quaternion[rowIndex] += stateDelta[rowIndex];
    }
    filter->gyroBiasRadps[0] += stateDelta[4];
    filter->gyroBiasRadps[1] += stateDelta[5];
    filter->gyroBiasRadps[2] = 0.0f;
    Algorithm_QuaternionEKF_NormalizeQuaternion(filter->quaternion);

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (measurementIndex = 0U; measurementIndex < ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT; measurementIndex++)
            {
                sum += filter->kalmanGain[(rowIndex * ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT) + measurementIndex] *
                       filter->measurementMatrix[(measurementIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex];
            }
            filter->tempStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] = sum;
        }
    }

    for (rowIndex = 0U; rowIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; columnIndex++)
        {
            sum = 0.0f;
            for (innerIndex = 0U; innerIndex < ALGORITHM_QUATERNION_EKF_STATE_COUNT; innerIndex++)
            {
                sum += filter->tempStateMatrix[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + innerIndex] *
                       filter->covariancePredict[(innerIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex];
            }
            filter->covariance[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] =
                filter->covariancePredict[(rowIndex * ALGORITHM_QUATERNION_EKF_STATE_COUNT) + columnIndex] - sum;
        }
    }
}

/**
 * @brief 输出欧拉角，并维护连续 yaw。
 */
static void Algorithm_QuaternionEKF_UpdateEuler(algorithm_quaternion_ekf_t *filter)
{
    float q0 = filter->quaternion[0];
    float q1 = filter->quaternion[1];
    float q2 = filter->quaternion[2];
    float q3 = filter->quaternion[3];
    float pitchSin;
    float yawDelta;

    filter->rollRad = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                             (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3));
    pitchSin = 2.0f * ((q0 * q2) - (q1 * q3));
    pitchSin = Algorithm_QuaternionEKF_Limit(pitchSin, -1.0f, 1.0f);
    filter->pitchRad = asinf(pitchSin);
    filter->yawRad = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                            (q0 * q0) + (q1 * q1) - (q2 * q2) - (q3 * q3));

    if (filter->updateCount > 0U)
    {
        yawDelta = filter->yawRad - filter->yawLastRad;
        if (yawDelta > ALGORITHM_QUATERNION_EKF_PI)
        {
            filter->yawRoundCount--;
        }
        else if (yawDelta < -ALGORITHM_QUATERNION_EKF_PI)
        {
            filter->yawRoundCount++;
        }
    }

    filter->yawTotalRad =
        filter->yawRad +
        ((float)filter->yawRoundCount * ALGORITHM_QUATERNION_EKF_TWO_PI);
    filter->yawLastRad = filter->yawRad;
}

void Algorithm_QuaternionEKF_Init(algorithm_quaternion_ekf_t *filter,
                                  const algorithm_quaternion_ekf_config_t *config)
{
    memset(filter, 0, sizeof(*filter));
    filter->config = *config;
    filter->quaternion[0] = 1.0f;

    Algorithm_QuaternionEKF_SetIdentity(filter->covariance,
                                        ALGORITHM_QUATERNION_EKF_STATE_COUNT);
    filter->covariance[0] = config->quaternionInitialCovariance;
    filter->covariance[7] = config->quaternionInitialCovariance;
    filter->covariance[14] = config->quaternionInitialCovariance;
    filter->covariance[21] = config->quaternionInitialCovariance;
    filter->covariance[28] = config->gyroBiasInitialCovariance;
    filter->covariance[35] = config->gyroBiasInitialCovariance;
}

void Algorithm_QuaternionEKF_Update(algorithm_quaternion_ekf_t *filter,
                                    const float gyroRadps[3],
                                    const float accMps2[3],
                                    float dtSec)
{
    float accelNormMps2;
    float accelInvNorm;
    float gyroNormRadps;
    float filterRatio;
    uint32_t axis;

    if (dtSec <= 0.0f)
    {
        return;
    }

    if (filter->isInitialized == 0U)
    {
        Algorithm_QuaternionEKF_InitAttitudeFromAccel(filter, accMps2);
    }

    if (filter->config.accelLpfTimeSec <= 0.0f)
    {
        filterRatio = 1.0f;
    }
    else
    {
        filterRatio = dtSec / (filter->config.accelLpfTimeSec + dtSec);
    }

    for (axis = 0U; axis < ALGORITHM_QUATERNION_EKF_AXIS_COUNT; axis++)
    {
        filter->filteredAccMps2[axis] =
            (filterRatio * accMps2[axis]) +
            ((1.0f - filterRatio) * filter->filteredAccMps2[axis]);
    }

    accelNormMps2 = sqrtf((filter->filteredAccMps2[0] * filter->filteredAccMps2[0]) +
                          (filter->filteredAccMps2[1] * filter->filteredAccMps2[1]) +
                          (filter->filteredAccMps2[2] * filter->filteredAccMps2[2]));
    if (accelNormMps2 > ALGORITHM_QUATERNION_EKF_EPSILON)
    {
        accelInvNorm = 1.0f / accelNormMps2;
        for (axis = 0U; axis < ALGORITHM_QUATERNION_EKF_AXIS_COUNT; axis++)
        {
            filter->measurement[axis] = filter->filteredAccMps2[axis] * accelInvNorm;
        }
    }

    Algorithm_QuaternionEKF_PredictQuaternion(filter, gyroRadps, dtSec);
    Algorithm_QuaternionEKF_PredictCovariance(filter, dtSec);

    gyroNormRadps = sqrtf((filter->filteredGyroRadps[0] * filter->filteredGyroRadps[0]) +
                          (filter->filteredGyroRadps[1] * filter->filteredGyroRadps[1]) +
                          (filter->filteredGyroRadps[2] * filter->filteredGyroRadps[2]));
    if (accelNormMps2 > ALGORITHM_QUATERNION_EKF_EPSILON)
    {
        Algorithm_QuaternionEKF_CorrectByAccel(filter,
                                               gyroNormRadps,
                                               accelNormMps2);
    }
    else
    {
        memcpy(filter->covariance,
               filter->covariancePredict,
               sizeof(filter->covariance));
    }

    filter->filteredGyroRadps[0] = gyroRadps[0] - filter->gyroBiasRadps[0];
    filter->filteredGyroRadps[1] = gyroRadps[1] - filter->gyroBiasRadps[1];
    filter->filteredGyroRadps[2] = gyroRadps[2];
    Algorithm_QuaternionEKF_UpdateEuler(filter);
    filter->updateCount++;
}

void Algorithm_QuaternionEKF_BodyToEarth(const float bodyVector[3],
                                         const float quaternion[4],
                                         float earthVector[3])
{
    float q0 = quaternion[0];
    float q1 = quaternion[1];
    float q2 = quaternion[2];
    float q3 = quaternion[3];

    earthVector[0] =
        (2.0f * ((0.5f - (q2 * q2) - (q3 * q3)) * bodyVector[0])) +
        (2.0f * (((q1 * q2) - (q0 * q3)) * bodyVector[1])) +
        (2.0f * (((q1 * q3) + (q0 * q2)) * bodyVector[2]));
    earthVector[1] =
        (2.0f * (((q1 * q2) + (q0 * q3)) * bodyVector[0])) +
        (2.0f * ((0.5f - (q1 * q1) - (q3 * q3)) * bodyVector[1])) +
        (2.0f * (((q2 * q3) - (q0 * q1)) * bodyVector[2]));
    earthVector[2] =
        (2.0f * (((q1 * q3) - (q0 * q2)) * bodyVector[0])) +
        (2.0f * (((q2 * q3) + (q0 * q1)) * bodyVector[1])) +
        (2.0f * ((0.5f - (q1 * q1) - (q2 * q2)) * bodyVector[2]));
}

void Algorithm_QuaternionEKF_EarthToBody(const float earthVector[3],
                                         const float quaternion[4],
                                         float bodyVector[3])
{
    float q0 = quaternion[0];
    float q1 = quaternion[1];
    float q2 = quaternion[2];
    float q3 = quaternion[3];

    bodyVector[0] =
        (2.0f * ((0.5f - (q2 * q2) - (q3 * q3)) * earthVector[0])) +
        (2.0f * (((q1 * q2) + (q0 * q3)) * earthVector[1])) +
        (2.0f * (((q1 * q3) - (q0 * q2)) * earthVector[2]));
    bodyVector[1] =
        (2.0f * (((q1 * q2) - (q0 * q3)) * earthVector[0])) +
        (2.0f * ((0.5f - (q1 * q1) - (q3 * q3)) * earthVector[1])) +
        (2.0f * (((q2 * q3) + (q0 * q1)) * earthVector[2]));
    bodyVector[2] =
        (2.0f * (((q1 * q3) + (q0 * q2)) * earthVector[0])) +
        (2.0f * (((q2 * q3) - (q0 * q1)) * earthVector[1])) +
        (2.0f * ((0.5f - (q1 * q1) - (q2 * q2)) * earthVector[2]));
}
