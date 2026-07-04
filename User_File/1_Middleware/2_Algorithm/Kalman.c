#include "Kalman.h"

#include <string.h>

/**
 * @brief 按当前维度设置方阵为单位矩阵。
 */
static void Algorithm_Kalman_SetIdentity(float *matrix, uint8_t matrixSize)
{
    uint32_t rowIndex;
    uint32_t columnIndex;

    for (rowIndex = 0U; rowIndex < matrixSize; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < matrixSize; columnIndex++)
        {
            matrix[(rowIndex * matrixSize) + columnIndex] =
                (rowIndex == columnIndex) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 测量矩阵默认取状态前 measurementCount 个量。
 */
static void Algorithm_Kalman_SetDefaultMeasurementMatrix(algorithm_kalman_t *kalman)
{
    uint32_t rowIndex;
    uint32_t columnIndex;

    memset(kalman->measurementMatrix, 0, sizeof(kalman->measurementMatrix));
    for (rowIndex = 0U; rowIndex < kalman->measurementCount; rowIndex++)
    {
        for (columnIndex = 0U; columnIndex < kalman->stateCount; columnIndex++)
        {
            if (rowIndex == columnIndex)
            {
                kalman->measurementMatrix[(rowIndex * kalman->stateCount) +
                                          columnIndex] = 1.0f;
            }
        }
    }
}

/**
 * @brief 绑定 CMSIS-DSP 矩阵描述符和滤波器内部数组。
 *
 * 参考 SPR 的做法，矩阵描述符只在初始化阶段建立一次，后续 Update 只关注五式流程。
 */
static void Algorithm_Kalman_InitMatrices(algorithm_kalman_t *kalman)
{
    uint16_t stateCount = kalman->stateCount;
    uint16_t measurementCount = kalman->measurementCount;
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    arm_mat_init_f32(&matrix->state,
                     stateCount,
                     1U,
                     kalman->state);
    arm_mat_init_f32(&matrix->statePredict,
                     stateCount,
                     1U,
                     kalman->statePredict);
    arm_mat_init_f32(&matrix->measurement,
                     measurementCount,
                     1U,
                     kalman->measurement);
    arm_mat_init_f32(&matrix->stateTransition,
                     stateCount,
                     stateCount,
                     kalman->stateTransition);
    arm_mat_init_f32(&matrix->stateTransitionTranspose,
                     stateCount,
                     stateCount,
                     kalman->stateTransitionTranspose);
    arm_mat_init_f32(&matrix->measurementMatrix,
                     measurementCount,
                     stateCount,
                     kalman->measurementMatrix);
    arm_mat_init_f32(&matrix->measurementTranspose,
                     stateCount,
                     measurementCount,
                     kalman->measurementTranspose);
    arm_mat_init_f32(&matrix->processNoise,
                     stateCount,
                     stateCount,
                     kalman->processNoise);
    arm_mat_init_f32(&matrix->measurementNoise,
                     measurementCount,
                     measurementCount,
                     kalman->measurementNoise);
    arm_mat_init_f32(&matrix->covariance,
                     stateCount,
                     stateCount,
                     kalman->covariance);
    arm_mat_init_f32(&matrix->covariancePredict,
                     stateCount,
                     stateCount,
                     kalman->covariancePredict);
    arm_mat_init_f32(&matrix->kalmanGain,
                     stateCount,
                     measurementCount,
                     kalman->kalmanGain);
    arm_mat_init_f32(&matrix->innovation,
                     measurementCount,
                     1U,
                     kalman->innovation);
    arm_mat_init_f32(&matrix->predictedMeasurement,
                     measurementCount,
                     1U,
                     kalman->predictedMeasurement);
    arm_mat_init_f32(&matrix->innovationCovariance,
                     measurementCount,
                     measurementCount,
                     kalman->innovationCovariance);
    arm_mat_init_f32(&matrix->innovationCovarianceInverse,
                     measurementCount,
                     measurementCount,
                     kalman->innovationCovarianceInverse);
    arm_mat_init_f32(&matrix->tempStateVector,
                     stateCount,
                     1U,
                     kalman->tempStateVector);
    arm_mat_init_f32(&matrix->tempStateMatrixA,
                     stateCount,
                     stateCount,
                     kalman->tempStateMatrixA);
    arm_mat_init_f32(&matrix->tempStateMatrixB,
                     stateCount,
                     stateCount,
                     kalman->tempStateMatrixB);
    arm_mat_init_f32(&matrix->tempStateMeasurementMatrix,
                     stateCount,
                     measurementCount,
                     kalman->tempStateMeasurementMatrix);
    arm_mat_init_f32(&matrix->tempMeasurementStateMatrix,
                     measurementCount,
                     stateCount,
                     kalman->tempMeasurementStateMatrix);
    arm_mat_init_f32(&matrix->tempMeasurementMatrix,
                     measurementCount,
                     measurementCount,
                     kalman->tempMeasurementMatrix);
}

/**
 * @brief 量测更新失败时使用预测值作为本轮后验值。
 */
static void Algorithm_Kalman_UsePrediction(algorithm_kalman_t *kalman)
{
    uint32_t stateDataCount;

    stateDataCount = (uint32_t)kalman->stateCount * kalman->stateCount;
    memcpy(kalman->state,
           kalman->statePredict,
           sizeof(float) * kalman->stateCount);
    memcpy(kalman->covariance,
           kalman->covariancePredict,
           sizeof(float) * stateDataCount);
}

/**
 * @brief 预测状态 x'(k) = F * x(k-1)。
 */
static arm_status Algorithm_Kalman_PredictState(algorithm_kalman_t *kalman)
{
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    return arm_mat_mult_f32(&matrix->stateTransition,
                            &matrix->state,
                            &matrix->statePredict);
}

/**
 * @brief 预测协方差 P'(k) = F * P(k-1) * F^T + Q。
 */
static arm_status Algorithm_Kalman_PredictCovariance(algorithm_kalman_t *kalman)
{
    arm_status matrixStatus;
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    matrixStatus = arm_mat_trans_f32(&matrix->stateTransition,
                                     &matrix->stateTransitionTranspose);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->stateTransition,
                                    &matrix->covariance,
                                    &matrix->tempStateMatrixA);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->tempStateMatrixA,
                                    &matrix->stateTransitionTranspose,
                                    &matrix->tempStateMatrixB);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    return arm_mat_add_f32(&matrix->tempStateMatrixB,
                           &matrix->processNoise,
                           &matrix->covariancePredict);
}

/**
 * @brief 计算卡尔曼增益 K = P' * H^T * inv(H * P' * H^T + R)。
 */
static arm_status Algorithm_Kalman_CalculateGain(algorithm_kalman_t *kalman)
{
    arm_status matrixStatus;
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    matrixStatus = arm_mat_trans_f32(&matrix->measurementMatrix,
                                     &matrix->measurementTranspose);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->measurementMatrix,
                                    &matrix->covariancePredict,
                                    &matrix->tempMeasurementStateMatrix);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->tempMeasurementStateMatrix,
                                    &matrix->measurementTranspose,
                                    &matrix->tempMeasurementMatrix);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_add_f32(&matrix->tempMeasurementMatrix,
                                   &matrix->measurementNoise,
                                   &matrix->innovationCovariance);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    /*
     * S 不能求逆时，说明当前噪声或协方差配置不适合本轮量测更新，
     * 上层会退化为纯预测，避免输出异常跳变。
     */
    matrixStatus = arm_mat_inverse_f32(&matrix->innovationCovariance,
                                       &matrix->innovationCovarianceInverse);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->covariancePredict,
                                    &matrix->measurementTranspose,
                                    &matrix->tempStateMeasurementMatrix);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    return arm_mat_mult_f32(&matrix->tempStateMeasurementMatrix,
                            &matrix->innovationCovarianceInverse,
                            &matrix->kalmanGain);
}

/**
 * @brief 融合状态 x(k) = x'(k) + K * (z - H * x'(k))。
 */
static arm_status Algorithm_Kalman_CorrectState(algorithm_kalman_t *kalman)
{
    arm_status matrixStatus;
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    matrixStatus = arm_mat_mult_f32(&matrix->measurementMatrix,
                                    &matrix->statePredict,
                                    &matrix->predictedMeasurement);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_sub_f32(&matrix->measurement,
                                   &matrix->predictedMeasurement,
                                   &matrix->innovation);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->kalmanGain,
                                    &matrix->innovation,
                                    &matrix->tempStateVector);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    return arm_mat_add_f32(&matrix->statePredict,
                           &matrix->tempStateVector,
                           &matrix->state);
}

/**
 * @brief 修正协方差 P(k) = P'(k) - K * H * P'(k)。
 */
static arm_status Algorithm_Kalman_CorrectCovariance(algorithm_kalman_t *kalman)
{
    arm_status matrixStatus;
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    matrixStatus = arm_mat_mult_f32(&matrix->kalmanGain,
                                    &matrix->measurementMatrix,
                                    &matrix->tempStateMatrixA);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    matrixStatus = arm_mat_mult_f32(&matrix->tempStateMatrixA,
                                    &matrix->covariancePredict,
                                    &matrix->tempStateMatrixB);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return matrixStatus;
    }

    return arm_mat_sub_f32(&matrix->covariancePredict,
                           &matrix->tempStateMatrixB,
                           &matrix->covariance);
}

void Algorithm_Kalman_Init(algorithm_kalman_t *kalman,
                           uint8_t stateCount,
                           uint8_t measurementCount)
{
    memset(kalman, 0, sizeof(*kalman));

    /*
     * 维度越界属于真实内存边界问题，直接保持全零状态。
     * 调用方可通过调试器看到 stateCount/measurementCount 为 0。
     */
    if ((stateCount == 0U) ||
        (measurementCount == 0U) ||
        (stateCount > ALGORITHM_KALMAN_MAX_STATE_COUNT) ||
        (measurementCount > ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT))
    {
        return;
    }

    kalman->stateCount = stateCount;
    kalman->measurementCount = measurementCount;
    Algorithm_Kalman_InitMatrices(kalman);

    Algorithm_Kalman_SetIdentity(kalman->stateTransition, stateCount);
    Algorithm_Kalman_SetDefaultMeasurementMatrix(kalman);
    Algorithm_Kalman_SetIdentity(kalman->processNoise, stateCount);
    Algorithm_Kalman_SetIdentity(kalman->covariance, stateCount);
    Algorithm_Kalman_SetIdentity(kalman->measurementNoise, measurementCount);
}

void Algorithm_Kalman_Update(algorithm_kalman_t *kalman,
                             const float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT])
{
    arm_status matrixStatus;

    if ((kalman->stateCount == 0U) || (kalman->measurementCount == 0U))
    {
        return;
    }

    memcpy(kalman->measurement,
           measurement,
           sizeof(float) * kalman->measurementCount);

    /*
     * 1. 先验状态预测。
     * 这里失败通常表示矩阵维度被破坏，直接保持上一帧后验状态。
     */
    matrixStatus = Algorithm_Kalman_PredictState(kalman);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return;
    }

    /*
     * 2. 先验协方差预测。
     * 协方差预测失败时没有可靠 P'，因此不进入量测更新。
     */
    matrixStatus = Algorithm_Kalman_PredictCovariance(kalman);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        return;
    }

    /*
     * 3. 计算卡尔曼增益。
     * S 不能求逆时，本轮只使用预测结果。
     */
    matrixStatus = Algorithm_Kalman_CalculateGain(kalman);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        Algorithm_Kalman_UsePrediction(kalman);
        return;
    }

    /*
     * 4. 用测量值修正状态。
     */
    matrixStatus = Algorithm_Kalman_CorrectState(kalman);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        Algorithm_Kalman_UsePrediction(kalman);
        return;
    }

    /*
     * 5. 修正协方差。
     * 公式保持和 SPR 黄金五式一致，便于后续对照调参。
     */
    matrixStatus = Algorithm_Kalman_CorrectCovariance(kalman);
    if (matrixStatus != ARM_MATH_SUCCESS)
    {
        Algorithm_Kalman_UsePrediction(kalman);
    }
}
