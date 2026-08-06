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

/**
 * @brief 执行一轮标准线性卡尔曼五式，顺序与 SPR 参考实现一致。
 *
 * 1. x' = F * x
 * 2. P' = F * P * F^T + Q
 * 3. K  = P' * H^T * inv(H * P' * H^T + R)
 * 4. x  = x' + K * (z - H * x')
 * 5. P  = P' - K * H * P'
 *
 * S 不可逆时说明当前噪声或协方差配置不适合本轮量测更新，退化为纯预测，
 * 避免输出异常跳变。
 */
void Algorithm_Kalman_Update(algorithm_kalman_t *kalman,
                             const float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT])
{
    algorithm_kalman_matrix_t *matrix = &kalman->matrix;

    if ((kalman->stateCount == 0U) || (kalman->measurementCount == 0U))
    {
        return;
    }

    memcpy(kalman->measurement,
           measurement,
           sizeof(float) * kalman->measurementCount);

    /* 1. 先验状态预测。 */
    (void)arm_mat_mult_f32(&matrix->stateTransition,
                           &matrix->state,
                           &matrix->statePredict);

    /* 2. 先验协方差预测。 */
    (void)arm_mat_trans_f32(&matrix->stateTransition,
                            &matrix->stateTransitionTranspose);
    (void)arm_mat_mult_f32(&matrix->stateTransition,
                           &matrix->covariance,
                           &matrix->tempStateMatrixA);
    (void)arm_mat_mult_f32(&matrix->tempStateMatrixA,
                           &matrix->stateTransitionTranspose,
                           &matrix->tempStateMatrixB);
    (void)arm_mat_add_f32(&matrix->tempStateMatrixB,
                          &matrix->processNoise,
                          &matrix->covariancePredict);

    /* 3. 计算卡尔曼增益。 */
    (void)arm_mat_trans_f32(&matrix->measurementMatrix,
                            &matrix->measurementTranspose);
    (void)arm_mat_mult_f32(&matrix->measurementMatrix,
                           &matrix->covariancePredict,
                           &matrix->tempMeasurementStateMatrix);
    (void)arm_mat_mult_f32(&matrix->tempMeasurementStateMatrix,
                           &matrix->measurementTranspose,
                           &matrix->tempMeasurementMatrix);
    (void)arm_mat_add_f32(&matrix->tempMeasurementMatrix,
                          &matrix->measurementNoise,
                          &matrix->innovationCovariance);
    if (arm_mat_inverse_f32(&matrix->innovationCovariance,
                            &matrix->innovationCovarianceInverse) !=
        ARM_MATH_SUCCESS)
    {
        /* S 不可逆，本轮只使用预测结果。 */
        memcpy(kalman->state,
               kalman->statePredict,
               sizeof(float) * kalman->stateCount);
        memcpy(kalman->covariance,
               kalman->covariancePredict,
               sizeof(float) * (uint32_t)kalman->stateCount * kalman->stateCount);
        return;
    }
    (void)arm_mat_mult_f32(&matrix->covariancePredict,
                           &matrix->measurementTranspose,
                           &matrix->tempStateMeasurementMatrix);
    (void)arm_mat_mult_f32(&matrix->tempStateMeasurementMatrix,
                           &matrix->innovationCovarianceInverse,
                           &matrix->kalmanGain);

    /* 4. 用测量值修正状态。 */
    (void)arm_mat_mult_f32(&matrix->measurementMatrix,
                           &matrix->statePredict,
                           &matrix->predictedMeasurement);
    (void)arm_mat_sub_f32(&matrix->measurement,
                          &matrix->predictedMeasurement,
                          &matrix->innovation);
    (void)arm_mat_mult_f32(&matrix->kalmanGain,
                           &matrix->innovation,
                           &matrix->tempStateVector);
    (void)arm_mat_add_f32(&matrix->statePredict,
                          &matrix->tempStateVector,
                          &matrix->state);

    /* 5. 修正协方差。 */
    (void)arm_mat_mult_f32(&matrix->kalmanGain,
                           &matrix->measurementMatrix,
                           &matrix->tempStateMatrixA);
    (void)arm_mat_mult_f32(&matrix->tempStateMatrixA,
                           &matrix->covariancePredict,
                           &matrix->tempStateMatrixB);
    (void)arm_mat_sub_f32(&matrix->covariancePredict,
                          &matrix->tempStateMatrixB,
                          &matrix->covariance);
}
