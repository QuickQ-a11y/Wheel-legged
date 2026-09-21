#include "LQR.h"

#include "Limit.h"

#include <stddef.h>

/**
 * @brief 计算单个 poly22 多项式元素。
 *
 * `inputX` 和 `inputY` 通常对应左右腿长，单位由调用方保证。
 */
static float Algorithm_LQR_EvaluatePoly22(const float coefficients[ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT],
                                          float inputX,
                                          float inputY)
{
    return coefficients[ALGORITHM_LQR_POLY22_P00] +
           (coefficients[ALGORITHM_LQR_POLY22_P10] * inputX) +
           (coefficients[ALGORITHM_LQR_POLY22_P01] * inputY) +
           (coefficients[ALGORITHM_LQR_POLY22_P20] * inputX * inputX) +
           (coefficients[ALGORITHM_LQR_POLY22_P11] * inputX * inputY) +
           (coefficients[ALGORITHM_LQR_POLY22_P02] * inputY * inputY);
}

void Algorithm_LQR_FitLqrKPoly22(const float *lqrKFitCoefficients,
                                 uint32_t outputCount,
                                 uint32_t stateCount,
                                 float inputX,
                                 float inputY,
                                 float minInput,
                                 float maxInput,
                                 float *lqrKMatrix,
                                 float *limitedInputX,
                                 float *limitedInputY,
                                 uint8_t *isInputLimited)
{
    uint32_t outputIndex;
    uint32_t stateIndex;
    float limitedX;
    float limitedY;

    /* 腿长必须夹紧到系数实际采样区间，禁止在采样范围外外推。 */
    limitedX = Algorithm_LimitRange(inputX, minInput, maxInput);
    limitedY = Algorithm_LimitRange(inputY, minInput, maxInput);

    *limitedInputX = limitedX;
    *limitedInputY = limitedY;
    *isInputLimited = ((limitedX != inputX) || (limitedY != inputY)) ? 1U : 0U;

    for (outputIndex = 0U; outputIndex < outputCount; outputIndex++)
    {
        for (stateIndex = 0U; stateIndex < stateCount; stateIndex++)
        {
            uint32_t matrixIndex = (outputIndex * stateCount) + stateIndex;
            const float *coefficients =
                &lqrKFitCoefficients[matrixIndex * ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];

            /* 矩阵和系数均按行优先存储：先输出行，再状态列。 */
            lqrKMatrix[matrixIndex] = Algorithm_LQR_EvaluatePoly22(coefficients,
                                                                   limitedX,
                                                                   limitedY);
        }
    }
}
