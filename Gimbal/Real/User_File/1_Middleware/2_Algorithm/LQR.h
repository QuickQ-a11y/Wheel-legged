#ifndef LQR_H
#define LQR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT 6U

typedef enum
{
    ALGORITHM_LQR_POLY22_P00 = 0,    /* 常数项。 */
    ALGORITHM_LQR_POLY22_P10,        /* inputX 一次项。 */
    ALGORITHM_LQR_POLY22_P01,        /* inputY 一次项。 */
    ALGORITHM_LQR_POLY22_P20,        /* inputX 二次项。 */
    ALGORITHM_LQR_POLY22_P11,        /* inputX * inputY 交叉项。 */
    ALGORITHM_LQR_POLY22_P02,        /* inputY 二次项。 */
} algorithm_lqr_poly22_coefficient_index_t;

/**
 * @brief 使用二元二次多项式拟合系数计算 LQR 增益矩阵。
 *
 * 系数顺序必须与 MATLAB `fit(..., "poly22")` 的 `coeffvalues()` 一致：
 * p00、p10、p01、p20、p11、p02。输入会先限制到给定范围，再逐项计算矩阵元素。
 */
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
                                 uint8_t *isInputLimited);

#ifdef __cplusplus
}
#endif

#endif
