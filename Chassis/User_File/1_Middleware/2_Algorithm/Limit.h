#ifndef LIMIT_H
#define LIMIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/**
 * @brief 把标量限制到 [minValue, maxValue] 区间。
 *
 * 用于反三角函数定义域、定点编码量程和拟合采样范围等有明确上下界的场合。
 */
static inline float Algorithm_LimitRange(float value,
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
 * @brief 按给定绝对值对称限幅。
 *
 * limit 为零或负表示该通道不允许输出，直接返回零。
 */
static inline float Algorithm_LimitSymmetric(float value, float limit)
{
    float positiveLimit = fabsf(limit);

    if (positiveLimit <= 0.0f)
    {
        return 0.0f;
    }
    if (value > positiveLimit)
    {
        return positiveLimit;
    }
    if (value < -positiveLimit)
    {
        return -positiveLimit;
    }

    return value;
}

#ifdef __cplusplus
}
#endif

#endif
