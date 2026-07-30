#ifndef ALGORITHM_ANGLE_H
#define ALGORITHM_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 将弧度角归一化到 [-pi, pi)。
 */
float Algorithm_AngleNormalizeRad(float angle_rad);

/**
 * @brief 返回与输入角等价且最接近参考角的值。
 */
float Algorithm_AngleNearestEquivalentRad(float angle_rad,
                                          float reference_rad);

/**
 * @brief 根据相邻单圈角更新连续角。
 */
float Algorithm_AngleUnwrapRad(float previous_wrapped_rad,
                               float previous_total_rad,
                               float current_wrapped_rad);

#ifdef __cplusplus
}
#endif

#endif
