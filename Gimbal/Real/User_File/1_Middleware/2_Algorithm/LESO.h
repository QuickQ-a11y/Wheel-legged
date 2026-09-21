#ifndef LESO_H
#define LESO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ALGORITHM_LESO_MAX_STATE_COUNT 10U
#define ALGORITHM_LESO_MAX_INPUT_COUNT 4U
#define ALGORITHM_LESO_MAX_TOTAL_COUNT (ALGORITHM_LESO_MAX_STATE_COUNT + \
                                        ALGORITHM_LESO_MAX_INPUT_COUNT)

/**
 * @brief 输入通道扩张的线性扩张状态观测器（LESO）。
 *
 * 把每个输入通道的总扰动扩张成一个状态，扩张系统为
 *     A_e = [Ad Bd; 0 I]、B_e = [Bd; 0]、C_e = [I 0]
 *     x_hat(k+1) = A_e*x_hat(k) + B_e*u(k) + L*(y(k) - C_e*x_hat(k))
 * 因为 C_e = [I 0]，残差就是 y 减估计的前 stateCount 项，不需要保存 C_e；
 * 又因为 A_e 的右上块就是 Bd、下半块是 [0 I]，扩张后的扰动与输入完全同通道进入，
 * 展开后只需要 Ad、Bd、L 三个矩阵：
 *     x_hat'[i]              = Ad*x_hat + Bd*(d_hat + u) + L*e     i < stateCount
 *     x_hat'[stateCount + k] = x_hat[stateCount + k] + L*e
 * 所以扰动估计与输入同单位（本工程为 N*m），可以直接从控制量里减去。
 */
typedef struct
{
    uint8_t stateCount; /* 原系统状态维度。 */
    uint8_t inputCount; /* 输入维度，同时是扩张的扰动维度。 */
    /* 估计向量：前 stateCount 项是原状态，后 inputCount 项是各通道总扰动。 */
    float estimate[ALGORITHM_LESO_MAX_TOTAL_COUNT];
    /* 残差 y - C_e*x_hat，收敛后应接近0，是判断观测器是否可信的第一入口。 */
    float innovation[ALGORITHM_LESO_MAX_STATE_COUNT];
} algorithm_leso_t;

/**
 * @brief 清空估计并记录维度。
 */
void Algorithm_LESO_Init(algorithm_leso_t *leso,
                         uint8_t stateCount,
                         uint8_t inputCount);

/**
 * @brief 用当前测量重建原状态估计并清零扰动估计。
 *
 * 首次启动和输出被封锁后重新放行时使用，避免从陈旧估计开始产生大残差冲击。
 */
void Algorithm_LESO_Seed(algorithm_leso_t *leso, const float *measurement);

/**
 * @brief 推进一个控制周期的扩张状态观测。
 *
 * Ad 为 stateCount*stateCount、Bd 为 stateCount*inputCount、
 * L 为 (stateCount+inputCount)*stateCount，一律行优先，与
 * Algorithm_LQR_FitLqrKPoly22 的输出布局一致。
 * disturbanceLimit 逐通道限制扰动估计幅值，元素为0表示该通道不限幅；
 * 传 NULL 表示全部不限幅。限幅在递推内部生效，防止扰动状态持续累积。
 */
void Algorithm_LESO_Update(algorithm_leso_t *leso,
                           const float *Ad,
                           const float *Bd,
                           const float *L,
                           const float *measurement,
                           const float *input,
                           const float *disturbanceLimit);

#ifdef __cplusplus
}
#endif

#endif
