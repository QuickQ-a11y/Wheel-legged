#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#include <stdint.h>

#define ALGORITHM_KALMAN_MAX_STATE_COUNT 4U
#define ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT 4U

/**
 * @brief CMSIS-DSP 矩阵描述符集合。
 *
 * 描述符只保存矩阵行列和数据指针，真实数据仍存放在 algorithm_kalman_t 的数组中。
 * 该结构体由 Algorithm_Kalman_Init 统一绑定，调用方不需要直接操作。
 */
typedef struct
{
    arm_matrix_instance_f32 state;                         /* 后验状态 x(k|k)。 */
    arm_matrix_instance_f32 statePredict;                  /* 先验状态 x(k|k-1)。 */
    arm_matrix_instance_f32 measurement;                   /* 测量向量 z。 */
    arm_matrix_instance_f32 stateTransition;               /* 状态转移矩阵 F。 */
    arm_matrix_instance_f32 stateTransitionTranspose;      /* 状态转移转置矩阵 F^T。 */
    arm_matrix_instance_f32 measurementMatrix;             /* 测量矩阵 H。 */
    arm_matrix_instance_f32 measurementTranspose;          /* 测量转置矩阵 H^T。 */
    arm_matrix_instance_f32 processNoise;                  /* 过程噪声矩阵 Q。 */
    arm_matrix_instance_f32 measurementNoise;              /* 测量噪声矩阵 R。 */
    arm_matrix_instance_f32 covariance;                    /* 后验协方差 P(k|k)。 */
    arm_matrix_instance_f32 covariancePredict;             /* 先验协方差 P(k|k-1)。 */
    arm_matrix_instance_f32 kalmanGain;                    /* 卡尔曼增益 K。 */
    arm_matrix_instance_f32 innovation;                    /* 残差 z - Hx。 */
    arm_matrix_instance_f32 predictedMeasurement;          /* 预测测量 Hx。 */
    arm_matrix_instance_f32 innovationCovariance;          /* 残差协方差 S。 */
    arm_matrix_instance_f32 innovationCovarianceInverse;   /* 残差协方差逆矩阵 S^-1。 */
    arm_matrix_instance_f32 tempStateVector;               /* 状态维度临时向量。 */
    arm_matrix_instance_f32 tempStateMatrixA;              /* 状态维度临时矩阵 A。 */
    arm_matrix_instance_f32 tempStateMatrixB;              /* 状态维度临时矩阵 B。 */
    arm_matrix_instance_f32 tempStateMeasurementMatrix;    /* 状态-测量维度临时矩阵。 */
    arm_matrix_instance_f32 tempMeasurementStateMatrix;    /* 测量-状态维度临时矩阵。 */
    arm_matrix_instance_f32 tempMeasurementMatrix;         /* 测量维度临时矩阵。 */
} algorithm_kalman_matrix_t;

/**
 * @brief 固定最大维度的线性卡尔曼滤波器。
 *
 * 算法层只保存矩阵和中间缓存，不申请动态内存；调用方负责填写 F、H、Q、R、P
 * 和初始状态，并保证状态量、测量量的物理单位一致。
 */
typedef struct
{
    uint8_t stateCount;       /* 状态维度。 */
    uint8_t measurementCount; /* 测量维度。 */

    float state[ALGORITHM_KALMAN_MAX_STATE_COUNT];          /* 后验状态 x(k|k)。 */
    float statePredict[ALGORITHM_KALMAN_MAX_STATE_COUNT];   /* 先验状态 x(k|k-1)。 */
    float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT]; /* 当前测量向量 z。 */

    float stateTransition[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                          ALGORITHM_KALMAN_MAX_STATE_COUNT]; /* 状态转移矩阵 F。 */
    float measurementMatrix[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                            ALGORITHM_KALMAN_MAX_STATE_COUNT]; /* 测量矩阵 H。 */
    float processNoise[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                       ALGORITHM_KALMAN_MAX_STATE_COUNT]; /* 过程噪声矩阵 Q。 */
    float measurementNoise[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                           ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT]; /* 测量噪声矩阵 R。 */
    float covariance[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                     ALGORITHM_KALMAN_MAX_STATE_COUNT]; /* 后验协方差 P(k|k)。 */

    float covariancePredict[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                            ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float stateTransitionTranspose[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                                   ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float measurementTranspose[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                               ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float kalmanGain[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                     ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float innovation[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float predictedMeasurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float innovationCovariance[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                               ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float innovationCovarianceInverse[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                                      ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float tempStateVector[ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float tempStateMatrixA[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                           ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float tempStateMatrixB[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                           ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float tempStateMeasurementMatrix[ALGORITHM_KALMAN_MAX_STATE_COUNT *
                                     ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    float tempMeasurementStateMatrix[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                                     ALGORITHM_KALMAN_MAX_STATE_COUNT];
    float tempMeasurementMatrix[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT *
                                ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT];
    algorithm_kalman_matrix_t matrix; /* CMSIS-DSP 矩阵描述符，初始化后指向本结构体内数组。 */
} algorithm_kalman_t;

/**
 * @brief 初始化卡尔曼滤波器存储和默认矩阵。
 *
 * 默认 F、H、Q、R、P 都按单位矩阵初始化；实际工程通常会在初始化后覆盖这些矩阵。
 */
void Algorithm_Kalman_Init(algorithm_kalman_t *kalman,
                           uint8_t stateCount,
                           uint8_t measurementCount);

/**
 * @brief 按当前 F、H、Q、R、P 和测量向量执行一次线性卡尔曼更新。
 *
 * 测量向量长度必须等于初始化时的 measurementCount；输出写回 kalman->state。
 */
void Algorithm_Kalman_Update(algorithm_kalman_t *kalman,
                             const float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
