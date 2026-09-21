#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT 4U
#define ALGORITHM_QUATERNION_EKF_AXIS_COUNT 3U
#define ALGORITHM_QUATERNION_EKF_STATE_COUNT 6U
#define ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT 3U

typedef struct
{
    float quaternionProcessNoise;       /* 四元数过程噪声，单位随离散化周期缩放。 */
    float gyroBiasProcessNoise;         /* X/Y 轴陀螺零偏过程噪声，单位随离散化周期缩放。 */
    float accelMeasurementNoise;        /* 加速度方向观测噪声。 */
    float quaternionInitialCovariance;  /* 四元数初始协方差。 */
    float gyroBiasInitialCovariance;    /* X/Y 轴陀螺零偏初始协方差。 */
    float accelLpfTimeSec;              /* 加速度进入 EKF 前的一阶低通时间常数，单位 s。 */
    float accelNormMinMps2;             /* 允许使用加速度修正姿态的最小模长，单位 m/s^2。 */
    float accelNormMaxMps2;             /* 允许使用加速度修正姿态的最大模长，单位 m/s^2。 */
    float gyroStableThresholdRadps;     /* 允许修正陀螺零偏的角速度模长阈值，单位 rad/s。 */
    float gyroBiasCorrectionLimitRadps; /* 单次更新允许修正的最大零偏变化率，单位 rad/s。 */
} algorithm_quaternion_ekf_config_t;

typedef struct
{
    uint8_t isInitialized; /* 已用第一帧有效加速度完成姿态初值后置 1。 */
    uint32_t updateCount;  /* EKF 更新次数，便于 Watch 判断任务是否持续运行。 */

    float quaternion[ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT]; /* 姿态四元数 q0,q1,q2,q3。 */
    float gyroBiasRadps[ALGORITHM_QUATERNION_EKF_AXIS_COUNT];    /* EKF 估计的陀螺零偏，Z 轴固定为 0。 */
    float filteredGyroRadps[ALGORITHM_QUATERNION_EKF_AXIS_COUNT]; /* 扣除 EKF 零偏后的角速度，单位 rad/s。 */
    float filteredAccMps2[ALGORITHM_QUATERNION_EKF_AXIS_COUNT];  /* 进入 EKF 的低通加速度，单位 m/s^2。 */
    float rollRad;                                               /* 绕机体 X 轴横滚角，单位 rad。 */
    float pitchRad;                                              /* 绕机体 Y 轴俯仰角，单位 rad。 */
    float yawRad;                                                /* 绕机体 Z 轴偏航角，单位 rad。 */
    float yawTotalRad;                                           /* 连续偏航角，跨越正负 pi 时累计圈数。 */

    algorithm_quaternion_ekf_config_t config; /* 当前滤波参数快照。 */

    float covariance[ALGORITHM_QUATERNION_EKF_STATE_COUNT *
                     ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 后验协方差 P。 */
    float covariancePredict[ALGORITHM_QUATERNION_EKF_STATE_COUNT *
                            ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 先验协方差 P'。 */
    float stateTransition[ALGORITHM_QUATERNION_EKF_STATE_COUNT *
                          ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 状态转移矩阵 F。 */
    float measurementMatrix[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT *
                            ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 加速度观测雅可比 H。 */
    float kalmanGain[ALGORITHM_QUATERNION_EKF_STATE_COUNT *
                     ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* 卡尔曼增益 K。 */
    float innovation[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* 加速度方向残差 z-h(x)。 */
    float measurement[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* 单位化加速度观测 z。 */
    float predictedMeasurement[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* 预测重力方向 h(x)。 */
    float tempStateMatrix[ALGORITHM_QUATERNION_EKF_STATE_COUNT *
                          ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 6x6 矩阵运算缓存。 */
    float tempMeasurementStateMatrix[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT *
                                     ALGORITHM_QUATERNION_EKF_STATE_COUNT]; /* 3x6 矩阵运算缓存。 */
    float innovationCovariance[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT *
                               ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* 残差协方差 S。 */
    float innovationCovarianceInverse[ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT *
                                      ALGORITHM_QUATERNION_EKF_MEASUREMENT_COUNT]; /* S 的逆矩阵。 */

    float yawLastRad;    /* 上一帧 [-pi, pi] 偏航角，用于连续 yaw 计圈。 */
    int32_t yawRoundCount; /* 连续 yaw 的整圈计数。 */
} algorithm_quaternion_ekf_t;

/**
 * @brief 初始化四元数 EKF 的参数、协方差和内部缓存。
 *
 * 调用方应在上电零偏采样完成前完成初始化，第一帧有效加速度会自动给出
 * roll/pitch 初值，yaw 初值固定为 0。
 */
void Algorithm_QuaternionEKF_Init(algorithm_quaternion_ekf_t *filter,
                                  const algorithm_quaternion_ekf_config_t *config);

/**
 * @brief 用陀螺和加速度更新姿态四元数。
 *
 * gyroRadps 输入应先扣除启动阶段静态零偏；本 EKF 只继续估计 X/Y 轴残余零偏，
 * Z 轴不具备加速度观测修正条件，因此 yaw 长期仍会按陀螺积分漂移。
 */
void Algorithm_QuaternionEKF_Update(algorithm_quaternion_ekf_t *filter,
                                    const float gyroRadps[ALGORITHM_QUATERNION_EKF_AXIS_COUNT],
                                    const float accMps2[ALGORITHM_QUATERNION_EKF_AXIS_COUNT],
                                    float dtSec);

/**
 * @brief 将机体系向量旋转到自然坐标系。
 */
void Algorithm_QuaternionEKF_BodyToEarth(
    const float bodyVector[ALGORITHM_QUATERNION_EKF_AXIS_COUNT],
    const float quaternion[ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT],
    float earthVector[ALGORITHM_QUATERNION_EKF_AXIS_COUNT]);

/**
 * @brief 将自然坐标系向量旋转到机体系。
 */
void Algorithm_QuaternionEKF_EarthToBody(
    const float earthVector[ALGORITHM_QUATERNION_EKF_AXIS_COUNT],
    const float quaternion[ALGORITHM_QUATERNION_EKF_QUATERNION_COUNT],
    float bodyVector[ALGORITHM_QUATERNION_EKF_AXIS_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
