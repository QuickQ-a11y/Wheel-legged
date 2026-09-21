#ifndef TASK_IMU_H
#define TASK_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_bmi088.h"

typedef struct
{
    uint8_t isInitialized;                           /* BMI088 初始化完成后置 1。 */
    uint8_t isAttitudeReady;                         /* 完成陀螺零偏采样后置 1。 */
    uint32_t initErrorCount;                         /* 初始化失败累计次数。 */
    uint32_t lastErrorCode;                          /* BMI088 最近一次错误码。 */
    bmi088_data_t bmi088Data;                        /* 最近一次 BMI088 原始坐标数据快照。 */
    float gyroBiasRadps[BMI088_AXIS_COUNT];          /* 整车右手系下的启动阶段陀螺零偏。 */
    float gyroBiasEkfRadps[BMI088_AXIS_COUNT];       /* 整车右手系下 EKF 估计的残余陀螺零偏，Z 轴固定为 0。 */
    float filteredGyroRadps[BMI088_AXIS_COUNT];      /* 整车右手系下修正后的角速度，单位 rad/s。 */
    float quaternion[4];                             /* 整车右手系姿态四元数 q0,q1,q2,q3。 */
    float rollRad;                                   /* 整车右手系横滚角，单位 rad。 */
    float pitchRad;                                  /* 整车右手系俯仰角，单位 rad。 */
    float yawRad;                                    /* 整车右手系偏航角，单位 rad。 */
    float yawTotalRad;                               /* 整车右手系连续偏航角，单位 rad。 */
    float bodyMotionAccMps2[BMI088_AXIS_COUNT];      /* 整车右手系机体系去重力运动加速度。 */
    float motionAccMps2[BMI088_AXIS_COUNT];          /* 整车右手系去重力运动加速度，单位 m/s^2。 */
    float accNormMps2;                               /* 原始加速度模长，单位 m/s^2，用于判断静止可信度。 */
    float gyroNormRadps;                             /* 修正后角速度模长，单位 rad/s，用于判断静止可信度。 */
    float temperatureTargetCelsius;                  /* IMU 恒温目标，单位摄氏度。 */
    float temperatureErrorCelsius;                   /* 温控误差 target - feedback，单位摄氏度。 */
    float temperaturePidOutput;                      /* 温控 PID 输出，限幅后映射到 PWM 比较值。 */
    uint32_t temperaturePwmCompare;                  /* TIM3_CH4 PWM 比较值，当前周期为 0~9999。 */
    uint8_t isTemperatureStable;                     /* 温度进入目标附近后置 1，用于允许慢速零偏学习。 */
    uint8_t isTemperatureProtected;                  /* 温度超过保护阈值后置 1，同时关闭加热 PWM。 */
    float zGyroResidualRadps;                        /* Z 轴扣除当前零偏后的残余角速度，单位 rad/s。 */
    uint32_t zBiasUpdateCount;                       /* 静止且温度稳定时，Z 轴零偏慢速更新次数。 */
    uint8_t isZBiasUpdated;                          /* 最近一轮是否更新了 Z 轴零偏。 */
    float dtSec;                                     /* 最近一次IMU采样处理的实际周期，单位s。 */
} task_imu_state_t;

extern task_imu_state_t imuTaskDebugState;           /* Watch 窗口长期观察用 IMU 快照，业务读取仍使用 IMU_Task_GetState。 */

void IMU_Task_Init(void);
void IMU_Task_GetState(task_imu_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
