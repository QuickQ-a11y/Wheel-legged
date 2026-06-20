#ifndef TASK_IMU_H
#define TASK_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"
#include "device_bmi088.h"

typedef struct
{
    uint8_t isInitialized;                           /* BMI088 初始化完成后置 1。 */
    uint8_t isAttitudeReady;                         /* 完成陀螺零偏采样后置 1。 */
    uint32_t initErrorCount;                         /* 初始化失败累计次数。 */
    uint32_t readErrorCount;                         /* 读取失败累计次数。 */
    uint32_t lastErrorCode;                          /* BMI088 最近一次错误码。 */
    bmi088_data_t bmi088Data;                        /* 最近一次 BMI088 数据快照。 */
    float gyroBiasRadps[BMI088_AXIS_COUNT];          /* 启动阶段估计的陀螺零偏。 */
    float rollRad;                                   /* 横滚角，单位 rad。 */
    float pitchRad;                                  /* 俯仰角，单位 rad。 */
    float yawRad;                                    /* 偏航角，单位 rad。 */
    float dtSec;                                     /* 最近一次姿态积分周期，单位 s。 */
} task_imu_state_t;

void IMU_Task_Init(void);
app_status_t IMU_Task_GetState(task_imu_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
