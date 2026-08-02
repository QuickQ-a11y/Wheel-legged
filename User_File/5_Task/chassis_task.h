#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_remote.h"

/**
 * @brief 初始化底盘控制状态并创建底盘任务。
 */
void Chassis_Task_Init(void);

#ifdef __cplusplus
}
#endif

#endif
