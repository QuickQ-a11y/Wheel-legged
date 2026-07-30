#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"

/**
 * @brief 初始化底盘控制状态并创建底盘任务。
 */
void Chassis_Task_Init(void);

/**
 * @brief 设置底盘非零输出许可，关闭时仍允许执行控制计算。
 */
void Chassis_SetOutputEnable(uint8_t enable);

/**
 * @brief 设置底盘外层模式，由任务主循环转换为控制状态。
 */
void Chassis_SetMode(chassis_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
