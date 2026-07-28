#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"

/**
 * @brief 初始化底盘控制状态并创建底盘任务。
 */
void chassis_task_init(void);

/**
 * @brief 设置底盘使能，未使能时持续输出零力矩和零电流。
 */
void chassis_set_enable(uint8_t enable);

/**
 * @brief 设置底盘外层模式，由任务主循环转换为控制状态。
 */
void chassis_set_mode(chassis_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
