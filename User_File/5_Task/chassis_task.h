#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"
#include "remote_input.h"

/**
 * @brief 初始化底盘控制状态并创建底盘任务。
 */
void Chassis_Task_Init(void);

/**
 * @brief 设置底盘非零输出许可，关闭时仍允许执行控制计算。
 *
 * DM协议使能由底盘任务启动流程维护，本接口只控制最终非零命令。
 */
void Chassis_SetOutputEnable(uint8_t enable);

/**
 * @brief 设置底盘外层模式，由任务主循环转换为控制状态。
 */
void Chassis_SetMode(chassis_mode_t mode);

/**
 * @brief 将一份通用遥控输入转换为底盘运动目标和外层模式。
 *
 * online为零时只关闭输出许可并保持控制计算，不使用input中的陈旧命令。
 */
void Chassis_SetRemoteInput(const remote_input_t *input, uint8_t online);

#ifdef __cplusplus
}
#endif

#endif
