#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"
#include "remote_input.h"

/** @brief 初始化遥控模式边沿和航向摇杆内部状态。 */
void Chassis_Remote_Init(void);

/**
 * @brief 将通用遥控输入转换为底盘运动目标和外层模式。
 *
 * online为零时只关闭输出许可并保持控制计算，不使用input中的陈旧命令。
 */
void Chassis_Remote_Update(const remote_input_t *input, uint8_t online_flag);

#ifdef __cplusplus
}
#endif

#endif
