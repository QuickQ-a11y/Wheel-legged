#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"
#include "remote_input.h"

/**
 * @brief 将通用遥控输入转换为底盘运动目标和外层模式。
 *
 * Remote.online为零时只关闭输出许可并保持控制计算，不使用陈旧命令。
 */
void Chassis_Remote_Update(const Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
