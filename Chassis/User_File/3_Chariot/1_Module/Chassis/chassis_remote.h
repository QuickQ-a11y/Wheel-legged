#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"
#include "remote_input.h"

/*
 * 拨杆到模式的映射表。重排模式只改下面这几行，不要动 device_ibus.c / device_dr16.c。
 * 可填任意 CHASSIS_MODE_*（BENCH 和 SELF_SAVE 的控制代码都在，只是当前没绑定档位）。
 *
 * SwC 是使能级，不在表里：上=卸力+急停，中=卸力待命，下=按本表选模式。
 * SwC 在下时，VrA 选组、SwA/SwB 在组内选模式。
 *
 * 两组的冲突处理【刻意不对称】，这不是笔误：
 *   低组里小陀螺和台阶是两个互斥动作，两个都拨下去无从判断要哪个，回落 FOLLOW；
 *   高组里跳跃本身就包含压腿蓄力，所以 SwB 直接盖过 SwA，不存在冲突。
 */
#define CHASSIS_MAP_LO_A_UP_B_UP CHASSIS_MODE_FOLLOW
#define CHASSIS_MAP_LO_A_DN_B_UP CHASSIS_MODE_TOP
#define CHASSIS_MAP_LO_A_UP_B_DN CHASSIS_MODE_STEP
#define CHASSIS_MAP_LO_A_DN_B_DN CHASSIS_MODE_FOLLOW

#define CHASSIS_MAP_HI_B_DN CHASSIS_MODE_JUMP
#define CHASSIS_MAP_HI_A_DN CHASSIS_MODE_FOLLOW /* 压缩腿只改腿长，模式仍是 FOLLOW。 */
#define CHASSIS_MAP_HI_A_UP CHASSIS_MODE_FOLLOW

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
