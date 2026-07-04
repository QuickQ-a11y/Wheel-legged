#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0, /* 零力模式，强制安全输出。 */
    CHASSIS_MODE_FOLLOW,        /* 跟随模式，当前作为站立控制主入口。 */
    CHASSIS_MODE_TOP,           /* 小陀螺模式，当前暂用站立控制链路。 */
    CHASSIS_MODE_SELF_SAVE,     /* 自救模式，当前未接入控制逻辑。 */
    CHASSIS_MODE_BENCH,         /* 小板凳模式，用于后续腿和轮调试。 */
} chassis_mode_t;

void ChassisTask_Init(void);
void ChassisTask_SetEnable(uint8_t enable);
void ChassisTask_SetMode(chassis_mode_t mode);
void ChassisTask_NotifyImuReady(void);

#ifdef __cplusplus
}
#endif

#endif
