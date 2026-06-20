#ifndef TASK_CHASSIS_H
#define TASK_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"

#include <stdint.h>

void Chassis_Task_Init(void);
void Chassis_Task_SetEnable(uint8_t enable);
uint8_t Chassis_Task_IsEnabled(void);
uint32_t Chassis_Task_GetFaultFlags(void);
void Chassis_Task_NotifyImuReady(void);

#ifdef __cplusplus
}
#endif

#endif
