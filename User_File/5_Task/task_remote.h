#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_dr16.h"
#include "remote_input.h"

#include <stdint.h>

typedef struct
{
    remote_input_t input;              /* 发布给底盘的通用输入。 */
    dr16_data_t dr16Data;              /* 当前DR16原始解析结果。 */
    uint8_t rawFrame[DR16_FRAME_LEN];  /* 最近一次DR16原始帧。 */
    uint8_t online;
    uint8_t syncFrameCount;
    uint16_t keyPressed;
    uint16_t keyReleased;
    uint16_t lastRxSize;
    uint32_t lastValidTick;
    uint32_t rxEventCount;
    uint32_t validFrameCount;
    uint32_t invalidSizeCount;
    uint32_t invalidFrameCount;
    uint32_t invalidDialCount;
    uint32_t overwriteCount;
    uint32_t uartErrorCount;
    uint32_t restartErrorCount;
    uint32_t lastUartError;
    uint32_t validFrameRateHz;
} task_remote_state_t;

extern task_remote_state_t remoteTaskDebugState;

/**
 * @brief 初始化 DR16 接收状态并创建遥控器任务。
 */
void Remote_Task_Init(void);

/**
 * @brief 复制最新的通用遥控输入、DR16数据与诊断状态快照。
 */
void Remote_Task_GetState(task_remote_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
