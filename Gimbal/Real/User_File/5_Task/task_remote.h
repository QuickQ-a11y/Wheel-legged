#ifndef TASK_REMOTE_H
#define TASK_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "remote_input.h"

/*
 * 遥控后端由 app_config.h 的 APP_REMOTE_BACKEND 选定。两种接收机挂同一路 UART5，
 * 但帧长、波特率和校验都不同，所以后端头文件、帧长常量和调试快照的形状都要跟着切。
 */
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_IA10B
#include "device_ia10b.h"
#define REMOTE_FRAME_LEN IA10B_FRAME_LEN
#else
#include "device_dr16.h"
#define REMOTE_FRAME_LEN DR16_FRAME_LEN
#endif

#include <stdint.h>

typedef struct
{
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_IA10B
    ia10b_data_t backendData;          /* 当前 i-BUS 原始通道值，单位 us。 */
#else
    dr16_data_t backendData;           /* 当前 DBUS 原始解析结果。 */
#endif
    uint8_t rawFrame[REMOTE_FRAME_LEN]; /* 最近一次原始帧。 */
    uint8_t online;
    uint8_t syncFrameCount;
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
    /* 键鼠是 DR16 独有的，换后端后这两个计数没有来源，不留空字段。 */
    uint16_t keyPressed;
    uint16_t keyReleased;
#endif
    uint16_t lastRxSize;
    uint32_t lastValidTick;
    uint32_t rxEventCount;
    uint32_t validFrameCount;
    uint32_t invalidSizeCount;
    uint32_t invalidFrameCount;
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
    /* DBUS 的滚轮字段可能标记为无效；i-BUS 的旋钮一直在发，没有这个状态。 */
    uint32_t invalidDialCount;
#endif
    uint32_t overwriteCount;
    uint32_t uartErrorCount;
    uint32_t restartErrorCount;
    uint32_t lastUartError;
    uint32_t validFrameRateHz;
} task_remote_state_t;

extern task_remote_state_t remoteTaskDebugState;

/**
 * @brief 初始化遥控接收状态并创建遥控器任务。
 */
void Remote_Task_Init(void);

/**
 * @brief 将任务发布的完整遥控快照刷新到全局Remote。
 */
void Remote_Task_Update(void);

#ifdef __cplusplus
}
#endif

#endif
