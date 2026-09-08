#ifndef DEVICE_BOARD_H
#define DEVICE_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "remote_input.h"

#include <stdint.h>

/** @brief 板间链路状态，同时是 Watch 观察入口。 */
typedef struct
{
    Remote_t remote;              /* 从板间帧解出的归一化遥控快照。 */
    float yaw_rel;                /* 云台 YAW 相对底盘的关节角，单位 rad。 */
    uint8_t sequence;             /* 最近一帧状态帧的序号。 */
    uint32_t stickFrameCount;     /* 累计收到的摇杆帧数。 */
    uint32_t stateFrameCount;     /* 累计收到的状态帧数。 */
    uint32_t lastUpdateTick;      /* 最近一次收到板间帧的 HAL tick。 */
    uint8_t received;             /* 收到过第一帧后置 1。 */
} board_state_t;

extern board_state_t boardDebugState;

/**
 * @brief 解析一帧板间报文并刷新链路状态。
 *
 * 帧格式见 app_config.h 的板间通信注释块。不匹配的 ID 直接忽略。
 */
void Board_UpdateFeedback(uint32_t identifier,
                          const uint8_t data[APP_BOARD_FRAME_LEN]);

/**
 * @brief 判断板间链路是否仍在线。
 *
 * 用"收到过帧"标志加上时间戳差判定：上电时 tick 和时间戳都是 0，
 * 只比时间差会误判成在线；相减而不是相加，保证 tick 回绕时仍然正确。
 */
uint8_t Board_IsOnline(uint32_t nowTick);

/** @brief 读取板间链路解出的遥控快照。 */
void Board_GetRemote(Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
