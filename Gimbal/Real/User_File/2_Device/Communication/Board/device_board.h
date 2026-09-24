#ifndef DEVICE_BOARD_H
#define DEVICE_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remote_input.h"

#include <stdint.h>

/**
 * @brief 把遥控快照和云台朝向打包成两帧板间报文，写入 CAN 发送缓存。
 *
 * 帧格式见 app_config.h 的板间通信注释块。序号由本模块内部维护，
 * 底盘据此判断丢帧。调用方负责按 APP_BOARD_SEND_DIV 分频。
 *
 * yaw_rel 是云台 YAW 相对底盘的关节角，单位 rad，需已归一化到 ±pi。
 * autoaim_flag 是自瞄【实际激活】的结论（拨杆在位且上位机在线），
 * 不是 SwD 的原始位置——底盘靠它决定要不要锁航向。VrB 不下发。
 */
void Board_UpdateTxFrames(const Remote_t *remote,
                          float yaw_rel,
                          uint8_t autoaim_flag);

#ifdef __cplusplus
}
#endif

#endif
