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
 * 底盘据此判断丢帧。调用方负责分频到 200 Hz。
 *
 * yaw_rel 是云台 YAW 相对底盘的关节角，单位 rad，需已归一化到 ±pi。
 */
void Board_UpdateTxFrames(const Remote_t *remote, float yaw_rel);

#ifdef __cplusplus
}
#endif

#endif
