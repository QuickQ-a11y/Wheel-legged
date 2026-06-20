#ifndef DRIVER_FDCAN_H
#define DRIVER_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "app_status.h"
#include "fdcan.h"

#include <stdint.h>

typedef struct
{
    FDCAN_RxHeaderTypeDef header;                       /* HAL 原始接收头。 */
    uint8_t data[APP_CONFIG_CAN_MAX_DATA_LENGTH];       /* Classic CAN 最大 8 字节数据。 */
} driver_fdcan_rx_frame_t;

typedef void (*driver_fdcan_rx_callback_t)(FDCAN_HandleTypeDef *handle,
                                        const driver_fdcan_rx_frame_t *frame);

typedef struct
{
    FDCAN_HandleTypeDef *handle;                        /* 当前管理对象绑定的 HAL 句柄。 */
    driver_fdcan_rx_callback_t rxCallback;                 /* 上层接收回调，允许为空。 */
    driver_fdcan_rx_frame_t rxFrame;                       /* 中断接收时复用的帧缓存。 */
} driver_fdcan_object_t;

extern driver_fdcan_object_t driverFdcan1Object;
extern driver_fdcan_object_t driverFdcan2Object;

/**
 * @brief 初始化 FDCAN 外设并启用标准帧接收。
 *
 * 驱动层只配置硬件收发和接收回调，不解析电机协议。
 */
app_status_t Driver_FDCAN_Init(FDCAN_HandleTypeDef *handle,
                            driver_fdcan_rx_callback_t rxCallback);

/**
 * @brief 更新指定 FDCAN 外设的接收回调。
 */
app_status_t Driver_FDCAN_RegisterRxCallback(FDCAN_HandleTypeDef *handle,
                                          driver_fdcan_rx_callback_t rxCallback);

/**
 * @brief 发送一帧标准 ID Classic CAN 数据帧。
 *
 * identifier 范围为 0x000..0x7FF，length 最大为 8 字节。
 */
app_status_t Driver_FDCAN_SendData(FDCAN_HandleTypeDef *handle,
                                uint32_t identifier,
                                const uint8_t *data,
                                uint8_t length);

/**
 * @brief 将 HAL DLC 字段转换为实际数据长度。
 */
uint8_t Driver_FDCAN_DlcToLength(uint32_t dataLength);

#ifdef __cplusplus
}
#endif

#endif
