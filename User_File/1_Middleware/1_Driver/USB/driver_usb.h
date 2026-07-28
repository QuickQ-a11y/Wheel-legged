#ifndef DRIVER_USB_H
#define DRIVER_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*driver_usb_receive_callback_t)(const uint8_t *data, uint32_t length);
typedef void (*driver_usb_transmit_callback_t)(void);

/**
 * @brief 注册 USB CDC 收发完成回调。
 */
void Driver_USB_Init(driver_usb_receive_callback_t receiveCallback,
                     driver_usb_transmit_callback_t transmitCallback,
                     driver_usb_transmit_callback_t disconnectCallback);

/**
 * @brief 提交一段 CDC 数据，返回 USB Device 库的 OK、BUSY 或 FAIL。
 *
 * 发送缓冲在完成回调到来前必须保持不变，因此本接口只保存调用者的缓冲地址。
 */
uint8_t Driver_USB_Send(uint8_t *data, uint16_t length);

/**
 * @brief 将 CDC OUT 端点收到的数据转交给 USB 任务。
 */
void Driver_USB_Receive(const uint8_t *data, uint32_t length);

/**
 * @brief 将 CDC IN 端点发送完成事件转交给 USB 任务。
 */
void Driver_USB_TransmitComplete(void);

/**
 * @brief 通知 USB 任务 CDC 连接已经释放。
 */
void Driver_USB_Disconnect(void);

#ifdef __cplusplus
}
#endif

#endif
