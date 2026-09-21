#ifndef TASK_CAN_H
#define TASK_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "fdcan.h"

#include <stdint.h>

typedef struct
{
    app_can_bus_t bus;                                   /* 报文来源 CAN 总线。 */
    uint32_t identifier;                                 /* 标准帧 ID。 */
    uint8_t length;                                      /* 数据长度，最大 8。 */
    uint8_t data[APP_CAN_DATA_MAX_BYTES];        /* Classic CAN 数据。 */
} task_can_rx_message_t;

void CAN_Task_Init(void);
void CAN_Task_SetDjiCurrent(const int16_t current[APP_WHEEL_COUNT]);
void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle,
                            uint32_t identifier,
                            const uint8_t *data,
                            uint8_t length);
void CAN_Task_RequestTx(void);
uint32_t CAN_Task_GetTxErrorCount(void);
void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message);

#ifdef __cplusplus
}
#endif

#endif
