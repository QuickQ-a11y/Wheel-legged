#include "task_can_dispatch.h"

#include "task_can.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"

/**
 * @brief 分发应用层 CAN 接收报文。
 *
 * 当前硬件映射：
 * FDCAN1 接 4 个 DM 髋关节电机；
 * FDCAN2 接 2 个 DJI 轮电机，反馈 ID 为 0x201 和 0x202。
 */
void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message)
{
    if ((message->bus == APP_CAN_BUS_FDCAN1) &&
        (message->length == APP_DM_FRAME_LEN))
    {
        Motor_DM_UpdateFeedback(message->bus, message->identifier, message->data);
        return;
    }

    if ((message->bus != APP_CAN_BUS_FDCAN2) ||
        (message->length != APP_DJI_RX_LEN))
    {
        return;
    }

    switch (message->identifier)
    {
    case APP_DJI_LEFT_RX_ID:
        Motor_DJI_UpdateFeedback(MOTOR_DJI_LEFT, message->data);
        break;

    case APP_DJI_RIGHT_RX_ID:
        Motor_DJI_UpdateFeedback(MOTOR_DJI_RIGHT, message->data);
        break;

    default:
        break;
    }
}
