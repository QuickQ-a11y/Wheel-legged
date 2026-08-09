#include "task_can_dispatch.h"

#include "task_can.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"

/**
 * @brief 分发应用层 CAN 接收报文。
 *
 * 当前硬件映射：
 * FDCAN1 接左侧 2 个 DM 髋关节电机；
 * FDCAN2 接右侧 2 个 DM 髋关节电机和 2 个 DJI 轮电机。
 */
void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message)
{
    if ((message->length == APP_DM_FRAME_LEN) &&
        (Motor_DM_UpdateFeedback(message->bus,
                                 message->identifier,
                                 message->data) != 0U))
    {
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
