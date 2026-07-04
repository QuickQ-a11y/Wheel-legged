#include "task_can_dispatch.h"

#include "task_can.h"
#include "device_motor_dm.h"

motor_dji_t chassisDjiWheels[APP_CONFIG_DJI_WHEEL_COUNT] = {0};

/**
 * @brief 分发应用层 CAN 接收报文。
 *
 * 当前硬件映射：
 * FDCAN1 接 4 个 DM 髋关节电机；
 * FDCAN2 接 2 个 DJI 轮电机，反馈 ID 为 0x201 和 0x202。
 */
void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message)
{
    if (message == NULL)
    {
        return;
    }

    if ((message->bus == APP_CAN_BUS_FDCAN1) &&
        (message->length == APP_CONFIG_DM_FRAME_LENGTH))
    {
        Motor_DM_UpdateFeedback(message->bus, message->identifier, message->data);
        return;
    }

    if ((message->bus != APP_CAN_BUS_FDCAN2) ||
        (message->length != APP_CONFIG_DJI_FEEDBACK_LENGTH))
    {
        return;
    }

    switch (message->identifier)
    {
    case APP_CONFIG_DJI_LEFT_FEEDBACK_ID:
        Motor_DJI_UpdateFeedback(&chassisDjiWheels[0U], message->data);
        break;

    case APP_CONFIG_DJI_RIGHT_FEEDBACK_ID:
        Motor_DJI_UpdateFeedback(&chassisDjiWheels[1U], message->data);
        break;

    default:
        break;
    }
}
