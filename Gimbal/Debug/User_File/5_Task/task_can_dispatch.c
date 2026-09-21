#include "task_can_dispatch.h"

#include "task_can.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"

/**
 * @brief 分发应用层 CAN 接收报文。
 *
 * 当前硬件映射：
 * FDCAN1 接云台 Yaw、Pitch 两台 DM 电机；
 * FDCAN2 接发射机构的拨弹盘和两个摩擦轮 DJI 电调。
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
    case APP_DJI_TRIGGER_RX_ID:
        Motor_DJI_UpdateFeedback(MOTOR_DJI_TRIGGER, message->data);
        break;

    case APP_DJI_FRIC_L_RX_ID:
        Motor_DJI_UpdateFeedback(MOTOR_DJI_FRIC_L, message->data);
        break;

    case APP_DJI_FRIC_R_RX_ID:
        Motor_DJI_UpdateFeedback(MOTOR_DJI_FRIC_R, message->data);
        break;

    default:
        break;
    }
}
