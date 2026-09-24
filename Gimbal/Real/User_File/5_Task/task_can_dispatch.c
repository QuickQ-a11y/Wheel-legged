#include "task_can_dispatch.h"

#include "task_can.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"

/**
 * @brief 分发应用层 CAN 接收报文。
 *
 * 当前硬件映射：
 * FDCAN2 接三个摩擦轮 M2006（C610，电调 ID 1~3）；
 * FDCAN3 接云台 Yaw、Pitch 两台 DM4310，外加拨盘 M2006（电调 ID 4）和板间报文。
 *
 * 两个设备层各自比对总线和 ID，这里不做任何 ID 判断——总线归属只在设备层
 * 定义一次（Motor_DJI_BusOf / motor_dm_config_t.bus），分发层再判一遍就成了
 * 第二个真相来源，换线时必然漏改一处。
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

    if ((message->length == APP_DJI_RX_LEN) &&
        (Motor_DJI_UpdateFeedback(message->bus,
                                  message->identifier,
                                  message->data) != 0U))
    {
        return;
    }
}
