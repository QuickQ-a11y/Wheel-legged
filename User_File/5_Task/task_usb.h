#ifndef TASK_USB_H
#define TASK_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_usb_protocol.h"

#include <stdint.h>

typedef void (*task_usb_control_handler_t)(uint16_t commandId,
                                           uint8_t sequence,
                                           const usb_control_payload_t *control);

typedef struct
{
    usb_protocol_parser_t rxParser;       /* 接收流解析状态，以及 CRC8、CRC16 和长度错误累计值。 */
    usb_control_payload_t recentControl;  /* 最近一帧有效控制数据：yaw、pitch 和 shoot。 */
    uint16_t recentControlCmdId;          /* 最近一帧有效控制数据对应的 cmd_id。 */
    uint8_t recentControlSeq;             /* 最近一帧有效控制数据对应的 sequence。 */
    uint8_t recentRxSeq;                  /* 最近一帧 CRC8、CRC16 均正确的 sequence。 */
    uint32_t rxByteCount;                 /* 本次上电后 CDC OUT 收到的累计字节数。 */
    uint32_t rxFrameCount;                /* 本次上电后 CRC 正确的累计帧数，包含重复帧。 */
    uint32_t rxLostCount;                 /* 本次上电后根据 sequence 跳跃累计的缺失帧数。 */
    uint32_t rxDuplicateCount;            /* 本次上电后与上一有效 sequence 相同的重复帧数。 */
    uint32_t rxSeqResetCount;             /* 本次上电后 sequence 大跨度反向并重新同步的次数。 */
    uint32_t rxControlCount;              /* 本次上电后 payload 长度符合控制数据的累计帧数。 */
    uint32_t rxOtherCount;                /* 本次上电后 CRC 正确但 payload 长度不符的累计帧数。 */
    uint32_t rxOverflowCount;             /* 本次上电后接收环形缓冲溢出丢弃的累计字节数。 */
    uint32_t txQueuedCount;               /* 本次上电后写入发送 FIFO 的累计状态帧数。 */
    uint32_t txCompleteCount;             /* 本次上电后 CDC IN 完成的累计状态帧数。 */
    uint32_t txDroppedCount;              /* 本次上电后因 FIFO 覆盖、断开或错误丢弃的累计帧数。 */
    uint32_t txBusyCount;                 /* 本次上电后 CDC 返回 BUSY 的累计次数。 */
    uint32_t txErrorCount;                /* 本次上电后 CDC 返回非 OK、非 BUSY 的累计次数。 */
    uint32_t rxByteRateBps;               /* 最近一秒的接收吞吐量，单位 B/s。 */
    uint32_t rxFrameRateHz;               /* 最近一秒 CRC 正确的接收帧频，单位 Hz。 */
    float rxLossPercent;                  /* 本次上电以来按 sequence 计算的累计接收丢包率，单位 %。 */
    uint32_t txByteRateBps;               /* 最近一秒完成发送的吞吐量，单位 B/s。 */
    uint32_t txFrameRateHz;               /* 最近一秒完成发送的状态帧频，单位 Hz。 */
    float txLossPercent;                  /* 本次上电以来发送完成与丢弃帧计算的累计丢包率，单位 %。 */
} task_usb_debug_t;

extern uint16_t usbTaskTxCommandId;              /* 当前状态帧命令号，调试阶段可直接修改。 */
extern usb_status_payload_t usbTaskTxData;       /* 当前实际发送的状态数据。 */
extern task_usb_debug_t usbTaskDebugState;

/**
 * @brief 初始化 USB CDC、软件缓冲和协议任务。
 */
void USB_Task_Init(void);

/**
 * @brief 注册视觉控制帧处理函数。
 */
void USB_Task_SetControlHandler(task_usb_control_handler_t controlHandler);

/**
 * @brief 将一帧下位机状态加入 USB 发送队列。
 *
 * commandId 由业务层定义；本函数在任务上下文调用，不在中断中调用。
 */
void USB_Task_SendStatus(uint16_t commandId,
                         const usb_status_payload_t *status);

#ifdef __cplusplus
}
#endif

#endif
