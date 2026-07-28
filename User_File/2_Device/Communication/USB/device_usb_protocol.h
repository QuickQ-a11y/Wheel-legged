#ifndef DEVICE_USB_PROTOCOL_H
#define DEVICE_USB_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define USB_PROTOCOL_SOF 0xA0U
#define USB_PROTOCOL_CRC8_DATA_SIZE 4U
#define USB_PROTOCOL_HEADER_SIZE 7U
#define USB_PROTOCOL_CRC16_SIZE 2U
#define USB_PROTOCOL_MAX_PAYLOAD_SIZE 128U
#define USB_PROTOCOL_MAX_FRAME_SIZE \
    (USB_PROTOCOL_HEADER_SIZE + USB_PROTOCOL_MAX_PAYLOAD_SIZE + USB_PROTOCOL_CRC16_SIZE)

typedef struct __attribute__((packed))
{
    uint8_t sof;                         /* 固定为 0xA5。 */
    uint16_t dataLength;                 /* payload 字节数，低字节在前。 */
    uint8_t sequence;                    /* 每发送一帧递增一次。 */
    uint8_t crc8;                        /* 前 4 字节的 CRC8。 */
    uint16_t commandId;                  /* 业务命令号，低字节在前。 */
} usb_frame_header_t;

typedef struct __attribute__((packed))
{
    float roll;                          /* 下位机横滚角，单位由通信双方约定。 */
    float yaw;                           /* 下位机偏航角，单位由通信双方约定。 */
    float pitch;                         /* 下位机俯仰角，单位由通信双方约定。 */
    float bulletSpeed;                   /* 弹丸速度，单位由通信双方约定。 */
    float heat;                          /* 当前热量。 */
    uint8_t enemyColor;                  /* 敌方颜色编号。 */
    uint8_t mode;                        /* 当前业务模式。 */
} usb_status_payload_t;

typedef struct __attribute__((packed))
{
    float yaw;                           /* 视觉偏航控制量，单位由通信双方约定。 */
    float pitch;                         /* 视觉俯仰控制量，单位由通信双方约定。 */
    uint8_t shoot;                       /* 射击命令。 */
} usb_control_payload_t;

typedef struct __attribute__((packed))
{
    usb_frame_header_t header;
    usb_status_payload_t payload;
    uint16_t crc16;
} usb_status_packet_t;

typedef struct __attribute__((packed))
{
    usb_frame_header_t header;
    usb_control_payload_t payload;
    uint16_t crc16;
} usb_control_packet_t;

typedef void (*usb_protocol_frame_handler_t)(uint16_t commandId,
                                             uint8_t sequence,
                                             const uint8_t *payload,
                                             uint16_t payloadLength);

typedef struct
{
    uint8_t frameBuffer[USB_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t bufferedLength;
    uint32_t headerCrcErrorCount;
    uint32_t frameCrcErrorCount;
    uint32_t lengthErrorCount;
    usb_protocol_frame_handler_t frameHandler;
} usb_protocol_parser_t;

/**
 * @brief 初始化流式协议解析器。
 */
void USB_Protocol_Init(usb_protocol_parser_t *parser,
                       usb_protocol_frame_handler_t frameHandler);

/**
 * @brief 解析任意分段到达的 USB CDC 字节流。
 *
 * 解析器按 SOF、CRC8、长度和 CRC16 逐层确认边界，错误数据只丢弃到下一个
 * 可能的 0xA5，因此不要求一次 USB 接收恰好对应一帧协议数据。
 */
void USB_Protocol_Parse(usb_protocol_parser_t *parser,
                        const uint8_t *data,
                        uint16_t length);

/**
 * @brief 从有效载荷中逐项解码上位机控制数据。
 *
 * 调用前必须已经完成 payload 长度、CRC8 和 CRC16 校验。
 */
void USB_Protocol_UnpackControl(const uint8_t *payload,
                                usb_control_payload_t *control);

/**
 * @brief 封装一帧下位机状态数据。
 */
void USB_Protocol_PackStatus(uint16_t commandId,
                             uint8_t sequence,
                             const usb_status_payload_t *payload,
                             usb_status_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif
