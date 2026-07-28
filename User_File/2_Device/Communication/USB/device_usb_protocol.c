#include "device_usb_protocol.h"

#include "CRC.h"

#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(usb_frame_header_t) == 7U, "USB frame header size mismatch");
_Static_assert(sizeof(usb_status_payload_t) == 22U, "USB status payload size mismatch");
_Static_assert(sizeof(usb_control_payload_t) == 9U, "USB control payload size mismatch");
_Static_assert(sizeof(usb_status_packet_t) == 31U, "USB status packet size mismatch");
_Static_assert(sizeof(usb_control_packet_t) == 18U, "USB control packet size mismatch");
_Static_assert(sizeof(float) == 4U, "USB protocol requires 32-bit float");

/**
 * @brief 从协议字节流读取低字节在前的 16 位数值。
 */
static uint16_t USB_Protocol_ReadUint16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 从协议字节流读取低字节在前的 32 位数值。
 */
static uint32_t USB_Protocol_ReadUint32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief 从协议字节流读取 IEEE-754 单精度浮点数。
 */
static float USB_Protocol_ReadFloat(const uint8_t *data)
{
    union
    {
        uint32_t uint32Value;
        float floatValue;
    } converter;

    converter.uint32Value = USB_Protocol_ReadUint32(data);
    return converter.floatValue;
}

/**
 * @brief 从解析缓存头部移除已经处理或确认错误的字节。
 */
static void USB_Protocol_RemoveBytes(usb_protocol_parser_t *parser,
                                     uint16_t removeLength)
{
    parser->bufferedLength -= removeLength;
    memmove(parser->frameBuffer,
            &parser->frameBuffer[removeLength],
            parser->bufferedLength);
}

/**
 * @brief 处理当前缓存中能够确定边界的全部协议帧。
 */
static void USB_Protocol_ParseBufferedData(usb_protocol_parser_t *parser)
{
    uint16_t sofIndex;
    uint16_t payloadLength;
    uint16_t frameLength;
    uint16_t commandId;
    uint16_t receivedCrc16;
    uint16_t calculatedCrc16;

    for (;;)
    {
        /* 先删除 SOF 前的无效字节，使缓存首字节始终是候选帧起点。 */
        for (sofIndex = 0U;
             sofIndex < parser->bufferedLength;
             sofIndex++)
        {
            if (parser->frameBuffer[sofIndex] == USB_PROTOCOL_SOF)
            {
                break;
            }
        }

        if (sofIndex == parser->bufferedLength)
        {
            parser->bufferedLength = 0U;
            return;
        }

        if (sofIndex > 0U)
        {
            USB_Protocol_RemoveBytes(parser, sofIndex);
        }

        if (parser->bufferedLength < 5U)
        {
            return;
        }

        /* CRC8 位于第 5 字节，因此校验范围是它之前的 4 字节。 */
        if (Algorithm_CRC8_Calculate(parser->frameBuffer,
                                     USB_PROTOCOL_CRC8_DATA_SIZE,
                                     ALGORITHM_CRC8_INIT) !=
            parser->frameBuffer[4])
        {
            parser->headerCrcErrorCount++;
            USB_Protocol_RemoveBytes(parser, 1U);
            continue;
        }

        if (parser->bufferedLength < USB_PROTOCOL_HEADER_SIZE)
        {
            return;
        }

        payloadLength = USB_Protocol_ReadUint16(&parser->frameBuffer[1]);
        if (payloadLength > USB_PROTOCOL_MAX_PAYLOAD_SIZE)
        {
            parser->lengthErrorCount++;
            USB_Protocol_RemoveBytes(parser, 1U);
            continue;
        }

        frameLength = USB_PROTOCOL_HEADER_SIZE +
                      payloadLength +
                      USB_PROTOCOL_CRC16_SIZE;
        if (parser->bufferedLength < frameLength)
        {
            return;
        }

        receivedCrc16 = USB_Protocol_ReadUint16(
            &parser->frameBuffer[frameLength - USB_PROTOCOL_CRC16_SIZE]);
        calculatedCrc16 = Algorithm_CRC16_Calculate(parser->frameBuffer,
                                                    frameLength - USB_PROTOCOL_CRC16_SIZE,
                                                    ALGORITHM_CRC16_INIT);
        if (receivedCrc16 != calculatedCrc16)
        {
            parser->frameCrcErrorCount++;
            USB_Protocol_RemoveBytes(parser, 1U);
            continue;
        }

        commandId = USB_Protocol_ReadUint16(&parser->frameBuffer[5]);
        parser->frameHandler(commandId,
                             parser->frameBuffer[3],
                             &parser->frameBuffer[USB_PROTOCOL_HEADER_SIZE],
                             payloadLength);
        USB_Protocol_RemoveBytes(parser, frameLength);
    }
}

void USB_Protocol_Init(usb_protocol_parser_t *parser,
                       usb_protocol_frame_handler_t frameHandler)
{
    memset(parser, 0, sizeof(*parser));
    parser->frameHandler = frameHandler;
}

void USB_Protocol_Parse(usb_protocol_parser_t *parser,
                        const uint8_t *data,
                        uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++)
    {
        if (parser->bufferedLength >= USB_PROTOCOL_MAX_FRAME_SIZE)
        {
            parser->lengthErrorCount++;
            USB_Protocol_RemoveBytes(parser, 1U);
        }

        parser->frameBuffer[parser->bufferedLength] = data[index];
        parser->bufferedLength++;
        USB_Protocol_ParseBufferedData(parser);
    }
}

void USB_Protocol_UnpackControl(const uint8_t *payload,
                                usb_control_payload_t *control)
{
    /* 每个字段都从协议中的固定位置读取，新增字段时在这里直接追加。 */
    control->yaw = USB_Protocol_ReadFloat(
        &payload[offsetof(usb_control_payload_t, yaw)]);
    control->pitch = USB_Protocol_ReadFloat(
        &payload[offsetof(usb_control_payload_t, pitch)]);
    control->shoot = payload[offsetof(usb_control_payload_t, shoot)];
}

void USB_Protocol_PackStatus(uint16_t commandId,
                             uint8_t sequence,
                             const usb_status_payload_t *payload,
                             usb_status_packet_t *packet)
{
    /* 帧头直接赋值，CRC8 只覆盖 SOF、长度和序号。 */
    packet->header.sof = USB_PROTOCOL_SOF;
    packet->header.dataLength = (uint16_t)sizeof(packet->payload);
    packet->header.sequence = sequence;
    packet->header.commandId = commandId;
    packet->header.crc8 = Algorithm_CRC8_Calculate(
        (const uint8_t *)&packet->header,
        USB_PROTOCOL_CRC8_DATA_SIZE,
        ALGORITHM_CRC8_INIT);

    /* 状态数据直接赋值，新增发送字段时在这里按协议顺序追加。 */
    packet->payload.roll = payload->roll;
    packet->payload.yaw = payload->yaw;
    packet->payload.pitch = payload->pitch;
    packet->payload.bulletSpeed = payload->bulletSpeed;
    packet->payload.heat = payload->heat;
    packet->payload.enemyColor = payload->enemyColor;
    packet->payload.mode = payload->mode;

    /* STM32H723 使用小端布局，CRC16 直接写入包尾。 */
    packet->crc16 = Algorithm_CRC16_Calculate(
        (const uint8_t *)packet,
        sizeof(*packet) - USB_PROTOCOL_CRC16_SIZE,
        ALGORITHM_CRC16_INIT);
}
