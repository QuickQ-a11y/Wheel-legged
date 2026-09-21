#include "CRC.h"
#include "device_usb_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t receivedFrameCount;
static uint16_t receivedCommandId;
static uint8_t receivedSequence;
static usb_control_payload_t receivedControl;

static uint8_t Test_CalculateCrc8Reference(const uint8_t *data, uint32_t length)
{
    uint8_t crc = ALGORITHM_CRC8_INIT;
    uint32_t byteIndex;
    uint32_t bitIndex;

    for (byteIndex = 0U; byteIndex < length; byteIndex++)
    {
        crc ^= data[byteIndex];
        for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
        {
            crc = (crc & 1U) != 0U
                      ? (uint8_t)((crc >> 1U) ^ 0x8CU)
                      : (uint8_t)(crc >> 1U);
        }
    }

    return crc;
}

static uint16_t Test_CalculateCrc16Reference(const uint8_t *data, uint32_t length)
{
    uint16_t crc = ALGORITHM_CRC16_INIT;
    uint32_t byteIndex;
    uint32_t bitIndex;

    for (byteIndex = 0U; byteIndex < length; byteIndex++)
    {
        crc ^= data[byteIndex];
        for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
        {
            crc = (crc & 1U) != 0U
                      ? (uint16_t)((crc >> 1U) ^ 0x8408U)
                      : (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

static void Test_WriteUint16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0x00FFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void Test_WriteUint32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0x000000FFUL);
    data[1] = (uint8_t)((value >> 8U) & 0x000000FFUL);
    data[2] = (uint8_t)((value >> 16U) & 0x000000FFUL);
    data[3] = (uint8_t)(value >> 24U);
}

static void Test_WriteFloat(uint8_t *data, float value)
{
    union
    {
        uint32_t uint32Value;
        float floatValue;
    } converter;

    converter.floatValue = value;
    Test_WriteUint32(data, converter.uint32Value);
}

static void Test_BuildControlFrame(uint16_t commandId,
                                   uint8_t sequence,
                                   const usb_control_payload_t *control,
                                   uint8_t frame[sizeof(usb_control_packet_t)])
{
    uint16_t crc16;

    frame[0] = USB_PROTOCOL_SOF;
    Test_WriteUint16(&frame[1], sizeof(*control));
    frame[3] = sequence;
    frame[4] = Test_CalculateCrc8Reference(frame, 4U);
    Test_WriteUint16(&frame[5], commandId);
    Test_WriteFloat(
        &frame[USB_PROTOCOL_HEADER_SIZE + offsetof(usb_control_payload_t, yaw)],
        control->yaw);
    Test_WriteFloat(
        &frame[USB_PROTOCOL_HEADER_SIZE + offsetof(usb_control_payload_t, pitch)],
        control->pitch);
    frame[USB_PROTOCOL_HEADER_SIZE + offsetof(usb_control_payload_t, shoot)] =
        control->shoot;

    crc16 = Test_CalculateCrc16Reference(
        frame,
        sizeof(usb_control_packet_t) - USB_PROTOCOL_CRC16_SIZE);
    Test_WriteUint16(&frame[sizeof(usb_control_packet_t) - USB_PROTOCOL_CRC16_SIZE],
                     crc16);
}

static void Test_FrameReceived(uint16_t commandId,
                               uint8_t sequence,
                               const uint8_t *payload,
                               uint16_t payloadLength)
{
    assert(payloadLength == sizeof(receivedControl));
    receivedFrameCount++;
    receivedCommandId = commandId;
    receivedSequence = sequence;
    USB_Protocol_UnpackControl(payload, &receivedControl);
}

int main(void)
{
    static const uint8_t testText[] = "123456789";
    static const uint8_t expectedStatusPayload[] = {
        0x00U, 0x00U, 0x80U, 0x3FU, /* roll = 1.0f */
        0x00U, 0x00U, 0x00U, 0xC0U, /* yaw = -2.0f */
        0x00U, 0x00U, 0x00U, 0x3FU, /* pitch = 0.5f */
        0x00U, 0x00U, 0x70U, 0x41U, /* bulletSpeed = 15.0f */
        0x00U, 0x00U, 0xA0U, 0x41U, /* heat = 20.0f */
        0x02U,                       /* enemyColor = 2 */
        0x03U,                       /* mode = 3 */
    };
    usb_protocol_parser_t parser;
    usb_control_payload_t control = {
        .yaw = 1.25f,
        .pitch = -0.75f,
        .shoot = 1U,
    };
    usb_status_payload_t status = {
        .roll = 1.0f,
        .yaw = -2.0f,
        .pitch = 0.5f,
        .bulletSpeed = 15.0f,
        .heat = 20.0f,
        .enemyColor = 2U,
        .mode = 3U,
    };
    usb_status_packet_t statusPacket;
    const uint8_t *statusFrame = (const uint8_t *)&statusPacket;
    uint16_t statusCrc16;
    uint8_t frame[sizeof(usb_control_packet_t)];
    uint8_t corruptedFrame[sizeof(frame)];
    uint8_t noise[] = {0x00U, 0x5AU, 0xFFU};

    assert(Algorithm_CRC8_Calculate(testText,
                                    sizeof(testText) - 1U,
                                    ALGORITHM_CRC8_INIT) ==
           Test_CalculateCrc8Reference(testText, sizeof(testText) - 1U));
    assert(Algorithm_CRC16_Calculate(testText,
                                     sizeof(testText) - 1U,
                                     ALGORITHM_CRC16_INIT) ==
           Test_CalculateCrc16Reference(testText, sizeof(testText) - 1U));

    USB_Protocol_PackStatus(0x5678U, 9U, &status, &statusPacket);
    statusCrc16 = Test_CalculateCrc16Reference(
        statusFrame,
        sizeof(statusPacket) - USB_PROTOCOL_CRC16_SIZE);
    assert(statusFrame[0] == USB_PROTOCOL_SOF);
    assert(statusFrame[1] == sizeof(status));
    assert(statusFrame[2] == 0U);
    assert(statusFrame[3] == 9U);
    assert(statusFrame[4] == Test_CalculateCrc8Reference(statusFrame, 4U));
    assert(statusFrame[5] == 0x78U);
    assert(statusFrame[6] == 0x56U);
    assert(memcmp(&statusFrame[USB_PROTOCOL_HEADER_SIZE],
                  expectedStatusPayload,
                  sizeof(expectedStatusPayload)) == 0);
    assert(statusFrame[sizeof(statusPacket) - 2U] ==
           (uint8_t)(statusCrc16 & 0x00FFU));
    assert(statusFrame[sizeof(statusPacket) - 1U] ==
           (uint8_t)(statusCrc16 >> 8U));

    Test_BuildControlFrame(0x1234U, 7U, &control, frame);
    USB_Protocol_Init(&parser, Test_FrameReceived);

    USB_Protocol_Parse(&parser, noise, sizeof(noise));
    USB_Protocol_Parse(&parser, frame, 4U);
    USB_Protocol_Parse(&parser, &frame[4], sizeof(frame) - 4U);

    assert(receivedFrameCount == 1U);
    assert(receivedCommandId == 0x1234U);
    assert(receivedSequence == 7U);
    assert(receivedControl.yaw == control.yaw);
    assert(receivedControl.pitch == control.pitch);
    assert(receivedControl.shoot == control.shoot);

    memcpy(corruptedFrame, frame, sizeof(frame));
    corruptedFrame[sizeof(corruptedFrame) - 1U] ^= 0x01U;
    USB_Protocol_Parse(&parser, corruptedFrame, sizeof(corruptedFrame));
    USB_Protocol_Parse(&parser, frame, sizeof(frame));

    assert(receivedFrameCount == 2U);
    assert(parser.frameCrcErrorCount == 1U);

    puts("USB protocol tests passed");
    return 0;
}
