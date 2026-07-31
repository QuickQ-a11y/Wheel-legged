#include "device_dr16.h"

#include <stddef.h>

#define DR16_AXIS_MAX 660

/** @brief 读取一个小端无符号 16 位字段。 */
static uint16_t DR16_ReadUint16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/** @brief 在不使用别名类型转换的情况下读取小端有符号 16 位字段。 */
static int16_t DR16_ReadInt16(const uint8_t *data)
{
    return (int16_t)DR16_ReadUint16(data);
}

/** @brief 在减去中值前校验一个无符号 DBUS 通道原始值。 */
static uint8_t DR16_IsChannelValid(uint16_t channel)
{
    return ((channel >= DR16_CH_MIN) &&
            (channel <= DR16_CH_MAX)) ? 1U : 0U;
}

/** @brief 校验 DT7 拨杆的三个合法位置。 */
static uint8_t DR16_IsSwitchValid(dr16_switch_t switchValue)
{
    return ((switchValue == DR16_SWITCH_UP) ||
            (switchValue == DR16_SWITCH_DOWN) ||
            (switchValue == DR16_SWITCH_MID)) ? 1U : 0U;
}

uint8_t DR16_ParseFrame(const uint8_t frame[DR16_FRAME_LEN],
                        dr16_data_t *data)
{
    dr16_data_t parsed = {0};
    uint16_t channels[4];
    uint16_t dialRaw;

    if ((frame == NULL) || (data == NULL))
    {
        return 0U;
    }

    channels[0] = (DR16_ReadUint16(&frame[0]) & 0x07FFU);
    channels[1] = (((uint16_t)frame[1] >> 3U) |
                   ((uint16_t)frame[2] << 5U)) & 0x07FFU;
    channels[2] = (((uint16_t)frame[2] >> 6U) |
                   ((uint16_t)frame[3] << 2U) |
                   ((uint16_t)frame[4] << 10U)) & 0x07FFU;
    channels[3] = (((uint16_t)frame[4] >> 1U) |
                   ((uint16_t)frame[5] << 7U)) & 0x07FFU;

    parsed.rightSwitch = (dr16_switch_t)((frame[5] >> 4U) & 0x03U);
    parsed.leftSwitch = (dr16_switch_t)((frame[5] >> 6U) & 0x03U);
    if ((DR16_IsChannelValid(channels[0]) == 0U) ||
        (DR16_IsChannelValid(channels[1]) == 0U) ||
        (DR16_IsChannelValid(channels[2]) == 0U) ||
        (DR16_IsChannelValid(channels[3]) == 0U) ||
        (DR16_IsSwitchValid(parsed.rightSwitch) == 0U) ||
        (DR16_IsSwitchValid(parsed.leftSwitch) == 0U))
    {
        return 0U;
    }

    parsed.rightX = (int16_t)channels[0] - (int16_t)DR16_CH_MID;
    parsed.rightY = (int16_t)channels[1] - (int16_t)DR16_CH_MID;
    parsed.leftX = (int16_t)channels[2] - (int16_t)DR16_CH_MID;
    parsed.leftY = (int16_t)channels[3] - (int16_t)DR16_CH_MID;
    parsed.mouse.x = DR16_ReadInt16(&frame[6]);
    parsed.mouse.y = DR16_ReadInt16(&frame[8]);
    parsed.mouse.z = DR16_ReadInt16(&frame[10]);
    parsed.mouse.leftPressed = frame[12];
    parsed.mouse.rightPressed = frame[13];
    parsed.keyBits = DR16_ReadUint16(&frame[14]);

    dialRaw = DR16_ReadUint16(&frame[16]) & 0x07FFU;
    if (DR16_IsChannelValid(dialRaw) != 0U)
    {
        parsed.dial = (int16_t)dialRaw - (int16_t)DR16_CH_MID;
        parsed.dialValid = 1U;
    }

    *data = parsed;
    return 1U;
}

uint8_t DR16_IsKeyDown(const dr16_data_t *data, dr16_key_t key)
{
    if (data == NULL)
    {
        return 0U;
    }

    return ((data->keyBits & (uint16_t)key) != 0U) ? 1U : 0U;
}

float DR16_NormalizeAxis(int16_t axis, int16_t deadband)
{
    int32_t limited_axis = axis;
    int32_t positive_deadband = deadband;
    int32_t magnitude;

    if (limited_axis > DR16_AXIS_MAX)
    {
        limited_axis = DR16_AXIS_MAX;
    }
    else if (limited_axis < -DR16_AXIS_MAX)
    {
        limited_axis = -DR16_AXIS_MAX;
    }

    if (positive_deadband < 0)
    {
        positive_deadband = -positive_deadband;
    }
    if (positive_deadband >= DR16_AXIS_MAX)
    {
        return 0.0f;
    }

    magnitude = (limited_axis >= 0) ? limited_axis : -limited_axis;
    if (magnitude <= positive_deadband)
    {
        return 0.0f;
    }

    magnitude -= positive_deadband;
    if (limited_axis < 0)
    {
        return -(float)magnitude / (float)(DR16_AXIS_MAX - positive_deadband);
    }

    return (float)magnitude / (float)(DR16_AXIS_MAX - positive_deadband);
}

void DR16_MakeInput(const dr16_data_t *data,
                    int16_t deadband,
                    int16_t dialThreshold,
                    remote_input_t *input)
{
    if ((data == NULL) || (input == NULL))
    {
        return;
    }

    input->forwardAxis = DR16_NormalizeAxis(data->leftY, deadband);
    input->yawAxis = DR16_NormalizeAxis(data->rightX, deadband);
    input->stop = (data->rightSwitch == DR16_SWITCH_DOWN) ? 1U : 0U;

    switch (data->leftSwitch)
    {
    case DR16_SWITCH_UP:
        input->modeRequest = REMOTE_MODE_FOLLOW;
        break;

    case DR16_SWITCH_MID:
        input->modeRequest = REMOTE_MODE_BENCH;
        break;

    case DR16_SWITCH_DOWN:
        input->modeRequest = REMOTE_MODE_SELF_SAVE;
        break;

    case DR16_SWITCH_UNKNOWN:
    default:
        input->modeRequest = REMOTE_MODE_NONE;
        break;
    }

    if (data->dialValid == 0U)
    {
        input->legRequest = REMOTE_LEG_KEEP;
    }
    else if (data->dial > dialThreshold)
    {
        input->legRequest = REMOTE_LEG_SHORT;
    }
    else if (data->dial < -dialThreshold)
    {
        input->legRequest = REMOTE_LEG_LONG;
    }
    else
    {
        input->legRequest = REMOTE_LEG_MIDDLE;
    }
}
