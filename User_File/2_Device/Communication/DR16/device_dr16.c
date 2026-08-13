#include "device_dr16.h"

#include <stddef.h>

#define DR16_AXIS_MAX 660

/** @brief 读取一个小端无符号 16 位字段。 */
static uint16_t DR16_ReadUint16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/** @brief 在不使用别名类型转换的情况下读取小端有符号 16 位字段。 */
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
    parsed.mouse.x = (int16_t)DR16_ReadUint16(&frame[6]);
    parsed.mouse.y = (int16_t)DR16_ReadUint16(&frame[8]);
    parsed.mouse.z = (int16_t)DR16_ReadUint16(&frame[10]);
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

float DR16_NormalizeAxis(int16_t axis, int16_t deadband)
{
    int32_t limitedAxis = axis;
    int32_t positiveDeadband = deadband;
    int32_t magnitude;

    if (limitedAxis > DR16_AXIS_MAX)
    {
        limitedAxis = DR16_AXIS_MAX;
    }
    else if (limitedAxis < -DR16_AXIS_MAX)
    {
        limitedAxis = -DR16_AXIS_MAX;
    }

    if (positiveDeadband < 0)
    {
        positiveDeadband = -positiveDeadband;
    }
    if (positiveDeadband >= DR16_AXIS_MAX)
    {
        return 0.0f;
    }

    magnitude = (limitedAxis >= 0) ? limitedAxis : -limitedAxis;
    if (magnitude <= positiveDeadband)
    {
        return 0.0f;
    }

    magnitude -= positiveDeadband;
    if (limitedAxis < 0)
    {
        return -(float)magnitude / (float)(DR16_AXIS_MAX - positiveDeadband);
    }

    return (float)magnitude / (float)(DR16_AXIS_MAX - positiveDeadband);
}

static Remote_Switch_t DR16_ConvertSwitch(dr16_switch_t value)
{
    switch (value)
    {
    case DR16_SWITCH_UP:
        return REMOTE_SWITCH_UP;

    case DR16_SWITCH_DOWN:
        return REMOTE_SWITCH_DOWN;

    case DR16_SWITCH_MID:
        return REMOTE_SWITCH_MID;

    case DR16_SWITCH_UNKNOWN:
    default:
        return REMOTE_SWITCH_UNKNOWN;
    }
}

void DR16_MakeRemote(const dr16_data_t *data,
                     int16_t deadband,
                     int16_t dialThreshold,
                     Remote_t *remote)
{
    Remote_t converted = {0};

    converted.leftStick.x = DR16_NormalizeAxis(data->leftX, deadband);
    converted.leftStick.y = DR16_NormalizeAxis(data->leftY, deadband);
    converted.rightStick.x = DR16_NormalizeAxis(data->rightX, deadband);
    converted.rightStick.y = DR16_NormalizeAxis(data->rightY, deadband);
    converted.leftSwitch = DR16_ConvertSwitch(data->leftSwitch);
    converted.rightSwitch = DR16_ConvertSwitch(data->rightSwitch);
    converted.dialValid = data->dialValid;
    if (data->dialValid != 0U)
    {
        converted.dial = DR16_NormalizeAxis(data->dial, deadband);
    }

    /*
     * 右拨杆决定使能级别，右上时才由左拨杆选模式。
     * 具体档位对应哪个模式全部由 remote_input.h 的 REMOTE_MAP_* 决定，
     * 改拨杆分配不需要动这里。右下急停由 rightSwitch 字段单独上报。
     */
    if (converted.rightSwitch == REMOTE_SWITCH_MID)
    {
        converted.modeRequest = REMOTE_MAP_RIGHT_MID;
    }
    else if (converted.rightSwitch == REMOTE_SWITCH_UP)
    {
        switch (converted.leftSwitch)
        {
        case REMOTE_SWITCH_UP:
            converted.modeRequest = REMOTE_MAP_LEFT_UP;
            break;

        case REMOTE_SWITCH_MID:
            converted.modeRequest = REMOTE_MAP_LEFT_MID;
            break;

        case REMOTE_SWITCH_DOWN:
            converted.modeRequest = REMOTE_MAP_LEFT_DOWN;
            break;

        case REMOTE_SWITCH_UNKNOWN:
        default:
            converted.modeRequest = REMOTE_MODE_NONE;
            break;
        }
    }
    else
    {
        converted.modeRequest = REMOTE_MODE_NONE;
    }

    if (data->dialValid == 0U)
    {
        converted.legRequest = REMOTE_LEG_KEEP;
    }
    else if (data->dial > dialThreshold)
    {
        converted.legRequest = REMOTE_LEG_SHORT;
    }
    else if (data->dial < -dialThreshold)
    {
        converted.legRequest = REMOTE_LEG_LONG;
    }
    else
    {
        converted.legRequest = REMOTE_LEG_MIDDLE;
    }

    *remote = converted;
}
