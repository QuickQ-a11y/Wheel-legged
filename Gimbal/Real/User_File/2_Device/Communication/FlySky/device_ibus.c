#include "device_ibus.h"

#include <string.h>

/* 通道量程的半幅，归一化用。中位 1500，两端 1000/2000。 */
#define IBUS_AXIS_MAX 500

uint8_t IBUS_ParseFrame(const uint8_t frame[IBUS_FRAME_LEN],
                         ibus_data_t *data)
{
    ibus_data_t parsed;
    uint16_t sum = 0U;
    uint16_t checksum;
    uint32_t index;

    if ((frame[0] != IBUS_HEADER_LEN) || (frame[1] != IBUS_HEADER_CMD))
    {
        return 0U;
    }

    /* 校验和覆盖除末尾两字节以外的全部内容。 */
    for (index = 0U; index < (IBUS_FRAME_LEN - 2U); index++)
    {
        sum = (uint16_t)(sum + frame[index]);
    }
    checksum = (uint16_t)((uint16_t)frame[31] << 8U) | (uint16_t)frame[30];
    if (checksum != (uint16_t)(0xFFFFU - sum))
    {
        return 0U;
    }

    for (index = 0U; index < IBUS_CH_COUNT; index++)
    {
        parsed.channel[index] =
            (uint16_t)((uint16_t)frame[3U + (index * 2U)] << 8U) |
            (uint16_t)frame[2U + (index * 2U)];
    }

    *data = parsed;
    return 1U;
}

float IBUS_NormalizeAxis(int16_t axis, int16_t deadband)
{
    int32_t limitedAxis = axis;
    int32_t positiveDeadband = deadband;
    int32_t magnitude;

    if (limitedAxis > IBUS_AXIS_MAX)
    {
        limitedAxis = IBUS_AXIS_MAX;
    }
    else if (limitedAxis < -IBUS_AXIS_MAX)
    {
        limitedAxis = -IBUS_AXIS_MAX;
    }

    if (positiveDeadband < 0)
    {
        positiveDeadband = -positiveDeadband;
    }
    if (positiveDeadband >= IBUS_AXIS_MAX)
    {
        return 0.0f;
    }

    magnitude = (limitedAxis >= 0) ? limitedAxis : -limitedAxis;
    if (magnitude <= positiveDeadband)
    {
        return 0.0f;
    }

    /* 去掉死区后重新铺满，保证满杆仍然是 1。 */
    magnitude -= positiveDeadband;
    if (limitedAxis < 0)
    {
        return -(float)magnitude / (float)(IBUS_AXIS_MAX - positiveDeadband);
    }

    return (float)magnitude / (float)(IBUS_AXIS_MAX - positiveDeadband);
}

Remote_Switch_t IBUS_ConvertSwitch(uint16_t channel)
{
    /*
     * ⚠ i6X 的拨杆是【上小下大】：实测 UP=1000、MID=1500、DOWN=2000，
     * 和"值大 = 位置高"的直觉相反，所以这里低值判 UP、高值判 DOWN。
     * REMOTE_SWITCH_UP 的语义必须和 DR16 一致 —— 表示拨杆物理朝上。
     */
    if (channel <= IBUS_SW_LOW_MAX)
    {
        return REMOTE_SWITCH_UP;
    }
    if (channel >= IBUS_SW_HIGH_MIN)
    {
        return REMOTE_SWITCH_DOWN;
    }

    return REMOTE_SWITCH_MID;
}

/** @brief 把单个通道去中值后交给归一化。 */
static float IBUS_Axis(const ibus_data_t *data,
                        uint32_t channel,
                        int16_t deadband)
{
    return IBUS_NormalizeAxis(
        (int16_t)((int32_t)data->channel[channel] - (int32_t)IBUS_CH_MID),
        deadband);
}

void IBUS_MakeRemote(const ibus_data_t *data,
                      int16_t deadband,
                      Remote_t *remote)
{
    Remote_t converted = {0};

    converted.rightStick.x = IBUS_Axis(data, IBUS_CH_RIGHT_X, deadband);
    converted.rightStick.y = IBUS_Axis(data, IBUS_CH_RIGHT_Y, deadband);
    converted.leftStick.x = IBUS_Axis(data, IBUS_CH_LEFT_X, deadband);
    converted.leftStick.y = IBUS_Axis(data, IBUS_CH_LEFT_Y, deadband);
    converted.sw[REMOTE_SW_A] =
        IBUS_ConvertSwitch(data->channel[IBUS_CH_SW_A]);
    converted.sw[REMOTE_SW_B] =
        IBUS_ConvertSwitch(data->channel[IBUS_CH_SW_B]);
    converted.sw[REMOTE_SW_C] =
        IBUS_ConvertSwitch(data->channel[IBUS_CH_SW_C]);
    converted.sw[REMOTE_SW_D] =
        IBUS_ConvertSwitch(data->channel[IBUS_CH_SW_D]);
    /* 两个旋钮一直在发，不像 DR16 的滚轮有"该字段无效"的情况。 */
    converted.knobA = IBUS_Axis(data, IBUS_CH_KNOB_A, deadband);
    converted.knobB = IBUS_Axis(data, IBUS_CH_KNOB_B, deadband);

    *remote = converted;
}
