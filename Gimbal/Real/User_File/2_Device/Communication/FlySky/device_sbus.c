#include "device_sbus.h"

/* 一路通道占的位数，以及取满这 11 位的掩码。 */
#define SBUS_CH_BITS 11U
#define SBUS_CH_MASK 0x07FFU

/* 通道位流从帧的第 1 个字节开始（第 0 个是帧头）。 */
#define SBUS_PAYLOAD_OFFSET 1U

uint8_t SBUS_ParseFrame(const uint8_t frame[SBUS_FRAME_LEN],
                        sbus_data_t *data)
{
    sbus_data_t parsed;
    uint32_t index;

    if (frame[0] != SBUS_HEADER)
    {
        return 0U;
    }

    /*
     * ⚠ 帧尾【故意不校验】。标准说 frame[24] 是 0x00，但带 SBUS2 遥测的接收机
     * 会在这里填 0x04 / 0x14 / 0x24 / 0x34 表示遥测时隙。严格判 0x00 会把这些
     * 接收机的合法帧全部拒掉，现象是"波特率对、rxEventCount 正常涨，却一帧都
     * 解不出来"——和之前 i-BUS 那次踩的坑长得一模一样，很难往这上面想。
     *
     * ⚠⚠ 于是这个后端能依靠的校验只剩三样：task_remote 先查的帧长 25、
     * 这里的帧头 0x0F、以及 UART 偶校验。S.BUS 没有校验和，这是它相对 i-BUS
     * 的实质退步。补偿手段是下面解出来的 failsafe 标志，见 task_remote.c。
     */

    /*
     * 16 路通道每路 11 bit、LSB 在前，跨字节边界连续打包在 frame[1..22]。
     * 用位偏移循环而不是手写 16 行移位表达式：那 16 行是纯抄写活，错一位
     * 很难看出来，而且改通道数就要重抄一遍。
     *
     * 末路 index=15 时 bitPos=165、bytePos=21，要读 frame[21..23]。
     * frame[23] 是标志字节，在 25 字节帧内、不越界；它落在移位后第 11 位往上，
     * 被 SBUS_CH_MASK 掩掉，不影响结果。
     */
    for (index = 0U; index < SBUS_CH_COUNT; index++)
    {
        uint32_t bitPos = index * SBUS_CH_BITS;
        uint32_t bytePos = SBUS_PAYLOAD_OFFSET + (bitPos / 8U);
        uint32_t shift = bitPos % 8U;
        uint32_t window = (uint32_t)frame[bytePos] |
                          ((uint32_t)frame[bytePos + 1U] << 8U) |
                          ((uint32_t)frame[bytePos + 2U] << 16U);

        parsed.channel[index] = (uint16_t)((window >> shift) & SBUS_CH_MASK);
    }

    parsed.frameLost =
        ((frame[23] & SBUS_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    parsed.failsafe =
        ((frame[23] & SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;

    *data = parsed;
    return 1U;
}

float SBUS_NormalizeAxis(int16_t axis, int16_t deadband)
{
    int32_t limitedAxis = axis;
    int32_t positiveDeadband = deadband;
    int32_t magnitude;

    if (limitedAxis > SBUS_AXIS_MAX)
    {
        limitedAxis = SBUS_AXIS_MAX;
    }
    else if (limitedAxis < -SBUS_AXIS_MAX)
    {
        limitedAxis = -SBUS_AXIS_MAX;
    }

    if (positiveDeadband < 0)
    {
        positiveDeadband = -positiveDeadband;
    }
    if (positiveDeadband >= SBUS_AXIS_MAX)
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
        return -(float)magnitude / (float)(SBUS_AXIS_MAX - positiveDeadband);
    }

    return (float)magnitude / (float)(SBUS_AXIS_MAX - positiveDeadband);
}

Remote_Switch_t SBUS_ConvertSwitch(uint16_t channel)
{
    /*
     * ⚠ 拨杆是【上小下大】：UP=172、MID=992、DOWN=1811，和"值大 = 位置高"
     * 的直觉相反。这是发射机的行为，与 i-BUS 时完全一致。
     * REMOTE_SWITCH_UP 的语义必须和另外两个后端一致 —— 表示拨杆物理朝上。
     */
    if (channel <= SBUS_SW_LOW_MAX)
    {
        return REMOTE_SWITCH_UP;
    }
    if (channel >= SBUS_SW_HIGH_MIN)
    {
        return REMOTE_SWITCH_DOWN;
    }

    return REMOTE_SWITCH_MID;
}

/** @brief 把单个通道去中值后交给归一化。 */
static float SBUS_Axis(const sbus_data_t *data,
                       uint32_t channel,
                       int16_t deadband)
{
    return SBUS_NormalizeAxis(
        (int16_t)((int32_t)data->channel[channel] - (int32_t)SBUS_CH_MID),
        deadband);
}

void SBUS_MakeRemote(const sbus_data_t *data,
                     int16_t deadband,
                     Remote_t *remote)
{
    Remote_t converted = {0};

    converted.rightStick.x = SBUS_Axis(data, SBUS_CH_RIGHT_X, deadband);
    converted.rightStick.y = SBUS_Axis(data, SBUS_CH_RIGHT_Y, deadband);
    converted.leftStick.x = SBUS_Axis(data, SBUS_CH_LEFT_X, deadband);
    converted.leftStick.y = SBUS_Axis(data, SBUS_CH_LEFT_Y, deadband);
    converted.sw[REMOTE_SW_A] = SBUS_ConvertSwitch(data->channel[SBUS_CH_SW_A]);
    converted.sw[REMOTE_SW_B] = SBUS_ConvertSwitch(data->channel[SBUS_CH_SW_B]);
    converted.sw[REMOTE_SW_C] = SBUS_ConvertSwitch(data->channel[SBUS_CH_SW_C]);
    converted.sw[REMOTE_SW_D] = SBUS_ConvertSwitch(data->channel[SBUS_CH_SW_D]);
    converted.knobA = SBUS_Axis(data, SBUS_CH_KNOB_A, deadband);
    converted.knobB = SBUS_Axis(data, SBUS_CH_KNOB_B, deadband);

    *remote = converted;
}
