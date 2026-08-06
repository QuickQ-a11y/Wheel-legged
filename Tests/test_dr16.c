#include "device_dr16.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-6f

static void testWriteInt16(uint8_t *data, int16_t value)
{
    uint16_t unsignedValue = (uint16_t)value;

    data[0] = (uint8_t)(unsignedValue & 0x00FFU);
    data[1] = (uint8_t)(unsignedValue >> 8U);
}

static void testBuildFrame(const uint16_t channels[4],
                            dr16_switch_t rightSwitch,
                            dr16_switch_t leftSwitch,
                            uint16_t dial,
                            uint16_t keyBits,
                            uint8_t frame[DR16_FRAME_LEN])
{
    memset(frame, 0, DR16_FRAME_LEN);
    frame[0] = (uint8_t)channels[0];
    frame[1] = (uint8_t)((channels[0] >> 8U) |
                         (channels[1] << 3U));
    frame[2] = (uint8_t)((channels[1] >> 5U) |
                         (channels[2] << 6U));
    frame[3] = (uint8_t)(channels[2] >> 2U);
    frame[4] = (uint8_t)((channels[2] >> 10U) |
                         (channels[3] << 1U));
    frame[5] = (uint8_t)((channels[3] >> 7U) |
                         ((uint8_t)rightSwitch << 4U) |
                         ((uint8_t)leftSwitch << 6U));
    frame[14] = (uint8_t)keyBits;
    frame[15] = (uint8_t)(keyBits >> 8U);
    frame[16] = (uint8_t)dial;
    frame[17] = (uint8_t)(dial >> 8U);
}

static void testCenteredFrame(void)
{
    uint16_t channels[4] = {
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
    };
    uint8_t frame[DR16_FRAME_LEN];
    dr16_data_t data;

    testBuildFrame(channels,
                    DR16_SWITCH_UP,
                    DR16_SWITCH_MID,
                    DR16_CH_MID,
                    0U,
                    frame);
    assert(DR16_ParseFrame(frame, &data) == 1U);
    assert(data.rightX == 0);
    assert(data.rightY == 0);
    assert(data.leftX == 0);
    assert(data.leftY == 0);
    assert(data.dial == 0);
    assert(data.dialValid == 1U);
    assert(data.rightSwitch == DR16_SWITCH_UP);
    assert(data.leftSwitch == DR16_SWITCH_MID);
    assert(data.keyBits == 0U);
}

static void testExtremesMouseAndKeys(void)
{
    static const dr16_key_t keys[16] = {
        DR16_KEY_W,
        DR16_KEY_S,
        DR16_KEY_D,
        DR16_KEY_A,
        DR16_KEY_SHIFT,
        DR16_KEY_CTRL,
        DR16_KEY_Q,
        DR16_KEY_E,
        DR16_KEY_R,
        DR16_KEY_F,
        DR16_KEY_G,
        DR16_KEY_Z,
        DR16_KEY_X,
        DR16_KEY_C,
        DR16_KEY_V,
        DR16_KEY_B,
    };
    uint16_t channels[4] = {
        DR16_CH_MIN,
        DR16_CH_MAX,
        DR16_CH_MIN,
        DR16_CH_MAX,
    };
    uint8_t frame[DR16_FRAME_LEN];
    dr16_data_t data;
    uint32_t index;

    testBuildFrame(channels,
                    DR16_SWITCH_DOWN,
                    DR16_SWITCH_UP,
                    DR16_CH_MAX,
                    0xFFFFU,
                    frame);
    testWriteInt16(&frame[6], INT16_MIN);
    testWriteInt16(&frame[8], INT16_MAX);
    testWriteInt16(&frame[10], -1234);
    frame[12] = 1U;
    frame[13] = 1U;

    assert(DR16_ParseFrame(frame, &data) == 1U);
    assert(data.rightX == -660);
    assert(data.rightY == 660);
    assert(data.leftX == -660);
    assert(data.leftY == 660);
    assert(data.dial == 660);
    assert(data.mouse.x == INT16_MIN);
    assert(data.mouse.y == INT16_MAX);
    assert(data.mouse.z == -1234);
    assert(data.mouse.leftPressed == 1U);
    assert(data.mouse.rightPressed == 1U);

    for (index = 0U; index < 16U; index++)
    {
        assert((uint16_t)keys[index] == (uint16_t)(1U << index));
        assert((data.keyBits & (uint16_t)keys[index]) != 0U);
    }
}

static void testInvalidMainFields(void)
{
    uint16_t channels[4] = {
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
    };
    uint8_t frame[DR16_FRAME_LEN];
    dr16_data_t data = {
        .rightX = 123,
    };

    channels[0] = DR16_CH_MIN - 1U;
    testBuildFrame(channels,
                    DR16_SWITCH_UP,
                    DR16_SWITCH_MID,
                    DR16_CH_MID,
                    0U,
                    frame);
    assert(DR16_ParseFrame(frame, &data) == 0U);
    assert(data.rightX == 123);

    channels[0] = DR16_CH_MID;
    testBuildFrame(channels,
                    DR16_SWITCH_UNKNOWN,
                    DR16_SWITCH_MID,
                    DR16_CH_MID,
                    0U,
                    frame);
    assert(DR16_ParseFrame(frame, &data) == 0U);
}

static void testReservedDialCompatibility(void)
{
    uint16_t channels[4] = {
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
        DR16_CH_MID,
    };
    uint8_t frame[DR16_FRAME_LEN];
    dr16_data_t data;

    testBuildFrame(channels,
                    DR16_SWITCH_MID,
                    DR16_SWITCH_MID,
                    0U,
                    DR16_KEY_Q | DR16_KEY_E,
                    frame);
    assert(DR16_ParseFrame(frame, &data) == 1U);
    assert(data.dial == 0);
    assert(data.dialValid == 0U);
    assert((data.keyBits & (uint16_t)DR16_KEY_Q) != 0U);
    assert((data.keyBits & (uint16_t)DR16_KEY_E) != 0U);
    assert((data.keyBits & (uint16_t)DR16_KEY_W) == 0U);
}

static void testAxisNormalization(void)
{
    assert(DR16_NormalizeAxis(0, 10) == 0.0f);
    assert(DR16_NormalizeAxis(10, 10) == 0.0f);
    assert(DR16_NormalizeAxis(-10, 10) == 0.0f);
    assert(fabsf(DR16_NormalizeAxis(11, 10) - (1.0f / 650.0f)) <
           TEST_TOLERANCE);
    assert(fabsf(DR16_NormalizeAxis(-11, 10) + (1.0f / 650.0f)) <
           TEST_TOLERANCE);
    assert(DR16_NormalizeAxis(660, 10) == 1.0f);
    assert(DR16_NormalizeAxis(-660, 10) == -1.0f);
    assert(DR16_NormalizeAxis(800, 10) == 1.0f);
    assert(DR16_NormalizeAxis(-800, 10) == -1.0f);
    assert(DR16_NormalizeAxis(335, -10) == 0.5f);
    assert(DR16_NormalizeAxis(660, 660) == 0.0f);
}

static void testGenericInputMapping(void)
{
    dr16_data_t data = {
        .rightX = -660,
        .rightY = -335,
        .leftX = 335,
        .leftY = 660,
        .dial = 500,
        .rightSwitch = DR16_SWITCH_DOWN,
        .leftSwitch = DR16_SWITCH_UP,
        .dialValid = 1U,
    };
    Remote_t remote = {0};

    DR16_MakeRemote(&data, 10, 400, &remote);
    assert(remote.leftStick.x == 0.5f);
    assert(remote.leftStick.y == 1.0f);
    assert(remote.rightStick.x == -1.0f);
    assert(remote.rightStick.y == -0.5f);
    assert(fabsf(remote.dial - (490.0f / 650.0f)) < TEST_TOLERANCE);
    assert(remote.dialValid == 1U);
    assert(remote.leftSwitch == REMOTE_SWITCH_UP);
    assert(remote.rightSwitch == REMOTE_SWITCH_DOWN);
    assert(remote.online == 0U);
    assert(remote.modeRequest == REMOTE_MODE_FOLLOW);
    assert(remote.legRequest == REMOTE_LEG_SHORT);

    data.rightSwitch = DR16_SWITCH_UP;
    data.leftSwitch = DR16_SWITCH_MID;
    data.dial = 400;
    DR16_MakeRemote(&data, 10, 400, &remote);
    assert(remote.rightSwitch == REMOTE_SWITCH_UP);
    assert(remote.leftSwitch == REMOTE_SWITCH_MID);
    assert(fabsf(remote.dial - 0.6f) < TEST_TOLERANCE);
    assert(remote.modeRequest == REMOTE_MODE_BENCH);
    assert(remote.legRequest == REMOTE_LEG_MIDDLE);

    data.leftSwitch = DR16_SWITCH_DOWN;
    data.dial = -400;
    DR16_MakeRemote(&data, 10, 400, &remote);
    assert(remote.leftSwitch == REMOTE_SWITCH_DOWN);
    assert(remote.modeRequest == REMOTE_MODE_SELF_SAVE);
    assert(remote.legRequest == REMOTE_LEG_MIDDLE);

    data.dial = -401;
    DR16_MakeRemote(&data, 10, 400, &remote);
    assert(remote.legRequest == REMOTE_LEG_LONG);

    data.dialValid = 0U;
    DR16_MakeRemote(&data, 10, 400, &remote);
    assert(remote.dial == 0.0f);
    assert(remote.dialValid == 0U);
    assert(remote.legRequest == REMOTE_LEG_KEEP);
}

int main(void)
{
    testCenteredFrame();
    testExtremesMouseAndKeys();
    testInvalidMainFields();
    testReservedDialCompatibility();
    testAxisNormalization();
    testGenericInputMapping();
    return 0;
}
