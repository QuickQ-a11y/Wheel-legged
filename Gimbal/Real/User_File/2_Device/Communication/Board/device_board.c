#include "device_board.h"

#include "app_config.h"
#include "Limit.h"
#include "task_can.h"

#include "fdcan.h"

static uint8_t boardTxSequence;

/**
 * @brief 把归一化量按定点比例编码成 int16。
 *
 * 限幅是编码本身需要的：超出 int16 量程会回绕成反号的大值。
 */
static int16_t Board_Encode(float value)
{
    float scaled = value * APP_BOARD_SCALE;

    return (int16_t)Algorithm_LimitRange(scaled, -32768.0f, 32767.0f);
}

/** @brief 按低字节在前写入一个 int16。 */
static void Board_WriteInt16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value);
    data[1] = (uint8_t)((uint16_t)value >> 8U);
}

void Board_UpdateTxFrames(const Remote_t *remote,
                          float yaw_rel,
                          uint8_t autoaim_flag)
{
    uint8_t stick[APP_BOARD_FRAME_LEN];
    uint8_t state[APP_BOARD_FRAME_LEN] = {0};
    uint8_t flags = 0U;
    uint8_t switches = 0U;
    uint32_t index;

    Board_WriteInt16(&stick[0], Board_Encode(remote->leftStick.x));
    Board_WriteInt16(&stick[2], Board_Encode(remote->leftStick.y));
    Board_WriteInt16(&stick[4], Board_Encode(remote->rightStick.x));
    Board_WriteInt16(&stick[6], Board_Encode(remote->rightStick.y));

    /* 四个拨杆压进一个字节：Remote_Switch_t 只有 0..3，2 bit 正好装下。 */
    for (index = 0U; index < (uint32_t)REMOTE_SW_COUNT; index++)
    {
        switches |= (uint8_t)(((uint8_t)remote->sw[index] & APP_BOARD_SW_MASK)
                              << (index * APP_BOARD_SW_BITS));
    }

    if (remote->online != 0U)
    {
        flags |= APP_BOARD_FLAG_REMOTE_ONLINE;
    }
    if (autoaim_flag != 0U)
    {
        flags |= APP_BOARD_FLAG_AUTOAIM;
    }

    state[0] = switches;
    state[1] = flags;
    Board_WriteInt16(&state[2], Board_Encode(remote->knobA));
    state[4] = boardTxSequence;
    /* state[5] 预留，靠上面的 {0} 保持为零。 */
    Board_WriteInt16(&state[6], Board_Encode(yaw_rel));

    boardTxSequence++;

    CAN_Task_UpdateTxFrame(&hfdcan3, APP_BOARD_STICK_ID, stick, APP_BOARD_FRAME_LEN);
    CAN_Task_UpdateTxFrame(&hfdcan3, APP_BOARD_STATE_ID, state, APP_BOARD_FRAME_LEN);
}
