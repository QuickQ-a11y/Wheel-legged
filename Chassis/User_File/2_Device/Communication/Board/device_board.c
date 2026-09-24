#include "device_board.h"

#include "stm32h7xx_hal.h"

board_state_t boardDebugState;

/** @brief 按低字节在前读出一个 int16。 */
static int16_t Board_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/** @brief 把定点值还原成归一化量。 */
static float Board_Decode(const uint8_t *data)
{
    return (float)Board_ReadInt16(data) / APP_BOARD_SCALE;
}

void Board_UpdateFeedback(uint32_t identifier,
                          const uint8_t data[APP_BOARD_FRAME_LEN])
{
    if (identifier == APP_BOARD_STICK_ID)
    {
        boardDebugState.remote.leftStick.x = Board_Decode(&data[0]);
        boardDebugState.remote.leftStick.y = Board_Decode(&data[2]);
        boardDebugState.remote.rightStick.x = Board_Decode(&data[4]);
        boardDebugState.remote.rightStick.y = Board_Decode(&data[6]);
        boardDebugState.stickFrameCount++;
    }
    else if (identifier == APP_BOARD_STATE_ID)
    {
        uint32_t index;

        /* 四个拨杆挤在 data[0] 里，每个 2 bit，下标即 REMOTE_SW_*。 */
        for (index = 0U; index < (uint32_t)REMOTE_SW_COUNT; index++)
        {
            boardDebugState.remote.sw[index] = (Remote_Switch_t)
                ((data[0] >> (index * APP_BOARD_SW_BITS)) & APP_BOARD_SW_MASK);
        }
        boardDebugState.remote.online =
            ((data[1] & APP_BOARD_FLAG_REMOTE_ONLINE) != 0U) ? 1U : 0U;
        boardDebugState.autoaim_flag =
            ((data[1] & APP_BOARD_FLAG_AUTOAIM) != 0U) ? 1U : 0U;
        boardDebugState.remote.knobA = Board_Decode(&data[2]);
        boardDebugState.sequence = data[4];
        boardDebugState.yaw_rel = Board_Decode(&data[6]);
        boardDebugState.stateFrameCount++;
    }
    else
    {
        return;
    }

    boardDebugState.lastUpdateTick = HAL_GetTick();
    boardDebugState.received = 1U;
}

uint8_t Board_IsOnline(uint32_t nowTick)
{
    if (boardDebugState.received == 0U)
    {
        return 0U;
    }

    return ((nowTick - boardDebugState.lastUpdateTick) <=
            APP_BOARD_TIMEOUT_TICKS) ? 1U : 0U;
}

void Board_GetRemote(Remote_t *remote)
{
    *remote = boardDebugState.remote;
}

float Board_GetYawRel(void)
{
    return boardDebugState.yaw_rel;
}

uint8_t Board_GetAutoaim(void)
{
    return boardDebugState.autoaim_flag;
}
