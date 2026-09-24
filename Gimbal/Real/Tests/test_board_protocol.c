/*
 * 板间通信协议单测：用云台侧的打包函数产出帧，再用底盘侧的解包函数还原，
 * 同时对着 app_config.h 里的协议表逐字节断言。
 *
 * 只做往返比较不够——两边同时改错同一个字段时往返仍然自洽，所以必须有
 * 对协议表的绝对断言。
 *
 * ⚠ 两个工程各有一个同名的 device_board.h 内容不同，所以两个 .c 必须
 * 【分两次 gcc 调用】各带自己的 -I 编译成 .o 再链接，一次编译会选错头文件。
 *
 * 编译（需要三个桩头文件，见工作区 CLAUDE.md 的 Host tests 一节）：
 *   gcc -c -Wall -Wextra -I<stub> -IUser_File/1_Middleware/0_Common \
 *       -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Communication/Board \
 *       -o gimbal_board.o User_File/2_Device/Communication/Board/device_board.c
 *   gcc -c -Wall -Wextra -I<stub> \
 *       -I../../Chassis/User_File/1_Middleware/0_Common \
 *       -I../../Chassis/User_File/2_Device/Communication/Board \
 *       -o chassis_board.o \
 *       ../../Chassis/User_File/2_Device/Communication/Board/device_board.c
 *   gcc -Wall -Wextra -I<stub> -IUser_File/1_Middleware/0_Common \
 *       -o t Tests/test_board_protocol.c gimbal_board.o chassis_board.o -lm
 */
#include "app_config.h"
#include "remote_input.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 云台侧：打包 */
void Board_UpdateTxFrames(const Remote_t *remote, float yaw_rel, uint8_t autoaim_flag);
/* 底盘侧：解包 + 在线判定 */
void Board_UpdateFeedback(uint32_t identifier, const uint8_t data[APP_BOARD_FRAME_LEN]);
uint8_t Board_IsOnline(uint32_t nowTick);
void Board_GetRemote(Remote_t *remote);
uint8_t Board_GetAutoaim(void);

/* ---- 桩：捕获云台发出的两帧 ---- */
typedef struct { int dummy; } FDCAN_HandleTypeDef;
FDCAN_HandleTypeDef hfdcan3;

static uint8_t capturedStick[APP_BOARD_FRAME_LEN];
static uint8_t capturedState[APP_BOARD_FRAME_LEN];

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    (void)handle;
    assert(length == APP_BOARD_FRAME_LEN);
    if (identifier == APP_BOARD_STICK_ID) { memcpy(capturedStick, data, length); }
    else if (identifier == APP_BOARD_STATE_ID) { memcpy(capturedState, data, length); }
    else { assert(0 && "unexpected board frame id"); }
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

#define TOL (1.5f / APP_BOARD_SCALE)   /* 定点量化误差上限 */
#define NEAR(a, b) (fabsf((a) - (b)) < TOL)

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void deliver(void)
{
    Board_UpdateFeedback(APP_BOARD_STICK_ID, capturedStick);
    Board_UpdateFeedback(APP_BOARD_STATE_ID, capturedState);
}

/** @brief 满量程往返，并对着协议表断言字节布局。 */
static void test_round_trip_and_layout(void)
{
    Remote_t tx = {0};
    Remote_t rx = {0};
    uint32_t index;

    tx.leftStick.x = 0.5f;
    tx.leftStick.y = -0.25f;
    tx.rightStick.x = 1.0f;
    tx.rightStick.y = -1.0f;
    /*
     * 四个拨杆给四个互不相同的位置，任意两路接反都会被抓到。
     * 打包后 data[0] 应当是 1 | (2<<2) | (3<<4) | (0<<6) = 57。
     */
    tx.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;      /* 1 */
    tx.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;    /* 2 */
    tx.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;     /* 3 */
    tx.sw[REMOTE_SW_D] = REMOTE_SWITCH_UNKNOWN; /* 0 */
    tx.knobA = 0.6f;
    tx.knobB = -0.9f;                           /* 不下发，用来确认它没混进去 */
    tx.online = 1U;

    Board_UpdateTxFrames(&tx, 1.5f, 1U);

    /* 协议表：摇杆帧四个 int16，低字节在前，比例 APP_BOARD_SCALE。 */
    assert(le16(&capturedStick[0]) == 5000);
    assert(le16(&capturedStick[2]) == -2500);
    assert(le16(&capturedStick[4]) == 10000);
    assert(le16(&capturedStick[6]) == -10000);

    /* 协议表：状态帧字节位置。 */
    assert(capturedState[0] == 57U);
    assert(capturedState[1] ==
           (APP_BOARD_FLAG_REMOTE_ONLINE | APP_BOARD_FLAG_AUTOAIM));
    assert(le16(&capturedState[2]) == 6000);
    assert(capturedState[5] == 0U);   /* 预留字节必须填零 */
    assert(le16(&capturedState[6]) == 15000);

    stubTick = 1000U;
    deliver();
    Board_GetRemote(&rx);

    assert(NEAR(rx.leftStick.x, tx.leftStick.x));
    assert(NEAR(rx.leftStick.y, tx.leftStick.y));
    assert(NEAR(rx.rightStick.x, tx.rightStick.x));
    assert(NEAR(rx.rightStick.y, tx.rightStick.y));
    for (index = 0U; index < (uint32_t)REMOTE_SW_COUNT; index++)
    {
        assert(rx.sw[index] == tx.sw[index]);
    }
    assert(NEAR(rx.knobA, tx.knobA));
    /* VrB 不下发，底盘侧必须保持 0，不能是发送方那个 -0.9。 */
    assert(rx.knobB == 0.0f);
    assert(rx.online == 1U);
    assert(Board_GetAutoaim() == 1U);
}

/**
 * @brief 四个拨杆的位域互不串扰。
 *
 * 一次只把一路拨到 DOWN，其余留 UNKNOWN，确认解出来只有那一路变。
 * 位移写错（比如都用下标 0）时这条会红，而全设成同一个值的往返测不出来。
 */
static void test_switch_slots_independent(void)
{
    uint32_t slot;

    for (slot = 0U; slot < (uint32_t)REMOTE_SW_COUNT; slot++)
    {
        Remote_t tx = {0};
        Remote_t rx = {0};
        uint32_t index;

        tx.sw[slot] = REMOTE_SWITCH_DOWN;
        Board_UpdateTxFrames(&tx, 0.0f, 0U);
        deliver();
        Board_GetRemote(&rx);

        for (index = 0U; index < (uint32_t)REMOTE_SW_COUNT; index++)
        {
            assert(rx.sw[index] ==
                   ((index == slot) ? REMOTE_SWITCH_DOWN : REMOTE_SWITCH_UNKNOWN));
        }
    }
}

/** @brief 标志位彼此独立，不能串。 */
static void test_flags_independent(void)
{
    Remote_t tx = {0};
    Remote_t rx = {0};

    tx.online = 0U;
    Board_UpdateTxFrames(&tx, 0.0f, 1U);
    deliver();
    Board_GetRemote(&rx);
    assert(rx.online == 0U && Board_GetAutoaim() == 1U);

    tx.online = 1U;
    Board_UpdateTxFrames(&tx, 0.0f, 0U);
    deliver();
    Board_GetRemote(&rx);
    assert(rx.online == 1U && Board_GetAutoaim() == 0U);
}

/** @brief 序号每发一轮递增，且能跨 255 回绕。 */
static void test_sequence_increments(void)
{
    Remote_t tx = {0};
    uint8_t first;
    int i;

    Board_UpdateTxFrames(&tx, 0.0f, 0U);
    first = capturedState[4];
    Board_UpdateTxFrames(&tx, 0.0f, 0U);
    assert(capturedState[4] == (uint8_t)(first + 1U));

    for (i = 0; i < 256; i++) { Board_UpdateTxFrames(&tx, 0.0f, 0U); }
    assert(capturedState[4] == (uint8_t)(first + 1U + 256U));
}

/** @brief yaw_rel 取满 ±pi 不溢出 int16。 */
static void test_yaw_range(void)
{
    Remote_t tx = {0};

    Board_UpdateTxFrames(&tx, 3.14159f, 0U);
    assert(le16(&capturedState[6]) == 31415);
    Board_UpdateTxFrames(&tx, -3.14159f, 0U);
    assert(le16(&capturedState[6]) == -31415);
}

/**
 * @brief 上电从未收到过板间帧时必须判离线。
 *
 * 这一条必须在任何 deliver() 之前跑：一旦别的用例把 received 置了 1，
 * 断言就会因为"时间戳太旧"而通过，根本测不到上电分支。
 */
static void test_boot_offline(void)
{
    /* tick 和时间戳都是 0，只比时间差的话 0-0=0 <= 超时，会误判成在线。 */
    stubTick = 0U;
    assert(Board_IsOnline(0U) == 0U);
}

/**
 * @brief 超时判定与 tick 无符号回绕。
 */
static void test_online_timeout(void)
{
    Remote_t tx = {0};

    /* 收到帧后在线，超时后离线。 */
    stubTick = 1000U;
    Board_UpdateTxFrames(&tx, 0.0f, 0U);
    deliver();
    assert(Board_IsOnline(1000U) == 1U);
    assert(Board_IsOnline(1000U + APP_BOARD_TIMEOUT_TICKS) == 1U);
    assert(Board_IsOnline(1000U + APP_BOARD_TIMEOUT_TICKS + 1U) == 0U);

    /* tick 回绕：时间戳在溢出前，now 在溢出后，无符号相减仍然正确。 */
    stubTick = 0xFFFFFFF0U;
    Board_UpdateTxFrames(&tx, 0.0f, 0U);
    deliver();
    assert(Board_IsOnline(0xFFFFFFF0U + 10U) == 1U);              /* 差 10，回绕后仍在线 */
    assert(Board_IsOnline(0xFFFFFFF0U + APP_BOARD_TIMEOUT_TICKS + 1U) == 0U);
}

int main(void)
{
    test_boot_offline();   /* 必须第一个跑，理由见函数注释 */
    test_round_trip_and_layout();
    test_switch_slots_independent();
    test_flags_independent();
    test_sequence_increments();
    test_yaw_range();
    test_online_timeout();
    printf("test_board_protocol: 7 组用例全过\n");
    return 0;
}
