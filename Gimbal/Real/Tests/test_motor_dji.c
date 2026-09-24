/*
 * 发射机构 DJI（M2006 + C610）设备层单测。
 *
 * device_motor_dji.c 要 stm32h7xx_hal.h / fdcan.h / task_can.h 三个头文件，
 * 复用 test_board_protocol 那三个桩（配方见 Gimbal/CLAUDE.md 的 Host tests 一节），
 * 符号由本文件自己定义。
 *
 * 编译（在 Gimbal/Real/ 下执行）：
 *   gcc -Wall -Wextra -I<stub> -IUser_File/1_Middleware/0_Common \
 *       -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Motor/Motor_DJI \
 *       -o t Tests/test_motor_dji.c \
 *       User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c \
 *       User_File/1_Middleware/2_Algorithm/Limit.c -lm
 */
#include "device_motor_dji.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 桩：捕获发出去的帧 ---- */
typedef struct { int dummy; } FDCAN_HandleTypeDef;
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

typedef struct
{
    uint8_t sent;
    uint8_t data[APP_DJI_TX_LEN];
} captured_frame_t;

static captured_frame_t capturedFric;    /* FDCAN2 */
static captured_frame_t capturedTrigger; /* FDCAN3 */

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    assert(identifier == APP_DJI_TX_ID);
    assert(length == APP_DJI_TX_LEN);

    if (handle == &hfdcan2)
    {
        capturedFric.sent = 1U;
        memcpy(capturedFric.data, data, length);
    }
    else if (handle == &hfdcan3)
    {
        capturedTrigger.sent = 1U;
        memcpy(capturedTrigger.data, data, length);
    }
    else
    {
        assert(0 && "shooter frame on unexpected bus");
    }
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

static void clearCaptured(void)
{
    memset(&capturedFric, 0, sizeof(capturedFric));
    memset(&capturedTrigger, 0, sizeof(capturedTrigger));
}

static int16_t slotValue(const uint8_t data[APP_DJI_TX_LEN], uint32_t rxId)
{
    uint8_t slot = APP_DJI_TX_SLOT(rxId);

    return (int16_t)(((uint16_t)data[slot] << 8U) | (uint16_t)data[slot + 1U]);
}

/** @brief 造一帧 C610 反馈。 */
static void buildFeedback(uint8_t data[APP_DJI_RX_LEN], uint16_t encoder,
                          int16_t rpm, int16_t current, uint8_t errorCode)
{
    memset(data, 0, APP_DJI_RX_LEN);
    data[0] = (uint8_t)(encoder >> 8U);
    data[1] = (uint8_t)encoder;
    data[2] = (uint8_t)((uint16_t)rpm >> 8U);
    data[3] = (uint8_t)(uint16_t)rpm;
    data[4] = (uint8_t)((uint16_t)current >> 8U);
    data[5] = (uint8_t)(uint16_t)current;
    /*
     * ⚠ data[6] 保持 0：C610 这一字节是【空】的。把它填成温度是 C620 的格式，
     * 本用例特意留空，好让"照抄 C620 解法"的实现在下面的 errorCode 断言上翻车。
     */
    data[7] = errorCode;
}

/** @brief 总线归属只在设备层定义一次，这里钉死它。 */
static void testBusMapping(void)
{
    assert(Motor_DJI_BusOf(MOTOR_DJI_FRIC_1) == APP_CAN_BUS_FDCAN2);
    assert(Motor_DJI_BusOf(MOTOR_DJI_FRIC_2) == APP_CAN_BUS_FDCAN2);
    assert(Motor_DJI_BusOf(MOTOR_DJI_FRIC_3) == APP_CAN_BUS_FDCAN2);
    /* 拨盘装在底盘上，只能走两板之间的 FDCAN3。 */
    assert(Motor_DJI_BusOf(MOTOR_DJI_TRIGGER) == APP_CAN_BUS_FDCAN3);
}

/** @brief 0x200 帧槽位：ID 1~4 落在 0/2/4/6，互不重叠且不越界。 */
static void testTxSlots(void)
{
    const uint32_t ids[] = {
        APP_DJI_FRIC_1_RX_ID, APP_DJI_FRIC_2_RX_ID,
        APP_DJI_FRIC_3_RX_ID, APP_DJI_TRIGGER_RX_ID,
    };
    uint32_t i;
    uint32_t j;

    assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_1_RX_ID) == 0U);
    assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_2_RX_ID) == 2U);
    assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_3_RX_ID) == 4U);
    assert(APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) == 6U);

    for (i = 0U; i < 4U; i++)
    {
        assert(APP_DJI_TX_SLOT(ids[i]) + 1U < APP_DJI_TX_LEN);
        for (j = i + 1U; j < 4U; j++)
        {
            assert(APP_DJI_TX_SLOT(ids[i]) != APP_DJI_TX_SLOT(ids[j]));
        }
    }
}

/**
 * @brief C610 反馈解析。
 *
 * ⚠⚠ 错误码在 data[7]，data[6] 是空字节——C620 正好相反（data[6] 温度、
 * data[7] 空）。照抄 C620 的解法会让堵转、三相未接这些诊断永远读成 0，
 * 而拨盘卡弹恰恰就是靠 MOTOR_DJI_ERR_STALL 发现的。这条断言就是为了拦它。
 */
static void testFeedbackParse(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t state;

    stubTick = 1000U;
    buildFeedback(data, 4096U, -1234, 567, MOTOR_DJI_ERR_STALL);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_2_RX_ID, data) == 1U);

    Motor_DJI_GetState(MOTOR_DJI_FRIC_2, &state);
    assert(state.encoderRaw == 4096U);
    assert(state.speedRpm == -1234);
    assert(state.currentRaw == 567);
    assert(state.errorCode == MOTOR_DJI_ERR_STALL);
    assert(state.isOnline == 1U);
    assert(state.feedbackCount == 1U);
    assert(state.lastUpdateTick == 1000U);

    /* 上一帧编码器值要跟着滚动，供将来做回绕处理。 */
    buildFeedback(data, 8000U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_2_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_FRIC_2, &state);
    assert(state.lastEncoderRaw == 4096U);
    assert(state.encoderRaw == 8000U);
    assert(state.errorCode == MOTOR_DJI_ERR_NONE);
}

/**
 * @brief 反馈必须同时比对总线和 ID。
 *
 * 0x201~0x203 在底盘的 FDCAN1 上是轮电调的反馈 ID。只比 ID 的实现，
 * 在总线接错或将来合并总线时会读到别人的电机——这条拦它。
 */
static void testFeedbackBusFiltering(void)
{
    uint8_t data[APP_DJI_RX_LEN];

    buildFeedback(data, 1U, 1, 1, MOTOR_DJI_ERR_NONE);

    /* 摩擦轮的 ID 出现在别的总线上，不能认。 */
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                                    APP_DJI_FRIC_1_RX_ID, data) == 0U);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_FRIC_1_RX_ID, data) == 0U);
    /* 拨盘的 ID 出现在 FDCAN2 上，同样不能认。 */
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_TRIGGER_RX_ID, data) == 0U);
    /* 没人认领的 ID。 */
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2, 0x2FFU, data) == 0U);

    /* 对上才认。 */
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
}

/**
 * @brief 两条总线各打一帧，互不串。
 *
 * ⚠ 这是本文件最重要的一条。摩擦轮和拨盘在【不同总线】上共用同一个帧 ID 0x200，
 * 把拨盘的电流写进摩擦轮那帧（或反过来）会直接控错电机，而两帧长得一样、
 * 肉眼看不出来。给四个互不相同的电流值，逐槽位核对。
 */
static void testTwoBusFramesIndependent(void)
{
    Motor_DJI_SetSafe(0U);
    Motor_DJI_ClearCommands();
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 1111);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_2, 2222);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_3, 3333);
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, -4444);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(capturedFric.sent == 1U);
    assert(capturedTrigger.sent == 1U);

    /* FDCAN2 帧：三个摩擦轮在各自槽位，拨盘槽位必须是零。 */
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) == 1111);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_2_RX_ID) == 2222);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_3_RX_ID) == 3333);
    assert(slotValue(capturedFric.data, APP_DJI_TRIGGER_RX_ID) == 0);

    /* FDCAN3 帧：只有拨盘槽位非零，三个摩擦轮槽位必须是零。 */
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == -4444);
    assert(slotValue(capturedTrigger.data, APP_DJI_FRIC_1_RX_ID) == 0);
    assert(slotValue(capturedTrigger.data, APP_DJI_FRIC_2_RX_ID) == 0);
    assert(slotValue(capturedTrigger.data, APP_DJI_FRIC_3_RX_ID) == 0);
}

/**
 * @brief 使能为 0 的总线【一帧都不能发】。
 *
 * ⚠ 这不是可有可无的开关。某条总线上一个节点都没有时，发出去的帧没人应答 ACK，
 * 发送错误计数会爬到 BusOff、驱动 Stop/Start 再 BusOff，空转在错误恢复里。
 * 摩擦轮未装机时 FDCAN2 就是这种情况，分步装机全靠这个门。
 */
static void testBusEnableGating(void)
{
    Motor_DJI_SetSafe(0U);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 1000);
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, 1000);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(0U, 0U);
    assert(capturedFric.sent == 0U);
    assert(capturedTrigger.sent == 0U);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 0U);
    assert(capturedFric.sent == 1U);
    assert(capturedTrigger.sent == 0U);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(0U, 1U);
    assert(capturedFric.sent == 0U);
    assert(capturedTrigger.sent == 1U);
}

/** @brief 安全门置位时下发的电流恒零，但命令值本身保留。 */
static void testSafeGate(void)
{
    Motor_DJI_ClearCommands();
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 5000);
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, 5000);

    Motor_DJI_SetSafe(1U);
    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) == 0);
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == 0);

    /* 解除安全门后不需要重新 SetCommand，命令值还在。 */
    Motor_DJI_SetSafe(0U);
    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) == 5000);
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == 5000);
}

/**
 * @brief 电流按 ±APP_DJI_CURRENT_MAX 限幅。
 *
 * ⚠ C610 是 ±10000 对应 ±10 A，C620（底盘轮电机）是 ±16384 对应 ±20 A。
 * 不限幅的话越界值在 C610 上行为未定义。
 */
static void testCurrentLimit(void)
{
    Motor_DJI_SetSafe(0U);
    Motor_DJI_ClearCommands();
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 32767);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_2, -32768);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_3, APP_DJI_CURRENT_MAX);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) ==
           (int16_t)APP_DJI_CURRENT_MAX);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_2_RX_ID) ==
           (int16_t)(-APP_DJI_CURRENT_MAX));
    /* 正好等于量程不该被改动。 */
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_3_RX_ID) ==
           (int16_t)APP_DJI_CURRENT_MAX);
}

/** @brief 在线判定与超时。 */
static void testOnlineTimeout(void)
{
    uint8_t data[APP_DJI_RX_LEN];

    buildFeedback(data, 1U, 1, 1, MOTOR_DJI_ERR_NONE);
    stubTick = 5000U;
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_3_RX_ID, data) == 1U);

    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_3, 5000U) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_3,
                              5000U + APP_DJI_TIMEOUT_TICKS) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_3,
                              5000U + APP_DJI_TIMEOUT_TICKS + 1U) == 0U);

    /* tick 回绕：时间戳在溢出前、now 在溢出后，无符号相减仍然正确。 */
    stubTick = 0xFFFFFFF0U;
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_3_RX_ID, data) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_3, 0xFFFFFFF0U + 10U) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_3,
                              0xFFFFFFF0U + APP_DJI_TIMEOUT_TICKS + 1U) == 0U);
}

int main(void)
{
    testBusMapping();
    testTxSlots();
    testFeedbackParse();
    testFeedbackBusFiltering();
    testTwoBusFramesIndependent();
    testBusEnableGating();
    testSafeGate();
    testCurrentLimit();
    testOnlineTimeout();
    printf("test_motor_dji: 9 组用例全过\n");
    return 0;
}
