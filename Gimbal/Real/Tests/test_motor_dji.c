/*
 * 发射机构 DJI（M2006 + C610）设备层单测。
 *
 * device_motor_dji.c 要 stm32h7xx_hal.h / fdcan.h（桩，在 Tests/stub）和真的 task_can.h，
 * 符号由本文件自己定义。
 *
 * 编译（在 Gimbal/Real/ 下执行；Tests/stub 必须排在最前）：
 *   gcc -Wall -Wextra -ITests/stub -IUser_File/5_Task \
 *       -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Motor/Motor_DJI \
 *       -o t Tests/test_motor_dji.c \
 *       User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c -lm
 *
 * ⚠ Limit.h 是纯 inline 头文件，【没有 Limit.c】，别往命令里加。
 */
#include "device_motor_dji.h"

#include <assert.h>
#include <math.h>
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
    /* 拨盘装在底盘上，只能走两板之间的 FDCAN3。 */
    assert(Motor_DJI_BusOf(MOTOR_DJI_TRIGGER) == APP_CAN_BUS_FDCAN3);
}

/** @brief 0x200 帧槽位：摩擦轮 ID 1/2 落在 0/2、拨盘 ID 4 落在 6，互不重叠且不越界。 */
static void testTxSlots(void)
{
    const uint32_t ids[] = {
        APP_DJI_FRIC_1_RX_ID, APP_DJI_FRIC_2_RX_ID, APP_DJI_TRIGGER_RX_ID,
    };
    uint32_t i;
    uint32_t j;

    assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_1_RX_ID) == 0U);
    assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_2_RX_ID) == 2U);
    assert(APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) == 6U);

    for (i = 0U; i < 3U; i++)
    {
        assert(APP_DJI_TX_SLOT(ids[i]) + 1U < APP_DJI_TX_LEN);
        for (j = i + 1U; j < 3U; j++)
        {
            assert(APP_DJI_TX_SLOT(ids[i]) != APP_DJI_TX_SLOT(ids[j]));
        }
    }
}

/**
 * @brief 拨盘（C610）反馈解析。
 *
 * ⚠⚠ C610 的错误码在 data[7]，data[6] 是空字节——C620（摩擦轮）正好相反（data[6] 温度、
 * data[7] 空）。拨盘照抄 C620 的解法会让堵转、三相未接这些诊断永远读成 0，
 * 而拨盘卡弹恰恰就是靠 MOTOR_DJI_ERR_STALL 发现的。这条断言就是为了拦它。
 */
static void testFeedbackParse(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t state;

    stubTick = 1000U;
    buildFeedback(data, 4096U, -1234, 567, MOTOR_DJI_ERR_STALL);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);

    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &state);
    assert(state.encoderRaw == 4096U);
    assert(state.speedRpm == -1234);
    assert(state.currentRaw == 567);
    assert(state.errorCode == MOTOR_DJI_ERR_STALL);
    assert(state.isOnline == 1U);
    assert(state.lastUpdateTick == 1000U);

    /* 上一帧编码器值要跟着滚动，供回绕处理。 */
    buildFeedback(data, 4200U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &state);
    assert(state.lastEncoderRaw == 4096U);
    assert(state.encoderRaw == 4200U);
    assert(state.errorCode == MOTOR_DJI_ERR_NONE);
}

/**
 * @brief 摩擦轮（M3508 + C620）反馈解析：data[6] 是电机温度，data[7] 是空字节、不是错误码。
 *
 * 拦"摩擦轮也按 C610 解"：温度读不到；而 data[7] 万一不是 0，会被当成错误码、
 * 把好好的摩擦轮判成电调故障停掉。C620 手册：DATA[6] Motor temperature、DATA[7] Null。
 */
static void testFricFeedbackIsC620(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t state;

    stubTick = 1100U;
    buildFeedback(data, 1234U, 5000, -321, 6U); /* data[7] 故意不是 0 */
    data[6] = 41U;                              /* 41 ℃ */
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_1_RX_ID, data) == 1U);

    Motor_DJI_GetState(MOTOR_DJI_FRIC_1, &state);
    assert(state.encoderRaw == 1234U);
    assert(state.speedRpm == 5000);
    assert(state.currentRaw == -321);
    assert(state.temperature == 41U);
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
 * 肉眼看不出来。给三个互不相同的电流值，逐槽位核对；没有电机的槽位 4~5 必须是零。
 */
static void testTwoBusFramesIndependent(void)
{
    Motor_DJI_SetSafe(0U);
    Motor_DJI_ClearCommands();
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 1111);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_2, 2222);
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, -4444);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(capturedFric.sent == 1U);
    assert(capturedTrigger.sent == 1U);

    /* FDCAN2 帧：两个摩擦轮在各自槽位，拨盘槽位必须是零。 */
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) == 1111);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_2_RX_ID) == 2222);
    assert(slotValue(capturedFric.data, 0x203U) == 0); /* ID 3 已经没有电机 */
    assert(slotValue(capturedFric.data, APP_DJI_TRIGGER_RX_ID) == 0);

    /* FDCAN3 帧：只有拨盘槽位非零，摩擦轮槽位必须是零。 */
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == -4444);
    assert(slotValue(capturedTrigger.data, APP_DJI_FRIC_1_RX_ID) == 0);
    assert(slotValue(capturedTrigger.data, APP_DJI_FRIC_2_RX_ID) == 0);
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
 * @brief 电流按【各自电调】的量程限幅：摩擦轮 C620 ±16384（±20 A），拨盘 C610 ±10000（±10 A）。
 *
 * 期望值是两份手册里的字面量，不拿代码里的常量算。拦两种错：摩擦轮也按 C610 夹到 10000
 * （C620 上只剩约 12 A，速度环永远到不了满量程）；拨盘也按 C620 放到 16384
 * （越界值在 C610 上行为未定义）。
 */
static void testCurrentLimit(void)
{
    Motor_DJI_SetSafe(0U);
    Motor_DJI_ClearCommands();
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_1, 32767);
    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_2, 12000); /* C620 量程内，不能被夹 */
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, -32768);

    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_1_RX_ID) == 16384);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_2_RX_ID) == 12000);
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == -10000);

    Motor_DJI_SetCommand(MOTOR_DJI_FRIC_2, -32768);
    Motor_DJI_SetCommand(MOTOR_DJI_TRIGGER, 10000); /* 正好等于量程不该被改动 */
    clearCaptured();
    Motor_DJI_UpdateTxFrames(1U, 1U);
    assert(slotValue(capturedFric.data, APP_DJI_FRIC_2_RX_ID) == -16384);
    assert(slotValue(capturedTrigger.data, APP_DJI_TRIGGER_RX_ID) == 10000);
}

/** @brief 在线判定与超时。 */
static void testOnlineTimeout(void)
{
    uint8_t data[APP_DJI_RX_LEN];

    buildFeedback(data, 1U, 1, 1, MOTOR_DJI_ERR_NONE);
    stubTick = 5000U;
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_2_RX_ID, data) == 1U);

    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_2, 5000U) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_2,
                              5000U + APP_DJI_TIMEOUT_TICKS) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_2,
                              5000U + APP_DJI_TIMEOUT_TICKS + 1U) == 0U);

    /* tick 回绕：时间戳在溢出前、now 在溢出后，无符号相减仍然正确。 */
    stubTick = 0xFFFFFFF0U;
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                    APP_DJI_FRIC_2_RX_ID, data) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_2, 0xFFFFFFF0U + 10U) == 1U);
    assert(Motor_DJI_IsOnline(MOTOR_DJI_FRIC_2,
                              0xFFFFFFF0U + APP_DJI_TIMEOUT_TICKS + 1U) == 0U);
}


/**
 * @brief 编码器回绕累加：跨过 0/8191 边界时圈数要增减，不能把回绕当成大跳变。
 *
 * ⚠ 这条是拨盘位置环的地基。没有累加的话 encoderRaw 一到边界就跳 8191 个计数，
 * 位置环会当成一个巨大误差猛冲一下。
 *
 * ⚠ 本用例必须在 main() 里【第一个】跑：它要验证"首帧不产生假 delta"，
 * 而设备层的状态是静态的，一旦别的用例先把这台电机带上线就验不了了。
 */
static void testEncoderWrap(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t s;

    stubTick = 100U;

    /* 首帧：lastEncoderRaw 还是 0，不能拿它算 delta，否则凭空多出一整段。 */
    buildFeedback(data, 8000U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &s);
    assert(s.totalRound == 0);
    assert(s.totalEncoder == 8000);

    /* 正向跨过回绕点：8000 -> 100，delta 为大负数，应记 +1 圈。 */
    buildFeedback(data, 100U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &s);
    assert(s.totalRound == 1);
    assert(s.totalEncoder == APP_DJI_ENCODER_PER_ROUND + 100);

    /* 反向跨回来：100 -> 8000，delta 为大正数，应记 -1 圈。 */
    buildFeedback(data, 8000U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &s);
    assert(s.totalRound == 0);
    assert(s.totalEncoder == 8000);

    /* 普通小步进不能误判成回绕。 */
    buildFeedback(data, 8100U, 0, 0, MOTOR_DJI_ERR_NONE);
    assert(Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                    APP_DJI_TRIGGER_RX_ID, data) == 1U);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &s);
    assert(s.totalRound == 0);
    assert(s.totalEncoder == 8100);
}

/**
 * @brief 摩擦轮【没有减速箱】（已物理拆掉），angle 与转子 1:1。
 *
 * ⚠ 这条和下面那条拨盘的用例必须成对看：减速比是【每台电机各不相同】的，
 * 拿一个全局常量去除，两条里必然有一条是错的。
 */
static void testFricAngleHasNoGearbox(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t before;
    motor_dji_state_t after;
    float expect;

    stubTick = 300U;

    buildFeedback(data, 0U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                   APP_DJI_FRIC_1_RX_ID, data);
    Motor_DJI_GetState(MOTOR_DJI_FRIC_1, &before);

    buildFeedback(data, 3000U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                   APP_DJI_FRIC_1_RX_ID, data);
    buildFeedback(data, 6000U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                   APP_DJI_FRIC_1_RX_ID, data);
    buildFeedback(data, 0U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN2,
                                   APP_DJI_FRIC_1_RX_ID, data);
    Motor_DJI_GetState(MOTOR_DJI_FRIC_1, &after);

    assert(after.totalEncoder - before.totalEncoder == APP_DJI_ENCODER_PER_ROUND);

    /* 转子一圈 = 轮子一圈 = 2π，不是 2π/36。 */
    expect = 2.0f * APP_PI_F / APP_DJI_FRIC_GEAR_RATIO;
    assert(fabsf((after.angle - before.angle) - expect) < 1e-4f);
}

/**
 * @brief 拨盘的减速箱【还在】：转子转满一圈只等于输出轴 2π/36。
 *
 * ⚠ 漏掉减速比的话这个断言会差 36 倍。断言的是【增量】，不依赖此前的历史。
 */
static void testAngleIsOutputShaft(void)
{
    uint8_t data[APP_DJI_RX_LEN];
    motor_dji_state_t before;
    motor_dji_state_t after;
    float expect;

    stubTick = 200U;

    buildFeedback(data, 0U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                   APP_DJI_TRIGGER_RX_ID, data);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &before);

    /*
     * 转子正向走满一整圈回到 0。中间必须踩够点：单帧跨度超过半圈会被判成
     * 反向回绕，而 0 -> 4000 -> 0 只是出去又回来、根本没绕圈。
     */
    buildFeedback(data, 3000U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                   APP_DJI_TRIGGER_RX_ID, data);
    buildFeedback(data, 6000U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                   APP_DJI_TRIGGER_RX_ID, data);
    buildFeedback(data, 0U, 0, 0, MOTOR_DJI_ERR_NONE);
    (void)Motor_DJI_UpdateFeedback(APP_CAN_BUS_FDCAN3,
                                   APP_DJI_TRIGGER_RX_ID, data);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &after);

    assert(after.totalRound - before.totalRound == 1);
    assert(after.totalEncoder - before.totalEncoder == APP_DJI_ENCODER_PER_ROUND);

    expect = 2.0f * APP_PI_F / APP_DJI_TRIGGER_GEAR_RATIO;
    assert(fabsf((after.angle - before.angle) - expect) < 1e-4f);
}

int main(void)
{
    /* ⚠ 回绕用例要验"首帧"，必须排在所有会带电机上线的用例之前。 */
    testEncoderWrap();
    testAngleIsOutputShaft();
    testFricAngleHasNoGearbox();

    testBusMapping();
    testTxSlots();
    testFeedbackParse();
    testFricFeedbackIsC620();
    testFeedbackBusFiltering();
    testTwoBusFramesIndependent();
    testBusEnableGating();
    testSafeGate();
    testCurrentLimit();
    testOnlineTimeout();
    printf("test_motor_dji: 13 组用例全过\n");
    return 0;
}
