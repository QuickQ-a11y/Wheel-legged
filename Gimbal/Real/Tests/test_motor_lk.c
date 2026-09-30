/*
 * 翎控（LK-TECH）MG4005E-i10 拨盘设备层单测：0xA1 转矩闭环命令的收发编解码。
 *
 * 协议依据：LK-TECH 电机 CAN 协议说明 V2.3 §20（0xA1），
 * 本机手册见工作区 Material/MG40-50系电机用户手册.pdf。
 *
 * 编译（在 Gimbal/Real/ 下执行；桩头文件在 Tests/stub，必须排在最前）：
 *   gcc -Wall -Wextra -ITests/stub -IUser_File/5_Task \
 *       -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Motor/Motor_LK \
 *       -o t Tests/test_motor_lk.c \
 *       User_File/2_Device/Motor/Motor_LK/device_motor_lk.c -lm
 *
 * ⚠⚠ 要【跑两遍】，第二遍加 -DAPP_LK_ENCODER_DIR=(-1.0f)：编码器方向默认 +1 时
 * "乘了方向常量"和"压根没乘"一模一样，单跑默认那遍查不出漏乘——
 * 与 test_shooter 翻 APP_SHOOT_TRIGGER_DIR 是同一个道理。
 *
 * ⚠ 设备层状态是 static 的，用例之间会带着走。testBootOffline 和
 * testFirstFrameNoFalseWrap 依赖"还没收到过任何反馈"，必须排在 main 的最前面。
 */
#include "device_motor_lk.h"

#include "fdcan.h"
#include "task_can.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 桩：捕获发出去的帧 ---- */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

typedef struct
{
    uint32_t sent;                        /* 本次捕获期间发了几帧。 */
    FDCAN_HandleTypeDef *handle;
    uint32_t identifier;
    uint8_t length;
    uint8_t data[APP_LK_FRAME_LEN];
} captured_frame_t;

static captured_frame_t captured;

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    captured.sent++;
    captured.handle = handle;
    captured.identifier = identifier;
    captured.length = length;
    memcpy(captured.data, data, length);
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

/* ---- 辅助 ---- */

#define TWO_PI 6.28318530717958647692f

/**
 * @brief 按协议【小端】手工造一帧 0xA1 回复。
 *
 * ⚠ 故意逐字节写，不复用任何解析公式——打包解包共用同一条公式时，
 * 公式本身错了也能往返自洽。
 */
static void buildReply(uint8_t data[APP_LK_FRAME_LEN], int8_t temperature,
                       int16_t iq, int16_t speedDps, uint16_t encoder)
{
    data[0] = 0xA1U;
    data[1] = (uint8_t)temperature;
    data[2] = (uint8_t)((uint16_t)iq & 0xFFU);
    data[3] = (uint8_t)((uint16_t)iq >> 8U);
    data[4] = (uint8_t)((uint16_t)speedDps & 0xFFU);
    data[5] = (uint8_t)((uint16_t)speedDps >> 8U);
    data[6] = (uint8_t)(encoder & 0xFFU);
    data[7] = (uint8_t)(encoder >> 8U);
}

/** @brief 在拨盘所在总线、正确 ID 上喂一帧回复。 */
static uint8_t feed(uint16_t encoder)
{
    uint8_t data[APP_LK_FRAME_LEN];

    buildReply(data, 30, 0, 0, encoder);
    return Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN3, APP_LK_TRIGGER_ID, data);
}

static void clearCaptured(void)
{
    memset(&captured, 0, sizeof(captured));
}

/* ---- 用例 ---- */

/** @brief 上电还没收到过反馈：离线。拦"isOnline 默认 1"或"只比时间戳"。 */
static void testBootOffline(void)
{
    stubTick = 10U;
    assert(Motor_LK_IsOnline(stubTick) == 0U);
}

/**
 * @brief 首帧编码器靠近满量程时不能误判成回绕。
 *
 * 上电时 lastEncoderRaw 是 0，首帧 65000 与它的差超过半圈，
 * 不加"已在线才判回绕"的话 totalRound 会凭空减一圈、角度跳一大截。
 */
static void testFirstFrameNoFalseWrap(void)
{
    motor_lk_state_t s;

    stubTick = 100U;
    assert(feed(65000U) == 1U);
    Motor_LK_GetState(&s);
    assert(s.totalRound == 0);
    assert(s.totalEncoder == 65000);
}

/**
 * @brief 字段按【小端】解析，温度和转速有符号。
 *
 * 用不对称的字节值：大端解法（照抄 DJI 的习惯）会把 0x0123 读成 0x2301；
 * 温度按无符号读会把 -10 读成 246；转速按无符号读会把 -300 读成 65236。
 */
static void testFeedbackParse(void)
{
    uint8_t data[APP_LK_FRAME_LEN];
    motor_lk_state_t before;
    motor_lk_state_t s;

    Motor_LK_GetState(&before);
    stubTick = 200U;
    buildReply(data, -10, 0x0123, -300, 0x1234U);
    /* 手工核对一遍小端：iq 低字节在 data[2]。 */
    assert(data[2] == 0x23U && data[3] == 0x01U);

    assert(Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN3, APP_LK_TRIGGER_ID, data) == 1U);
    Motor_LK_GetState(&s);
    assert(s.temperature == -10);
    assert(s.currentRaw == 291);        /* 0x0123 */
    assert(s.speedDps == -300);
    assert(s.encoderRaw == 0x1234U);
    assert(s.feedbackCount == before.feedbackCount + 1U);
    assert(s.lastUpdateTick == 200U);
    assert(s.isOnline == 1U);
}

/**
 * @brief 只认 FDCAN3 + 0x141 + 命令字节 0xA1，其余一概不动状态。
 *
 * 三条各拦一种漏判：不比总线（换线或合并总线时读到别人的电机）、
 * 不比 ID（同一条线上挂第二台翎控时串台）、不比命令字节（其他命令的回复
 * 字节布局完全不同，按 0xA1 解会得到一串垃圾）。
 */
static void testRejectsForeignFrames(void)
{
    uint8_t data[APP_LK_FRAME_LEN];
    motor_lk_state_t before;
    motor_lk_state_t s;

    Motor_LK_GetState(&before);
    buildReply(data, 99, 999, 999, 0x5555U);

    assert(Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN2, APP_LK_TRIGGER_ID, data) == 0U);
    assert(Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN3, APP_LK_TRIGGER_ID + 1U, data) == 0U);
    data[0] = 0x9AU; /* 读状态 1 的回复 */
    assert(Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN3, APP_LK_TRIGGER_ID, data) == 0U);

    Motor_LK_GetState(&s);
    assert(s.feedbackCount == before.feedbackCount);
    assert(s.encoderRaw == before.encoderRaw);
    assert(s.currentRaw == before.currentRaw);
}

/**
 * @brief 编码器正反两个方向跨回绕点都能累加对。
 *
 * 拦回绕方向写反（正转一圈却减一圈）和漏累加。
 */
static void testEncoderWrapBothWays(void)
{
    motor_lk_state_t s;
    int32_t start;
    int32_t startRound;

    stubTick = 300U;
    assert(feed(65000U) == 1U);
    Motor_LK_GetState(&s);
    start = s.totalEncoder;
    startRound = s.totalRound;

    /* 正转跨过 65535 -> 0：本帧读数变小，但这是多转了 1036 个计数。 */
    assert(feed(500U) == 1U);
    Motor_LK_GetState(&s);
    assert(s.totalRound == startRound + 1);
    assert(s.totalEncoder == start + 1036);

    /* 原路倒回去。 */
    assert(feed(65000U) == 1U);
    Motor_LK_GetState(&s);
    assert(s.totalRound == startRound);
    assert(s.totalEncoder == start);
}

/**
 * @brief 【输出轴】角：电机端转满 减速比 圈 = 输出轴一圈 = 2π，方向跟 APP_LK_ENCODER_DIR。
 *
 * 期望值 2π 是手写的字面量，不是拿驱动里的公式算的。
 * 拦三种错：漏除减速比（差 10 倍）、用错每圈计数（照抄 DJI 的 8192）、漏乘方向。
 */
static void testOutputShaftAngle(void)
{
    const int32_t perStep = APP_LK_ENCODER_PER_ROUND / 4; /* 远小于半圈，不会被当成反转 */
    int32_t total = (int32_t)((float)APP_LK_ENCODER_PER_ROUND * APP_LK_TRIGGER_GEAR_RATIO);
    motor_lk_state_t s;
    float angleStart;
    int32_t enc;

    stubTick = 400U;
    assert(feed(0U) == 1U);
    Motor_LK_GetState(&s);
    angleStart = s.angle;
    enc = 0;

    while (total > 0)
    {
        int32_t d = (total > perStep) ? perStep : total;

        enc = (enc + d) % APP_LK_ENCODER_PER_ROUND;
        assert(feed((uint16_t)enc) == 1U);
        total -= d;
    }

    Motor_LK_GetState(&s);
    assert(fabsf((s.angle - angleStart) - (TWO_PI * APP_LK_ENCODER_DIR)) < 1e-3f);
}

/** @brief 最后一帧之后 APP_LK_TIMEOUT_TICKS 内在线，再多 1 tick 离线。拦边界写反。 */
static void testOnlineTimeout(void)
{
    stubTick = 1000U;
    assert(feed(0U) == 1U);
    assert(Motor_LK_IsOnline(1000U + APP_LK_TIMEOUT_TICKS) == 1U);
    assert(Motor_LK_IsOnline(1000U + APP_LK_TIMEOUT_TICKS + 1U) == 0U);
}

/**
 * @brief 命令帧：FDCAN3、ID 0x141、{0xA1,0,0,0,iq 低,iq 高,0,0}，【小端】。
 *
 * 拦大端打包（DJI 的 0x200 帧是大端，照抄就反了）、发错总线、发错 ID、
 * 命令字节写成别的模式（0xA2 是速度环，电机会按速度去理解这个数）。
 */
static void testTxFrameLayout(void)
{
    const uint8_t expectPos[APP_LK_FRAME_LEN] = {0xA1U, 0U, 0U, 0U, 0xE8U, 0x03U, 0U, 0U};
    const uint8_t expectNeg[APP_LK_FRAME_LEN] = {0xA1U, 0U, 0U, 0U, 0xFEU, 0xFFU, 0U, 0U};

    Motor_LK_SetSafe(0U);

    clearCaptured();
    Motor_LK_SetCommand(1000);
    Motor_LK_UpdateTxFrames(1U);
    assert(captured.sent == 1U);
    assert(captured.handle == &hfdcan3);
    assert(captured.identifier == 0x141U);
    assert(captured.length == 8U);
    assert(memcmp(captured.data, expectPos, sizeof(expectPos)) == 0);

    clearCaptured();
    Motor_LK_SetCommand(-2);
    Motor_LK_UpdateTxFrames(1U);
    assert(memcmp(captured.data, expectNeg, sizeof(expectNeg)) == 0);
}

/**
 * @brief 命令按 ±APP_LK_IQ_MAX（协议有效范围 ±2000）限幅。
 *
 * 拦漏限幅：30000 不限幅会原样编码成 0x7530，超出协议范围的行为未定义。
 */
static void testCommandClamp(void)
{
    Motor_LK_SetSafe(0U);

    clearCaptured();
    Motor_LK_SetCommand(30000);
    Motor_LK_UpdateTxFrames(1U);
    assert(captured.data[4] == 0xD0U && captured.data[5] == 0x07U); /* 2000 */

    clearCaptured();
    Motor_LK_SetCommand(-30000);
    Motor_LK_UpdateTxFrames(1U);
    assert(captured.data[4] == 0x30U && captured.data[5] == 0xF8U); /* -2000 */
}

/**
 * @brief 安全门置位：照样发帧，但 iq 为 0。
 *
 * 两头都要拦：安全门不生效（电机照出力）；以及安全时干脆不发帧——
 * 翎控只在收到命令时才回复，不发帧就收不到反馈，在线判定会一直掉线。
 */
static void testSafeSendsZeroFrame(void)
{
    Motor_LK_SetCommand(1000);
    Motor_LK_SetSafe(1U);

    clearCaptured();
    Motor_LK_UpdateTxFrames(1U);
    assert(captured.sent == 1U);
    assert(captured.data[0] == 0xA1U);
    assert(captured.data[4] == 0U && captured.data[5] == 0U);

    Motor_LK_SetSafe(0U);
}

/**
 * @brief 总线门为 0 时一帧都不发。
 *
 * 分步装机时拨盘可能还没装：那时发出去的帧没人应答，理由见 device_motor_dji.h。
 */
static void testBusGate(void)
{
    Motor_LK_SetCommand(1000);

    clearCaptured();
    Motor_LK_UpdateTxFrames(0U);
    assert(captured.sent == 0U);
}

int main(void)
{
    /* 这两条依赖"还没收到过任何反馈"，必须最先跑。 */
    testBootOffline();
    testFirstFrameNoFalseWrap();

    testFeedbackParse();
    testRejectsForeignFrames();
    testEncoderWrapBothWays();
    testOutputShaftAngle();
    testOnlineTimeout();
    testTxFrameLayout();
    testCommandClamp();
    testSafeSendsZeroFrame();
    testBusGate();

    printf("test_motor_lk: 11 组用例全过\n");
    return 0;
}
