/*
 * 发射机构模块单测：拨弹状态机（单发/连发）与防卡弹状态机（退弹/放弃）。
 *
 * shooter_control.c 依赖 device_motor_dji.c，后者要 stm32h7xx_hal.h / fdcan.h /
 * task_can.h 三个桩头文件（配方见 Gimbal/CLAUDE.md 的 Host tests 一节），
 * 符号由本文件自己定义。
 *
 * 编译（在 Gimbal/Real/ 下执行）：
 *   gcc -Wall -Wextra -I<stub> -IUser_File/1_Middleware/0_Common \
 *       -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Motor/Motor_DJI \
 *       -IUser_File/3_Chariot/1_Module/Shooter \
 *       -o t Tests/test_shooter.c \
 *       User_File/3_Chariot/1_Module/Shooter/shooter_control.c \
 *       User_File/3_Chariot/1_Module/Shooter/shooter_config.c \
 *       User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c \
 *       User_File/1_Middleware/2_Algorithm/PID.c -lm
 *
 * ⚠ Limit.h 是纯 inline 头文件，【没有 Limit.c】，别往命令里加。
 *
 * ⚠⚠ 这个测试要【跑两遍】，第二遍把送弹方向翻过来：
 *   gcc ... -DAPP_SHOOT_TRIGGER_DIR=(-1.0f) ...
 * 理由：默认 DIR 是 +1，此时"代码乘了这个常量"和"压根没乘"在数学上完全一样，
 * 单跑默认那遍查不出漏乘。实机上把宏翻成 -1 却毫无反应会非常难查，
 * 所以 app_config.h 那边特意留了 #ifndef 让测试能覆盖它。
 *
 * ⚠ 本文件断言的是【目标角的增量】，不是电流值。电流取决于两组未整定的 PID
 * 参数，拿它当判据的话改一次参数就全红；而"一次进使能区 = 目标角正好加一发"
 * 是与参数无关的语义。
 */
#include "shooter_control.h"

#include "remote_input.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 桩 ---- */
typedef struct { int dummy; } FDCAN_HandleTypeDef;
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    (void)handle;
    (void)identifier;
    (void)data;
    (void)length;
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

/* task_remote.c 里的那个全局，测试里自己提供。 */
Remote_t Remote;

/* ---- 辅助 ---- */

#define TICK_S 0.001f
#define EPS 1e-4f

/** @brief 造一帧 C610 反馈并喂给设备层。 */
static void feedMotor(motor_dji_index_t index, uint32_t rxId, app_can_bus_t bus,
                      uint16_t encoder, int16_t rpm, int16_t current,
                      uint8_t errorCode)
{
    uint8_t data[APP_DJI_RX_LEN];

    (void)index;
    memset(data, 0, sizeof(data));
    data[0] = (uint8_t)(encoder >> 8U);
    data[1] = (uint8_t)encoder;
    data[2] = (uint8_t)((uint16_t)rpm >> 8U);
    data[3] = (uint8_t)(uint16_t)rpm;
    data[4] = (uint8_t)((uint16_t)current >> 8U);
    data[5] = (uint8_t)(uint16_t)current;
    data[7] = errorCode;
    assert(Motor_DJI_UpdateFeedback(bus, rxId, data) == 1U);
}

/** @brief 让四台电机都在线、零速零电流、无错误码。 */
static void feedAllHealthy(uint16_t triggerEncoder)
{
    feedMotor(MOTOR_DJI_FRIC_1, APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_2, APP_DJI_FRIC_2_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_3, APP_DJI_FRIC_3_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_TRIGGER, APP_DJI_TRIGGER_RX_ID, APP_CAN_BUS_FDCAN3,
              triggerEncoder, 0, 0, MOTOR_DJI_ERR_NONE);
}

/** @brief 拨盘上报堵转码或大电流，其余三台健康。 */
static void feedTriggerJam(uint16_t encoder, int16_t current, uint8_t errorCode)
{
    feedMotor(MOTOR_DJI_FRIC_1, APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_2, APP_DJI_FRIC_2_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_3, APP_DJI_FRIC_3_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_TRIGGER, APP_DJI_TRIGGER_RX_ID, APP_CAN_BUS_FDCAN3,
              encoder, 0, current, errorCode);
}

/** @brief 跑一个控制周期，dt 自定。VrB 用归一化值直接给。 */
static void stepDt(float knobB, float dt)
{
    stubTick++;
    Remote.knobB = knobB;
    Shooter.dt = dt;
    Shooter_Feedback_Update();
    Shooter_Control();
    Shooter_Command_Send();
}

/** @brief 跑一个 1 ms 控制周期，也就是实机 1 kHz 的标称周期。 */
static void step(float knobB)
{
    stepDt(knobB, TICK_S);
}

/** @brief 三个摩擦轮喂健康帧，拨盘单独给（用来让拨盘真的转起来）。 */
static void feedFricHealthy(void)
{
    feedMotor(MOTOR_DJI_FRIC_1, APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_2, APP_DJI_FRIC_2_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
    feedMotor(MOTOR_DJI_FRIC_3, APP_DJI_FRIC_3_RX_ID, APP_CAN_BUS_FDCAN2,
              0U, 0, 0, MOTOR_DJI_ERR_NONE);
}

/**
 * @brief 让拨盘真的转过 counts 个【转子】计数，counts 带符号。
 *
 * ⚠ 每步跨度必须远小于半圈，否则设备层会把它当成反向回绕。
 */
static void advanceTrigger(int32_t counts, float knobB)
{
    const int32_t perStep = 500;
    int32_t sign = (counts >= 0) ? 1 : -1;
    int32_t left = (counts >= 0) ? counts : -counts;

    while (left > 0)
    {
        int32_t d = (left > perStep) ? perStep : left;
        int32_t enc = ((int32_t)Shooter.motor[MOTOR_DJI_TRIGGER].encoderRaw +
                       (sign * d) + APP_DJI_ENCODER_PER_ROUND) %
                      APP_DJI_ENCODER_PER_ROUND;

        feedFricHealthy();
        /* rpm 与位移自洽：d 个计数 / 1 ms，方向跟着 sign。 */
        feedMotor(MOTOR_DJI_TRIGGER, APP_DJI_TRIGGER_RX_ID, APP_CAN_BUS_FDCAN3,
                  (uint16_t)enc,
                  (int16_t)((float)(sign * d) /
                            (float)APP_DJI_ENCODER_PER_ROUND * 60000.0f),
                  0, MOTOR_DJI_ERR_NONE);
        step(knobB);
        left -= d;
    }
}

/** @brief 拨盘不使能区 / 使能区对应的 VrB 归一化值。 */
#define KNOB_OFF 0.0f
#define KNOB_FIRE 1.0f

/**
 * @brief 把发射机构拉到稳定的待发状态：总开关开、VrB 未进使能区。
 *
 * ⚠ Shooter_Config 是 const，两个按总线分的输出门在主机测试里【打不开】，
 * 所以 Shooter.I[] 恒为 0，断言一律看 I_req 和 trigger_target。
 * "两个门为 0 时一帧不发"由 test_motor_dji 的 testBusEnableGating 守。
 */
static void settleIdle(void)
{
    uint32_t i;

    Shooter.enable_flag = 1U;
    /*
     * ⚠ 要跑够久：上一条用例可能把卡弹机停在 BACKING，而退弹本身要
     * jam_back_s（300 ms）才结束。只跑几拍的话下一条用例会从"还在退弹"
     * 的状态开始，而拨弹逻辑那时是被旁路的。
     */
    for (i = 0U; i < 2000U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_OFF);
        if ((Shooter.trigger_state == SHOOTER_TRIGGER_IDLE) &&
            (Shooter.jam_state == SHOOTER_JAM_NORMAL))
        {
            break;
        }
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_retry == 0U);
}

/* ---- 用例 ---- */

/**
 * @brief 进使能区一次，目标角【正好】加一发，不是两发也不是零发。
 *
 * 这是整套语义的地基：APP_SHOOT_BURST_* 的注释块说"进入使能区间立刻发一发"。
 */
static void testSingleShotOnRisingEdge(void)
{
    float before;

    settleIdle();
    before = Shooter.trigger_target;

    feedAllHealthy(0U);
    step(KNOB_FIRE);

    assert(fabsf((Shooter.trigger_target - before) -
                 APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) < EPS);
}

/**
 * @brief 保持在使能区但没到 BURST_DELAY，仍然只有那一发。
 *
 * 少了这条，"进使能区就一直加"的实现也能过上一条用例。
 */
static void testHoldBelowDelayStaysOneShot(void)
{
    float before;
    uint32_t i;
    uint32_t hold = (APP_SHOOT_BURST_DELAY_MS / 2U);

    settleIdle();
    before = Shooter.trigger_target;

    for (i = 0U; i < hold; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }

    assert(Shooter.trigger_state == SHOOTER_TRIGGER_HOLD);
    assert(fabsf((Shooter.trigger_target - before) -
                 APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) < EPS);
}

/**
 * @brief 保持超过 BURST_DELAY 后按 BURST_HZ 连发，发数对得上时长。
 */
static void testBurstAfterDelay(void)
{
    float before;
    uint32_t i;
    uint32_t burstMs = 1000U; /* 连发持续 1 s */
    uint32_t total = APP_SHOOT_BURST_DELAY_MS + burstMs;
    float shots;
    float expect;

    settleIdle();
    before = Shooter.trigger_target;

    for (i = 0U; i < total; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }

    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    /*
     * 1 发（进使能区那一下）+ 连发 1 s × BURST_HZ。容差给 1.5 发：
     * 节拍相位本来就能差一发，再加上 shots 是浮点除出来的，
     * 卡在整数 1.0 上会被舍入误差推到界外。
     */
    shots = (Shooter.trigger_target - before) /
            (APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR);
    expect = 1.0f + (float)APP_SHOOT_BURST_HZ * (float)burstMs * 0.001f;
    assert(fabsf(shots - expect) <= 1.5f);
}

/**
 * @brief 退出使能区：目标角拉回当前角，计时复位。
 */
static void testReleaseResetsTarget(void)
{
    uint32_t i;

    settleIdle();

    for (i = 0U; i < 100U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    /* 拨盘一动不动，此时目标角已经明显领先反馈角。 */
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) > EPS);

    feedAllHealthy(0U);
    step(KNOB_OFF);

    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
}

/**
 * @brief 连发中途松开再按下，是重新"一发"，不是接着连发。
 */
static void testReenterStartsFromSingle(void)
{
    uint32_t i;
    float before;

    settleIdle();

    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + 200U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    feedAllHealthy(0U);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);

    before = Shooter.trigger_target;
    feedAllHealthy(0U);
    step(KNOB_FIRE);

    /* 再次进入只加一发，而不是直接续上连发节拍。 */
    assert(fabsf((Shooter.trigger_target - before) -
                 APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) < EPS);
}

/**
 * @brief 目标角从【上一次目标】累加，不是从【当前反馈角】累加。
 *
 * ⚠ 这条专打参考实现（RoboWalker crt_booster.cpp:217 的
 * Get_Now_Angle() + 2π/7）的缺陷：上一发还没转到位就来第二发时，
 * 从反馈角累加会让这一发变短，连发和卡弹脱困后必然踩到。
 *
 * 做法：让拨盘【完全不动】（反馈角恒为 0），连打两发。
 * 从目标累加 -> 目标 = 2 发；从反馈累加 -> 目标永远只有 1 发。
 */
static void testTargetAccumulatesFromTargetNotFeedback(void)
{
    float before;
    uint32_t i;

    settleIdle();
    before = Shooter.trigger_target;

    /* 连发 1 s，全程拨盘纹丝不动（编码器恒为 0）。 */
    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + 1000U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }

    /*
     * 从目标累加 -> 约 1 + 8 = 9 发；从反馈角累加 -> 反馈恒为 0，
     * 目标永远只有一发。取 3 发做判据，中间留足余量。
     */
    assert((Shooter.trigger_target - before) /
           (APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) > 3.0f);
}

/**
 * @brief 堵转码持续到确认时长后退弹：目标角回退 jam_back_rad。
 */
static void testJamByErrorCodeBacksOff(void)
{
    uint32_t i;
    uint32_t confirmMs;
    float targetBeforeJam;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    feedAllHealthy(0U);
    step(KNOB_FIRE);

    /* 堵转码一直挂着，但还没到确认时长。 */
    for (i = 0U; i < confirmMs - 2U; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_SUSPECT);

    /*
     * 基准取在【即将确认】的这一拍，而不是卡弹之前——拨弹状态机此时仍在跑，
     * 拿更早的值当基准会把连发加的发数一起算进来。
     */
    targetBeforeJam = Shooter.trigger_target;
    for (i = 0U; i < 5U; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }

    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);
    /*
     * 判"沿送弹方向往回退了"，不是判数值变小——APP_SHOOT_TRIGGER_DIR 翻成 -1 时
     * 退弹会让 trigger_target 变大。乘上 DIR 换成方向无关的说法。
     */
    assert(((targetBeforeJam - Shooter.trigger_target) * APP_SHOOT_TRIGGER_DIR) >
           (Shooter_Config.jam_back_rad * 0.5f));
}

/**
 * @brief 电流饱和【单独】也能触发卡弹，不依赖堵转码。
 *
 * 取或的两条路必须各测一次：只测一条的话，实现里把另一条写死成 0 也能过。
 */
static void testJamByCurrentSaturationAlone(void)
{
    uint32_t i;
    uint32_t confirmMs;
    int16_t bigCurrent = (int16_t)APP_DJI_CURRENT_MAX;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    for (i = 0U; i < confirmMs + 5U; i++)
    {
        /* errorCode 保持 NONE，只有电流顶满。 */
        feedTriggerJam(0U, bigCurrent, MOTOR_DJI_ERR_NONE);
        step(KNOB_FIRE);
    }

    assert(Shooter.jam_state != SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_state != SHOOTER_JAM_SUSPECT);
}

/**
 * @brief 卡弹处理期间拨弹逻辑被旁路：VrB 一直按着，目标角也不再加发。
 *
 * 少了这条，"退弹指令被下一轮拨弹逻辑覆盖"这个缺陷测不出来——
 * 那正是 RoboWalker 用 crt_booster.cpp:155 那个 if 防住的东西。
 */
static void testTriggerBypassedWhileBacking(void)
{
    uint32_t i;
    uint32_t confirmMs;
    uint32_t observeMs;
    float targetAtBacking;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    /* 先按到进连发，让拨弹逻辑处在"会持续加发"的状态。 */
    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + 50U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    /* 再卡住，跑到确认退弹。 */
    for (i = 0U; i < confirmMs + 5U; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);
    targetAtBacking = Shooter.trigger_target;

    /*
     * 退弹期间继续按着 VrB，目标角必须纹丝不动。
     *
     * ⚠ 观察窗口要【夹在】两个时间之间，两边都卡死：
     *   - 比一个连发节拍 1000/BURST_HZ 长，否则旁路被删掉也看不出来
     *     （短窗口里连发本来就还没到下一拍）；
     *   - 比 jam_back_s 短，否则退弹已经结束、拨弹逻辑本来就该恢复。
     */
    observeMs = (uint32_t)(Shooter_Config.jam_back_s * 1000.0f) - 50U;
    assert(observeMs > (1000U / APP_SHOOT_BURST_HZ));
    for (i = 0U; i < observeMs; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }
    assert(fabsf(Shooter.trigger_target - targetAtBacking) < EPS);
}

/**
 * @brief 连续退弹 jam_retry_max 次仍未脱困 -> GIVEUP，拨盘电流清零并置 fault。
 *
 * ⚠ 参考的两个工程都【没有】这一层：RoboWalker 的四态是死循环，
 * 可以永远"退一点、再顶、再退"；SPR 作者自己在注释里点了同一个问题。
 */
static void testGiveUpAfterRetries(void)
{
    uint32_t i;

    settleIdle();

    /* 一直卡着，跑足够长时间走完所有重试。 */
    for (i = 0U; i < 20000U; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
        if (Shooter.jam_state == SHOOTER_JAM_GIVEUP)
        {
            break;
        }
    }

    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);
    assert(Shooter.jam_retry >= Shooter_Config.jam_retry_max);
    assert(Shooter.I_req[MOTOR_DJI_TRIGGER] == 0);
    /*
     * ⚠ GIVEUP 故意【不】置 fault：fault 会经 safe_flag 把四台电机全清零，
     * 而拨盘卡死时摩擦轮应该继续转。这条断言钉死"只封锁拨盘"。
     */
    assert(Shooter.fault == SHOOTER_FAULT_NONE);
    assert(Shooter.I_req[MOTOR_DJI_FRIC_1] != 0);
}

/**
 * @brief GIVEUP 只能靠 VrB 退出使能区解除，堵转消失本身解不开。
 */
static void testGiveUpOnlyClearedByRelease(void)
{
    uint32_t i;

    settleIdle();
    for (i = 0U; i < 20000U; i++)
    {
        feedTriggerJam(0U, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
        if (Shooter.jam_state == SHOOTER_JAM_GIVEUP)
        {
            break;
        }
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);

    /* 堵转消失，但 VrB 还按着：不能自己解除。 */
    for (i = 0U; i < 2000U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);

    /* 松开 VrB 才解除。 */
    feedAllHealthy(0U);
    step(KNOB_OFF);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_retry == 0U);
}

/**
 * @brief 摩擦轮由 VrB 过中位驱动：不请求时目标为零且积分被清。
 */
static void testFrictionFollowsRequest(void)
{
    uint32_t i;

    settleIdle();
    assert(Shooter.fric_request == 0U);
    for (i = 0U; i < APP_DJI_FRIC_COUNT; i++)
    {
        assert(Shooter.I_req[i] == 0);
    }

    /* VrB 过中位但没到拨弹区：摩擦轮转、拨盘不动。 */
    feedAllHealthy(0U);
    step(0.5f);
    assert(Shooter.fric_request == 1U);
    assert(Shooter.trigger_request == 0U);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(Shooter.I_req[MOTOR_DJI_FRIC_1] != 0);
}

/**
 * @brief 方向常量真的被用上了：一发的增量符号跟着 APP_SHOOT_TRIGGER_DIR 走。
 */
static void testDirectionConstantIsApplied(void)
{
    float before;

    settleIdle();
    before = Shooter.trigger_target;

    feedAllHealthy(0U);
    step(KNOB_FIRE);

    if (APP_SHOOT_TRIGGER_DIR > 0.0f)
    {
        assert(Shooter.trigger_target > before);
    }
    else
    {
        assert(Shooter.trigger_target < before);
    }
}


/**
 * @brief 短暂的大电流不算卡弹：判据在确认时长内消失就回落 NORMAL，不退弹。
 *
 * 弹丸挤压、起转那一下都会顶出大电流。少了这条回落，
 * "一沾判据就退弹"的实现也能过后面那几条卡弹用例。
 */
static void testBriefSpikeDoesNotJam(void)
{
    uint32_t i;
    uint32_t confirmMs;
    float targetBefore;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    feedAllHealthy(0U);
    step(KNOB_FIRE);
    targetBefore = Shooter.trigger_target;

    /* 顶一下，但远不到确认时长。 */
    for (i = 0U; i < confirmMs / 5U; i++)
    {
        feedTriggerJam(0U, (int16_t)APP_DJI_CURRENT_MAX, MOTOR_DJI_ERR_NONE);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_SUSPECT);

    /* 电流回落，应当立刻回 NORMAL，且一次都没退过弹。 */
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_retry == 0U);
    assert(Shooter.trigger_target >= targetBefore);
}

/**
 * @brief 连发节拍按时间走，不随 dt 的零头漂移。
 *
 * ⚠ 用一个【故意粗】的 dt：dt 正好整除节拍时（实机 1 kHz / 8 Hz = 125 拍整）
 * "减去一个周期"和"清零"结果完全一样，这条差异只有在除不尽时才看得见。
 * 实现写成清零的话，每一发都要向上取整到下一个 dt，长期偏慢。
 */
static void testBurstRateIndependentOfDt(void)
{
    const float coarseDt = 0.04f; /* 25 Hz，节拍 0.125 s 除不尽 */
    const float runS = 4.0f;
    uint32_t ticks = (uint32_t)(runS / coarseDt);
    uint32_t i;
    float before;
    float shots;
    float expect;

    settleIdle();

    /* 先用粗 dt 跨过连发延迟。 */
    for (i = 0U; i < (uint32_t)((float)APP_SHOOT_BURST_DELAY_MS * 0.001f /
                                coarseDt) + 2U; i++)
    {
        feedAllHealthy(0U);
        stepDt(KNOB_FIRE, coarseDt);
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    before = Shooter.trigger_target;
    for (i = 0U; i < ticks; i++)
    {
        feedAllHealthy(0U);
        stepDt(KNOB_FIRE, coarseDt);
    }

    shots = (Shooter.trigger_target - before) /
            (APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR);
    expect = (float)APP_SHOOT_BURST_HZ * runS;
    /* 清零实现会慢约 28%（每发要 4 拍而不是 3.125 拍），容差 2 发抓得住。 */
    assert(fabsf(shots - expect) <= 2.0f);
}


/**
 * @brief 串级环真的接上了：目标领先反馈时，两级的输出都朝送弹方向，且外环被限幅。
 *
 * ⚠ 前面那些用例只看 trigger_target，串级环怎么接的一概测不到——
 * 把位置环的 target/feedback 喂反、或者速度环没吃位置环的输出，
 * 目标角照样一模一样。
 */
static void testCascadeDrivesTowardTarget(void)
{
    uint32_t i;

    settleIdle();

    /* 待发状态：目标 == 反馈，拨盘不该出力。 */
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
    assert(Shooter.I_req[MOTOR_DJI_TRIGGER] == 0);

    /* 打一发，拨盘不动，于是位置误差就是一整发。 */
    feedAllHealthy(0U);
    step(KNOB_FIRE);

    if (APP_SHOOT_TRIGGER_DIR > 0.0f)
    {
        assert(Shooter.trigger_target_omega > 0.0f);
        assert(Shooter.I_req[MOTOR_DJI_TRIGGER] > 0);
    }
    else
    {
        assert(Shooter.trigger_target_omega < 0.0f);
        assert(Shooter.I_req[MOTOR_DJI_TRIGGER] < 0);
    }

    /* 误差再大，外环输出也不许超过 outputLimit——那是连发跟不跟得上的上限。 */
    for (i = 0U; i < 200U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(fabsf(Shooter.trigger_target_omega) <=
           Shooter_Config.trigger_angle_pid.outputLimit + EPS);
}


/**
 * @brief 拨盘的反馈量取自设备层，且换到【输出轴】口径。
 *
 * ⚠ 漏除减速比会差 36 倍。前面那些用例里拨盘一动不动、speedRpm 恒为 0，
 * 所以单位错了也看不出来——必须专门喂一个非零转速。
 */
static void testTriggerFeedbackUnits(void)
{
    const int16_t rotorRpm = 3600;
    float expectOmega;

    settleIdle();

    feedFricHealthy();
    feedMotor(MOTOR_DJI_TRIGGER, APP_DJI_TRIGGER_RX_ID, APP_CAN_BUS_FDCAN3,
              1000U, rotorRpm, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);

    /* 反馈角必须就是设备层那份，不能是别处算的第二份。 */
    assert(fabsf(Shooter.trigger_angle -
                 Shooter.motor[MOTOR_DJI_TRIGGER].angle) < EPS);

    /* 转子 3600 rpm -> 输出轴 3600/36 = 100 rpm = 10.47 rad/s。 */
    expectOmega = (float)rotorRpm * (2.0f * APP_PI_F / 60.0f) / APP_DJI_GEAR_RATIO;
    assert(fabsf(Shooter.trigger_omega - expectOmega) < 1e-3f);
}

/**
 * @brief 拨盘真转过一发之后误差归零、电流退回去——单发是会【停】的。
 *
 * 这条是"单发"真正的落地判据：前面的用例都只证明目标角加对了，
 * 没有证明拨盘转到位之后会停下来。
 */
static void testShotCompletesWhenTriggerTurns(void)
{
    int32_t countsPerAmmo =
        (int32_t)((float)APP_DJI_ENCODER_PER_ROUND * APP_DJI_GEAR_RATIO /
                  (float)APP_SHOOT_AMMO_PER_ROUND);
    float target;

    settleIdle();

    feedAllHealthy(Shooter.motor[MOTOR_DJI_TRIGGER].encoderRaw);
    step(KNOB_FIRE);
    target = Shooter.trigger_target;
    assert(Shooter.I_req[MOTOR_DJI_TRIGGER] != 0);

    /*
     * 松开 VrB 会把目标拉回反馈角，所以这里保持按住、让它转到位。
     * ⚠ 转的方向要跟着 APP_SHOOT_TRIGGER_DIR，否则把宏翻成 -1 时
     * 这条用例会朝着离目标更远的方向转。
     */
    advanceTrigger((int32_t)((float)countsPerAmmo * APP_SHOOT_TRIGGER_DIR),
                   KNOB_FIRE);

    /* 目标没变（还没到连发延迟），而反馈已经追上来了。 */
    assert(fabsf(Shooter.trigger_target - target) < EPS);
    assert(fabsf(Shooter.trigger_angle - target) < 0.05f);
}

int main(void)
{
    Shooter_Init();

    testSingleShotOnRisingEdge();
    testHoldBelowDelayStaysOneShot();
    testBurstAfterDelay();
    testReleaseResetsTarget();
    testReenterStartsFromSingle();
    testTargetAccumulatesFromTargetNotFeedback();
    testTriggerFeedbackUnits();
    testCascadeDrivesTowardTarget();
    testShotCompletesWhenTriggerTurns();
    testBriefSpikeDoesNotJam();
    testJamByErrorCodeBacksOff();
    testJamByCurrentSaturationAlone();
    testTriggerBypassedWhileBacking();
    testGiveUpAfterRetries();
    testGiveUpOnlyClearedByRelease();
    testFrictionFollowsRequest();
    testDirectionConstantIsApplied();
    testBurstRateIndependentOfDt();

    printf("test_shooter: 18 组用例全过\n");
    return 0;
}
