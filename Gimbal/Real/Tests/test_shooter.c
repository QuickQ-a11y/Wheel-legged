/*
 * 发射机构模块单测：拨弹状态机（单发/连发）与防卡弹状态机（退弹/放弃）。
 *
 * shooter_control.c 依赖 device_motor_dji.c，后者要 stm32h7xx_hal.h / fdcan.h 两个桩
 * （在 Tests/stub，必须排在 include 路径最前面），符号由本文件自己定义。
 *
 * 输出门出厂都是 0，写死在 shooter_config.c 里（与底盘同一个风格，不留 #ifndef）。
 * 要打开就 sed 出一份临时配置、拿它代替 shooter_config.c 去链——sed 【强制】写成要的值，
 * 不假设工作区当前是几（理由同 Chassis/CLAUDE.md 的 config overrides 一节）：
 *
 *   CFG=User_File/3_Chariot/1_Module/Shooter/shooter_config.c
 *   sed -e 's/\.fric_flag = 0U,/.fric_flag = 1U,/' \
 *       -e 's/\.trigger_flag = 0U,/.trigger_flag = 1U,/' $CFG > $TMP/shooter_cfg_open.c
 *   sed -e 's/\.fric_flag = 1U,/.fric_flag = 0U,/' \
 *       -e 's/\.trigger_flag = 0U,/.trigger_flag = 1U,/' $CFG > $TMP/shooter_cfg_trig.c
 *
 * 编译（在 Gimbal/Real/ 下执行；<变体> 与 <配置> 见下）：
 *   gcc -Wall -Wextra <变体> -ITests/stub -IUser_File/5_Task \
 *       -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Communication/FlySky \
 *       -IUser_File/2_Device/Motor/Motor_DJI -IUser_File/2_Device/Motor/Motor_LK \
 *       -IUser_File/3_Chariot/1_Module/Shooter \
 *       -o t Tests/test_shooter.c \
 *       User_File/3_Chariot/1_Module/Shooter/shooter_control.c \
 *       <配置> \
 *       User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c \
 *       User_File/2_Device/Motor/Motor_LK/device_motor_lk.c \
 *       User_File/1_Middleware/2_Algorithm/PID.c -lm
 *
 * ⚠ <变体> 里必须带 -DTEST_FRIC_FLAG=0/1 -DTEST_TRIGGER_FLAG=0/1，声明这一遍【期望】的门，
 * main 一开头就拿它核对 Shooter_Config。sed 一旦没匹配上（比如有人把那一行对齐了空格），
 * 门就还关着，依赖门开的用例会静默跳过、整遍照样"全过"——少了这道核对就是假通过。
 *
 * ⚠ Limit.h 是纯 inline 头文件，【没有 Limit.c】，别往命令里加。
 * ⚠ FlySky 目录是给 task_remote.h 用的：它按 APP_REMOTE_BACKEND 引后端头文件，
 *   换后端时要跟着换成 DR16 目录。
 *
 * ⚠⚠ 这个测试要跑【5 遍】，每一遍都在变异检查里独家抓住过错误：
 *   1. -DAPP_TRIGGER_MOTOR=APP_TRIGGER_M2006 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1，
 *      配置用 shooter_cfg_open.c。两个输出门【打开】：门关着时 Shooter.I 恒为 0，
 *      安全门、急停刹车、故障清零一行都测不到。拨盘堵转码把摩擦轮一起停掉的缺陷，
 *      就是这样在门关着的单测里藏了一轮。
 *   2. 同 1，再加 -DAPP_SHOOT_TRIGGER_DIR=(-1.0f)：默认 DIR 是 +1，此时"代码乘了这个常量"
 *      和"压根没乘"在数学上完全一样，查不出漏乘。
 *   3. -DAPP_TRIGGER_MOTOR=APP_TRIGGER_M2006 -DTEST_FRIC_FLAG=0 -DTEST_TRIGGER_FLAG=1，
 *      配置用 shooter_cfg_trig.c（摩擦轮门关）：分步装机时未使能总线上的电机不能计入故障。
 *   4. -DAPP_TRIGGER_MOTOR=APP_TRIGGER_LK4005 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1，
 *      配置用 shooter_cfg_open.c：翎控拨盘专属的换算、在线判定、帧路由。
 *   5. 同 4，再加 -DAPP_SHOOT_TRIGGER_DIR=(-1.0f)。
 *
 * ⚠ 本文件断言的多是【目标角的增量】和请求量，不是电流的具体数值。电流取决于未整定的
 * PID 参数，拿它当判据的话改一次参数就全红；需要看电流的地方只断言符号、比例和是否为零。
 */
#include "shooter_control.h"

#include "device_motor_lk.h"
#include "fdcan.h"
#include "remote_input.h"
#include "task_can.h"
#include "task_remote.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined(TEST_FRIC_FLAG) || !defined(TEST_TRIGGER_FLAG)
#error "test_shooter: 用 -DTEST_FRIC_FLAG=0/1 -DTEST_TRIGGER_FLAG=0/1 声明本遍期望的输出门，见文件头"
#endif

/* ---- 桩 ---- */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* FDCAN3 上两种拨盘帧各发了几次，检查"只发选中的那台"。 */
static uint32_t txDjiTriggerCount; /* 0x200，C610 拨盘帧 */
static uint32_t txLkTriggerCount;  /* 0x141，翎控拨盘帧 */

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    (void)data;
    (void)length;
    if ((handle == &hfdcan3) && (identifier == APP_DJI_TX_ID))
    {
        txDjiTriggerCount++;
    }
    if ((handle == &hfdcan3) && (identifier == APP_LK_TRIGGER_ID))
    {
        txLkTriggerCount++;
    }
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

/* 遥控快照由测试直接给：发射机构自己取，不再借云台任务刷新的全局 Remote。 */
static Remote_t stubRemote;
void Remote_Task_GetRemote(Remote_t *remote) { *remote = stubRemote; }

/* ---- 辅助 ---- */

#define TICK_S 0.001f
#define EPS 1e-4f

/*
 * 拨盘电调口径，按型号独立写，不复用生产代码的换算宏：
 *   FULL_SCALE_RAW         拿来造"电流顶满"的卡弹
 *   RAW_PER_A              每安培的原始计数，力矩限幅的期望值用它算
 *   ENC_PER_ROUND          反馈帧里编码器字段的量程，只用来【生成】自洽的反馈帧
 *   COUNTS_PER_OUTPUT_ROUND  输出轴转一圈，编码器字段走过的计数
 */
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
/* 翎控 0xA1：iq 协议范围 ±2000；每安培按本机手册 ±2048 ↔ ±16.5 A（⚠ 未实机核对）。 */
#define TRIGGER_FULL_SCALE_RAW ((int16_t)2000)
#define TRIGGER_RAW_PER_A (2048.0f / 16.5f)
/* 2026-09-30 实测：回复帧编码器在【输出端】，输出轴一圈回绕一次、最大值 65535。 */
#define TRIGGER_ENC_PER_ROUND 65536
#define TRIGGER_COUNTS_PER_OUTPUT_ROUND 65536
#else
/* C610：±10000 = ±10 A（手册 §CAN 通信协议）；编码器在转子上，减速箱 36:1。 */
#define TRIGGER_FULL_SCALE_RAW ((int16_t)10000)
#define TRIGGER_RAW_PER_A 1000.0f
#define TRIGGER_ENC_PER_ROUND 8192
#define TRIGGER_COUNTS_PER_OUTPUT_ROUND (8192 * 36)
#endif

/*
 * 用例用的拨盘力矩限幅，A。trigger_default_I_limit 是实机上随时会调的值（上机第一步
 * 就要临时改成 0），用例不能跟着它变，见 initShooter。
 */
#define TEST_TRIGGER_I_LIMIT_A 3.0f

/** @brief 造一帧 C610 反馈并喂给设备层。 */
static void feedDji(uint32_t rxId, app_can_bus_t bus, uint16_t encoder, int16_t rpm,
                    int16_t current, uint8_t errorCode)
{
    uint8_t data[APP_DJI_RX_LEN];

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

#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
/**
 * @brief 拨盘一帧反馈：翎控 0xA1 回复，【小端】逐字节手写。
 *
 * speedNative 是电机端 dps。翎控的 0xA1 回复里没有错误码，errorCode 忽略。
 */
static void feedTrigger(uint16_t encoder, int16_t speedNative, int16_t current,
                        uint8_t errorCode)
{
    uint8_t data[APP_LK_FRAME_LEN];

    (void)errorCode;
    data[0] = 0xA1U;
    data[1] = 30U;
    data[2] = (uint8_t)((uint16_t)current & 0xFFU);
    data[3] = (uint8_t)((uint16_t)current >> 8U);
    data[4] = (uint8_t)((uint16_t)speedNative & 0xFFU);
    data[5] = (uint8_t)((uint16_t)speedNative >> 8U);
    data[6] = (uint8_t)(encoder & 0xFFU);
    data[7] = (uint8_t)(encoder >> 8U);
    assert(Motor_LK_UpdateFeedback(APP_CAN_BUS_FDCAN3, APP_LK_TRIGGER_ID, data) == 1U);
}

/**
 * @brief 1 ms 内编码器走过 counts 个计数时，回复帧里的转速，dps。
 *
 * 2026-09-30 实测：编码器在【输出端】，转速却在【电机端】（输出轴约一圈每秒时
 * speedDps 三千多），所以输出端的 dps 要再乘减速比 10。
 */
static int16_t speedFromCountsPerMs(int32_t counts)
{
    return (int16_t)((float)counts / (float)TRIGGER_COUNTS_PER_OUTPUT_ROUND * 360000.0f *
                     10.0f);
}
#else
/** @brief 拨盘一帧反馈：C610，speedNative 是转子 rpm。 */
static void feedTrigger(uint16_t encoder, int16_t speedNative, int16_t current,
                        uint8_t errorCode)
{
    feedDji(APP_DJI_TRIGGER_RX_ID, APP_CAN_BUS_FDCAN3, encoder, speedNative, current,
            errorCode);
}

/** @brief 1 ms 内转过 counts 个转子计数时的转速，rpm。 */
static int16_t speedFromCountsPerMs(int32_t counts)
{
    return (int16_t)((float)counts / (float)TRIGGER_ENC_PER_ROUND * 60000.0f);
}
#endif

/** @brief 三个摩擦轮同一转速、零电流、无错误码。 */
static void feedFric(int16_t rpm)
{
    feedDji(APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2, 0U, rpm, 0, MOTOR_DJI_ERR_NONE);
    feedDji(APP_DJI_FRIC_2_RX_ID, APP_CAN_BUS_FDCAN2, 0U, rpm, 0, MOTOR_DJI_ERR_NONE);
    feedDji(APP_DJI_FRIC_3_RX_ID, APP_CAN_BUS_FDCAN2, 0U, rpm, 0, MOTOR_DJI_ERR_NONE);
}

/** @brief 三个摩擦轮喂健康帧（静止），拨盘单独给。 */
static void feedFricHealthy(void)
{
    feedFric(0);
}

/** @brief 让四台电机都在线、零速零电流、无错误码。 */
static void feedAllHealthy(uint16_t triggerEncoder)
{
    feedFricHealthy();
    feedTrigger(triggerEncoder, 0, 0, MOTOR_DJI_ERR_NONE);
}

/**
 * @brief 拨盘卡住：停在【当前位置】不动、电流顶满，摩擦轮健康。
 *
 * 用"电流饱和"这条判据造卡弹，与电机型号无关；堵转码那条只有 C610 有，
 * 在 testJamByErrorCodeBacksOff 里单独测。
 * 编码器沿用上一帧的值：先 advanceTrigger 转进一发再卡，卡住的位置才是转进去的那里。
 */
static void feedTriggerJam(void)
{
    feedFricHealthy();
    feedTrigger(Shooter.trigger_motor.encoderRaw, 0, TRIGGER_FULL_SCALE_RAW, MOTOR_DJI_ERR_NONE);
}

/** @brief 遥控在线、SwC 在中位（运行），VrB 给定。 */
static void remoteRun(float knobB)
{
    memset(&stubRemote, 0, sizeof(stubRemote));
    stubRemote.online = 1U;
    stubRemote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    stubRemote.knobB = knobB;
}

/** @brief 遥控在线、SwC 打到 UP（急停），VrB 给定。 */
static void remoteEstop(float knobB)
{
    remoteRun(knobB);
    stubRemote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UP;
}

/**
 * @brief 遥控离线，但快照里还留着上一帧的拨杆和旋钮。
 *
 * task_remote 离线时会把快照整体清零，但判定不能靠这个巧合——
 * 这里故意留着"SwC 在中位、VrB 在拨弹区"，只把 online 置 0。
 */
static void remoteOfflineStale(float knobB)
{
    remoteRun(knobB);
    stubRemote.online = 0U;
}

static uint8_t fricGateOpen(void) { return Shooter_Config.output.fric_flag; }
static uint8_t triggerGateOpen(void) { return Shooter_Config.output.trigger_flag; }

/** @brief 跑一个控制周期，dt 自定。遥控快照用当前的 stubRemote。 */
static void stepRemoteDt(float dt)
{
    stubTick++;
    Shooter.dt = dt;
    Shooter_Feedback_Update();
    Shooter_Control();
    Shooter_Command_Send();
}

/** @brief 遥控在线运行档、VrB 给定，跑一个周期，dt 自定。 */
static void stepDt(float knobB, float dt)
{
    remoteRun(knobB);
    stepRemoteDt(dt);
}

/** @brief 跑一个 1 ms 控制周期，也就是实机 1 kHz 的标称周期。 */
static void step(float knobB)
{
    stepDt(knobB, TICK_S);
}

/**
 * @brief 让拨盘真的转过 counts 个【电机端】编码器计数，counts 带符号。
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
        int32_t enc = ((int32_t)Shooter.trigger_motor.encoderRaw +
                       (sign * d) + TRIGGER_ENC_PER_ROUND) %
                      TRIGGER_ENC_PER_ROUND;

        feedFricHealthy();
        /* 转速与位移自洽：d 个计数 / 1 ms，方向跟着 sign。 */
        feedTrigger((uint16_t)enc, speedFromCountsPerMs(sign * d), 0, MOTOR_DJI_ERR_NONE);
        step(knobB);
        left -= d;
    }
}

/**
 * @brief 初始化发射机构，并把拨盘力矩限幅换成用例自己给定的值。
 *
 * 先核对 Shooter_Init 确实把配置默认值载进了 trigger_I_limit（这是要测的行为），
 * 再换成 TEST_TRIGGER_I_LIMIT_A——默认值本身是实机调参值，工作区里可能正临时改成 0，
 * 用例跟着它走的话，调一次参单测就全红。
 */
static void initShooter(void)
{
    Shooter_Init();
    assert(Shooter.trigger_I_limit == Shooter_Config.trigger_default_I_limit);
    Shooter.trigger_I_limit = TEST_TRIGGER_I_LIMIT_A;
    /* 回拨开关同理：先核对 Init 载入了配置默认值，再打开——机械组可能正把默认值改成 0。 */
    assert(Shooter.jam_back_flag == Shooter_Config.jam_back_default_flag);
    Shooter.jam_back_flag = 1U;
}

/** @brief 拨盘不使能区 / 使能区对应的 VrB 归一化值。 */
#define KNOB_OFF 0.0f
#define KNOB_FIRE 1.0f

/**
 * @brief 把发射机构拉到稳定的待发状态：遥控在线运行档、VrB 未进使能区。
 */
static void settleIdle(void)
{
    uint32_t i;

    /*
     * 先打一拍急停：两层状态机一起复位，退弹、在途那一发统统放弃。
     * ⚠ 不能只靠松开 VrB：松开之后在途那一发要打完才回待发，而用例里的拨盘
     * 多半纹丝不动、永远打不完。急停是唯一"立刻放弃"的路径，实机上也一样。
     */
    remoteEstop(KNOB_OFF);
    feedAllHealthy(0U);
    stepRemoteDt(TICK_S);

    /* 再松开急停、退出拨弹区，解开上膛锁。 */
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

/** @brief 一发对应的拨盘编码器计数。 */
static int32_t countsPerAmmo(void)
{
    return (int32_t)((float)TRIGGER_COUNTS_PER_OUTPUT_ROUND / (float)APP_SHOOT_AMMO_PER_ROUND);
}

/** @brief 输出轴弧度 -> 拨盘编码器计数（带符号）。 */
static int32_t countsOfRad(float rad)
{
    return (int32_t)(rad / (2.0f * APP_PI_F) * (float)TRIGGER_COUNTS_PER_OUTPUT_ROUND);
}

/** @brief 连发攒积压：拨盘不动、电流为零（不会判卡弹），按住到至少过 3 个连发节拍。 */
static void holdBurstWithoutMoving(void)
{
    uint32_t i;

    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + (3000U / APP_SHOOT_BURST_HZ) + 10U; i++)
    {
        feedAllHealthy(Shooter.trigger_motor.encoderRaw);
        step(KNOB_FIRE);
    }
}

/** @brief 一直卡着、VrB 停在 knobB，跑到卡弹机进入（或离开）指定状态为止。 */
static void jamUntil(shooter_jam_state_t state, uint8_t entering, float knobB)
{
    uint32_t i;

    for (i = 0U; i < 5000U; i++)
    {
        if ((Shooter.jam_state == state) == (entering != 0U))
        {
            return;
        }
        feedTriggerJam();
        step(knobB);
    }
    assert(0); /* 5 s 都没等到，状态机卡住了 */
}

/**
 * @brief 回拨中拨盘真的往回退：每拍走一点，走满 jam_back_rad 就停在那里，直到回拨结束。
 *
 * 每轮循环正好一拍，返回回拨持续了几拍（ms）。
 */
static uint32_t reverseUntilBackingEnds(void)
{
    int32_t backCounts = countsOfRad(Shooter_Config.jam_back_rad);
    uint32_t backingMs = 0U;

    while ((Shooter.jam_state == SHOOTER_JAM_BACKING) && (backingMs < 2000U))
    {
        if (backCounts > 0)
        {
            int32_t d = (backCounts > 500) ? 500 : backCounts;

            advanceTrigger(-(int32_t)((float)d * APP_SHOOT_TRIGGER_DIR), KNOB_FIRE);
            backCounts -= d;
        }
        else
        {
            feedAllHealthy(Shooter.trigger_motor.encoderRaw);
            step(KNOB_FIRE);
        }
        backingMs++;
    }
    return backingMs;
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
 * @brief 单发途中松开 VrB：在途的那一发照样打完，转到位才回待发（目标跟随反馈）。
 *
 * 拦"松开即放弃"——改之前就是这样：目标立刻拉回反馈角，拨盘停在槽位中间、弹推到一半，
 * 之后每一发都带着这截相位差（探针 E：按 30 ms 就松开，这一发只转了 0.60 发）。
 * 松开只是不再加新的一发；RoboWalker、DJI 的单发都是设完目标就不管按键。
 */
static void testReleaseFinishesInFlightShot(void)
{
    float start;
    float shotEnd;

    settleIdle();
    start = Shooter.trigger_angle;
    shotEnd = start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR;

    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    advanceTrigger((int32_t)(0.3f * (float)countsPerAmmo() * APP_SHOOT_TRIGGER_DIR), KNOB_FIRE);

    /* 转了 30% 就松开：目标还是这一发的终点，拨盘还在往前推。 */
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);
    assert(fabsf(Shooter.trigger_target - shotEnd) < EPS);
    assert((float)Shooter.trigger_I_req * APP_SHOOT_TRIGGER_DIR > 0.0f);

    /* 转完剩下的：到位回待发。 */
    advanceTrigger(countsOfRad(shotEnd - Shooter.trigger_angle), KNOB_OFF);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
}

/**
 * @brief 收尾停在终点前一点（容差以内）回了待发，下一发仍从【上一发的终点】累加。
 *
 * 拦"待发后从反馈角起算"：收尾到容差以内就回待发，待发态目标跟随反馈，拨盘停在终点前一点；
 * 下一发要是从这里起算，每松开一次就丢一截相位，一发发漂出去。连发时目标从上一次目标累加，
 * 不会漂；松开再按也不该漂。
 */
static void testNextShotFromLastTarget(void)
{
    float start;
    float shotEnd;

    settleIdle();
    start = Shooter.trigger_angle;
    shotEnd = start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR;

    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);

    /* 停在终点前半个容差：收尾算完成，回待发。 */
    advanceTrigger(countsOfRad(shotEnd - Shooter.trigger_angle -
                               0.5f * Shooter_Config.trigger_arrive_rad * APP_SHOOT_TRIGGER_DIR),
                   KNOB_OFF);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);

    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    assert(fabsf(Shooter.trigger_target -
                 (start + 2.0f * APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR)) < EPS);
}

/**
 * @brief 连发中途松开再按下，是重新"一发"，不是接着连发。
 */
static void testReenterStartsFromSingle(void)
{
    uint32_t i;
    float before;
    float start;

    settleIdle();
    start = Shooter.trigger_angle;

    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + 200U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    /*
     * 松开：只留在途的那一发，后面攒的扔掉——拨盘一动没动，在途的就是第一发（终点 start + 1 发）。
     * 拦"松开把积压也打完"：连发跟不上时松开，还会把一串发补打出去。
     */
    feedAllHealthy(0U);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);
    assert(fabsf(Shooter.trigger_target - (start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR)) <
           EPS);

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

#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
/**
 * @brief 堵转码持续到确认时长后回拨：从反馈角往回退 jam_back_rad，电流朝退弹方向。
 *
 * 反馈电流故意给 0——C610 进了堵转、关了输出就是这样：电流判据失效，只剩堵转码，
 * 照样要判出来、照样发反向命令（电调听不听要实机测）。
 *
 * 堵转码是 C610 独有的判据（反馈 data[7]），电流饱和那条另测。
 * 翎控的 0xA1 回复里没有错误码，这条只在 M2006 下跑。
 */
static void testJamByErrorCodeBacksOff(void)
{
    uint32_t i;
    uint32_t confirmMs;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    feedAllHealthy(0U);
    step(KNOB_FIRE);

    /* 堵转码一直挂着，但还没到确认时长。 */
    for (i = 0U; i < confirmMs - 2U; i++)
    {
        feedFricHealthy();
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_SUSPECT);

    for (i = 0U; i < 5U; i++)
    {
        feedFricHealthy();
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }

    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);
    /* 方向都乘了 DIR：判"沿送弹方向在反馈角后面"，DIR 翻成 -1 时同样成立。 */
    assert(fabsf((Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR +
                 Shooter_Config.jam_back_rad) < EPS);
    assert((float)Shooter.trigger_I_req * APP_SHOOT_TRIGGER_DIR < 0.0f);
}
#endif

/**
 * @brief 电流饱和【单独】也能触发卡弹，不依赖堵转码。
 *
 * 取或的两条路必须各测一次：只测一条的话，实现里把另一条写死成 0 也能过。
 */
static void testJamByCurrentSaturationAlone(void)
{
    uint32_t i;
    uint32_t confirmMs;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    for (i = 0U; i < confirmMs + 5U; i++)
    {
        /* errorCode 保持 NONE，只有电流顶满。 */
        feedTriggerJam();
        step(KNOB_FIRE);
    }

    assert(Shooter.jam_state != SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_state != SHOOTER_JAM_SUSPECT);
}

/**
 * @brief 确认卡弹后拨盘【真的反转】：回拨目标正好在反馈角后面 jam_back_rad，请求电流朝退弹方向。
 *
 * 拦"从累加目标往回减"——改之前就是这样：卡住时目标领先反馈一整发，退 0.3 发之后
 * 仍在前面，拨盘照样往前顶（探针实测三次 BACKING 全程 +8 A）。回拨的基准必须是
 * 反馈角，RoboWalker crt_booster.cpp:76、上交 rm_controllers 的 block() 都是这样。
 * 方向都乘了 APP_SHOOT_TRIGGER_DIR，DIR 翻成 -1 时同样成立。
 */
static void testJamBacksOffFromFeedback(void)
{
    uint32_t i;
    uint32_t confirmMs;

    settleIdle();
    confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    /* 单发：目标领先反馈一整发。 */
    feedAllHealthy(0U);
    step(KNOB_FIRE);

    for (i = 0U; i < confirmMs + 5U; i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
        if (Shooter.jam_state == SHOOTER_JAM_BACKING)
        {
            break;
        }
    }
    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);

    assert(fabsf((Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR +
                 Shooter_Config.jam_back_rad) < EPS);
    assert((float)Shooter.trigger_I_req * APP_SHOOT_TRIGGER_DIR < 0.0f);
}

/**
 * @brief 回拨开关关着（机械组看原始卡弹用）：确认卡弹那一拍直接 GIVEUP，停在卡点、不回拨；
 *        按着 VrB 一直停着，松开回待发。摩擦轮照转。
 *
 * 拦"开关不起作用"：关着还在回拨，机械组就看不到弹到底卡在哪。
 * 也拦"关掉就什么都不管"：判到了还一直往前顶，会把弹顶碎、把电机顶热。
 */
static void testJamBackOffDisabledStopsAtJam(void)
{
    uint32_t i;

    settleIdle();
    Shooter.jam_back_flag = 0U;
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);

    /* 卡死到第一次确认为止。 */
    for (i = 0U; (i < 5000U) && ((Shooter.jam_state == SHOOTER_JAM_NORMAL) ||
                                 (Shooter.jam_state == SHOOTER_JAM_SUSPECT));
         i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);
    assert(Shooter.trigger_I_req == 0);
    assert((Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR > 0.0f);
    assert(Shooter.fault == SHOOTER_FAULT_NONE);
    assert(Shooter.I_req[MOTOR_DJI_FRIC_1] != 0);

    /* 按着 VrB：一直停着。 */
    for (i = 0U; i < 200U; i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);
    assert(Shooter.trigger_I_req == 0);

    /* 松开：回待发，被卡的那一发放弃。 */
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);

    Shooter.jam_back_flag = 1U;
}

/**
 * @brief 回拨结束后目标回到【被卡的那一发】重打：这一发不丢，它后面的积压扔掉。
 *
 * 拦两种错：
 *   - 回拨完目标停在退后的位置：被卡的那一发丢了，之后每发都错开一截槽位相位
 *     （RoboWalker 的单发就是这样）；
 *   - 恢复成卡住前的目标：把攒下的积压一口气补打出去（探针 F：脱困后 1 s 打出 38 发）。
 *
 * 期望值手算：连发从待发角 start 起，第一发的终点是 start + 1 发；拨盘转进第一发 30% 处卡住，
 * 在途的就是这一发。
 */
static void testBackingResumesInFlightShot(void)
{
    float start;
    float resumeShift;

    settleIdle();
    start = Shooter.trigger_angle;

    holdBurstWithoutMoving();
    assert((Shooter.trigger_target - start) * APP_SHOOT_TRIGGER_DIR > 3.5f * APP_SHOOT_RAD_PER_AMMO);

    advanceTrigger((int32_t)(0.3f * (float)countsPerAmmo() * APP_SHOOT_TRIGGER_DIR), KNOB_FIRE);
    jamUntil(SHOOTER_JAM_BACKING, 1U, KNOB_FIRE);
    assert(fabsf(Shooter.trigger_resume_target -
                 (start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR)) < EPS);

    /*
     * 拨盘退不动，回拨超时结束，目标回到这一发。同一拍的连发节拍可能刚好再加一发，所以上限给一发余量；
     * 但不能是退后的位置（差 -1 发），也不能是卡住前攒下的那一串（多 3 发以上）。
     */
    jamUntil(SHOOTER_JAM_BACKING, 0U, KNOB_FIRE);
    resumeShift = (Shooter.trigger_target - Shooter.trigger_resume_target) * APP_SHOOT_TRIGGER_DIR;
    assert(resumeShift > -EPS);
    assert(resumeShift < APP_SHOOT_RAD_PER_AMMO + EPS);
}

/**
 * @brief 卡住时拨盘往回让了一点（不到到位容差）：重打的仍是刚按下的那一发。
 *
 * 拦"只按整发数扣"：拨盘比上一个槽位边界还靠后一丝，那条边界就会被当成"还在途"，
 * 刚按下、被卡住的那一发被当成积压扔掉。离目标在容差以内就算转到了，与回拨到位、
 * 重打完成用同一个判据。弹丸顶回来、齿隙都会让拨盘这样往回让一点。
 */
static void testResumeKeepsShotWhenTriggerSlipsBack(void)
{
    float start;
    float slip = 0.5f * Shooter_Config.trigger_arrive_rad;

    settleIdle();
    start = Shooter.trigger_angle;

    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    advanceTrigger(-(int32_t)(slip / (2.0f * APP_PI_F) * (float)TRIGGER_COUNTS_PER_OUTPUT_ROUND *
                              APP_SHOOT_TRIGGER_DIR),
                   KNOB_FIRE);
    jamUntil(SHOOTER_JAM_BACKING, 1U, KNOB_FIRE);

    assert(fabsf(Shooter.trigger_resume_target -
                 (start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR)) < EPS);
}

/**
 * @brief 拨盘真退到了回拨目标，回拨立刻结束、回去重打，不等满 jam_back_s。
 *
 * 拦"只按时长结束"（RoboWalker 固定 300 ms）：弹已经松开了，拨盘还停在后面往回顶，
 * 白等一段、多吃一段反向力矩。退不动的情况由超时兜底，见 testBackingResumesInFlightShot。
 * 判据取 jam_back_s 的一半：超时本身不可能让这条通过。
 */
static void testBackingEndsOnArrival(void)
{
    uint32_t backingMs;

    settleIdle();
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    jamUntil(SHOOTER_JAM_BACKING, 1U, KNOB_FIRE);

    backingMs = reverseUntilBackingEnds();

    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert((float)backingMs * TICK_S < 0.5f * Shooter_Config.jam_back_s);
}

/**
 * @brief 回拨后重打的那一发打完了，连续卡弹计数清零——这一轮卡弹算解决了。
 *
 * 拦"计数只在松开 VrB 时清"：一次长连发里先后卡三次、每次都回拨解决了，
 * 第三次照样进 GIVEUP 把拨盘封掉。GIVEUP 只该给"连续回拨都没打出去"用。
 */
static void testRetryClearedWhenShotCompletes(void)
{
    float start;

    settleIdle();
    start = Shooter.trigger_angle;
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    jamUntil(SHOOTER_JAM_BACKING, 1U, KNOB_FIRE);
    reverseUntilBackingEnds();
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_retry == 1U);

    /* 拨盘转回去，把被卡的那一发（终点 start + 1 发）打完。 */
    advanceTrigger(countsOfRad((start + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR -
                                Shooter.trigger_angle)),
                   KNOB_FIRE);
    assert(Shooter.jam_retry == 0U);
}

/**
 * @brief 疑似卡弹（SUSPECT）期间连发节拍跳过不补：一发都不加，回落之后也不补打。
 *
 * 拦两种错：
 *   - SUSPECT 期间照加：卡弹在确认之前自己松开时，这段时间攒的发会按 outputLimit
 *     一口气补打出去（探针 F：攒了 21 发，脱困后 1 s 打出 38 发）；
 *   - 只是不加、节拍却没消耗：回落之后欠下的节拍每拍补一发，照样是一串。
 */
static void testSuspectSkipsBurstBeats(void)
{
    uint32_t period = 1000U / APP_SHOOT_BURST_HZ;
    uint32_t window = (uint32_t)(0.8f * Shooter_Config.jam_confirm_s * 1000.0f);
    float before;
    uint32_t i;

    assert(window >= 2U * period); /* 窗口里至少落两个节拍，否则这条测不出东西 */

    settleIdle();
    for (i = 0U; i < APP_SHOOT_BURST_DELAY_MS + period; i++)
    {
        feedAllHealthy(Shooter.trigger_motor.encoderRaw);
        step(KNOB_FIRE);
    }
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_BURST);

    before = Shooter.trigger_target;
    for (i = 0U; i < window; i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_SUSPECT);
    assert(fabsf(Shooter.trigger_target - before) < EPS);

    /* 电流回落：节拍照常往下走，一个节拍之内最多加一发。 */
    before = Shooter.trigger_target;
    for (i = 0U; i < period - 1U; i++)
    {
        feedAllHealthy(Shooter.trigger_motor.encoderRaw);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert((Shooter.trigger_target - before) * APP_SHOOT_TRIGGER_DIR <
           1.5f * APP_SHOOT_RAD_PER_AMMO);
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
        feedTriggerJam();
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
        feedTriggerJam();
        step(KNOB_FIRE);
    }
    assert(fabsf(Shooter.trigger_target - targetAtBacking) < EPS);
}

/**
 * @brief 连续退弹 jam_retry_max 次仍未脱困 -> GIVEUP，拨盘电流清零，摩擦轮照转。
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
        feedTriggerJam();
        step(KNOB_FIRE);
        if (Shooter.jam_state == SHOOTER_JAM_GIVEUP)
        {
            break;
        }
    }

    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);
    assert(Shooter.jam_retry >= Shooter_Config.jam_retry_max);
    assert(Shooter.trigger_I_req == 0);
    /*
     * ⚠ GIVEUP 故意【不】置 fault：它只封锁拨盘的请求电流，摩擦轮照转。
     * 这条断言钉死"只封锁拨盘"。
     */
    assert(Shooter.fault == SHOOTER_FAULT_NONE);
    assert(Shooter.I_req[MOTOR_DJI_FRIC_1] != 0);
}

/**
 * @brief GIVEUP 只能靠 VrB 退出使能区解除，卡弹消失本身解不开。
 */
static void testGiveUpOnlyClearedByRelease(void)
{
    uint32_t i;

    settleIdle();
    for (i = 0U; i < 20000U; i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
        if (Shooter.jam_state == SHOOTER_JAM_GIVEUP)
        {
            break;
        }
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);

    /* 卡弹消失，但 VrB 还按着：不能自己解除。 */
    for (i = 0U; i < 2000U; i++)
    {
        feedAllHealthy(0U);
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_GIVEUP);

    /*
     * 松开 VrB 才解除，而且被卡的那一发一起放弃（回待发）。
     * 拦"只复位卡弹机"：松开在拨弹那边是"收尾、把在途那一发打完"，拨盘会立刻顶回那颗卡死的弹。
     */
    feedAllHealthy(0U);
    step(KNOB_OFF);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.jam_retry == 0U);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
}

/**
 * @brief 松开 VrB 之后（收尾中）卡住：照样从反馈角回拨，退完回去重打这一发，仍在收尾。
 *
 * 拦两种错：卡弹处理只在按着 VrB 时才跑（收尾那一发卡住没人管，拨盘顶着限幅一直推）；
 * 以及"已经松手了，回拨完就算了"——这一发被放弃，拨盘停在槽位中间，正是收尾要避免的。
 */
static void testJamWhileFinishingBacksOff(void)
{
    float shotEnd;

    settleIdle();
    shotEnd = Shooter.trigger_angle + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR;
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);

    jamUntil(SHOOTER_JAM_BACKING, 1U, KNOB_OFF);
    assert(fabsf((Shooter.trigger_target - Shooter.trigger_angle) * APP_SHOOT_TRIGGER_DIR +
                 Shooter_Config.jam_back_rad) < EPS);
    assert((float)Shooter.trigger_I_req * APP_SHOOT_TRIGGER_DIR < 0.0f);

    /* 退不动，超时结束：回去重打这一发，还在收尾。 */
    jamUntil(SHOOTER_JAM_BACKING, 0U, KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);
    assert(fabsf(Shooter.trigger_target - shotEnd) < EPS);
}

/**
 * @brief 收尾中一直卡死到 GIVEUP：VrB 本来就松着，下一拍就解除并回待发，不再往前顶。
 *
 * 拦"GIVEUP 出口只复位卡弹机"：收尾那边还留着被卡的那一发，GIVEUP 一解除拨盘又顶上去——
 * 判卡弹、回拨、GIVEUP、解除，没完没了；操作者早松手了，拨盘还在跟那颗弹较劲。
 */
static void testGiveUpAfterReleaseGoesIdle(void)
{
    uint32_t i;

    settleIdle();
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_FINISH);

    jamUntil(SHOOTER_JAM_GIVEUP, 1U, KNOB_OFF);

    /* 电流不再顶着（实机上电调关了输出、或者命令已经是 0）：待发、不再推。 */
    for (i = 0U; i < 100U; i++)
    {
        feedAllHealthy(Shooter.trigger_motor.encoderRaw);
        step(KNOB_OFF);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
}

/**
 * @brief 摩擦轮由 VrB 过中位驱动：不请求时静止的摩擦轮不出力。
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
        feedTriggerJam();
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
    assert(Shooter.trigger_I_req == 0);

    /* 打一发，拨盘不动，于是位置误差就是一整发。 */
    feedAllHealthy(0U);
    step(KNOB_FIRE);

    if (APP_SHOOT_TRIGGER_DIR > 0.0f)
    {
        assert(Shooter.trigger_target_omega > 0.0f);
        assert(Shooter.trigger_I_req > 0);
    }
    else
    {
        assert(Shooter.trigger_target_omega < 0.0f);
        assert(Shooter.trigger_I_req < 0);
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
 * ⚠ 漏除减速比会差 36 倍（M2006）或 10 倍（翎控）；翎控还有 dps 与 rpm 的单位差。
 * 前面那些用例里拨盘一动不动、转速恒为 0，单位错了也看不出来——必须专门喂一个非零转速。
 */
static void testTriggerFeedbackUnits(void)
{
    settleIdle();

    feedFricHealthy();
    feedTrigger(1000U, 3600, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);

    /* 反馈角必须就是设备层那份，不能是别处算的第二份。 */
    assert(fabsf(Shooter.trigger_angle - Shooter.trigger_motor.angle) < EPS);

#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    /* 翎控：电机端 3600 dps -> 输出轴 3600/10 = 360 dps = 2π rad/s。 */
    assert(fabsf(Shooter.trigger_omega - 6.2832f) < 1e-3f);
#else
    /* M2006 减速箱还在：转子 3600 rpm -> 输出轴 3600/36 = 100 rpm = 10.472 rad/s。 */
    assert(fabsf(Shooter.trigger_omega - 10.4720f) < 1e-3f);
#endif
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
        (int32_t)((float)TRIGGER_COUNTS_PER_OUTPUT_ROUND / (float)APP_SHOOT_AMMO_PER_ROUND);
    float target;

    settleIdle();

    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_FIRE);
    target = Shooter.trigger_target;
    assert(Shooter.trigger_I_req != 0);

    /*
     * 保持按住、让它转到位（途中松开的收尾另测，见 testReleaseFinishesInFlightShot）。
     * ⚠ 转的方向要跟着 APP_SHOOT_TRIGGER_DIR，否则把宏翻成 -1 时
     * 这条用例会朝着离目标更远的方向转。
     */
    advanceTrigger((int32_t)((float)countsPerAmmo * APP_SHOOT_TRIGGER_DIR),
                   KNOB_FIRE);

    /* 目标没变（还没到连发延迟），而反馈已经追上来了。 */
    assert(fabsf(Shooter.trigger_target - target) < EPS);
    assert(fabsf(Shooter.trigger_angle - target) < 0.05f);

    /*
     * 到位之后再松开：没有在途的了，下一拍就回待发，一发都不多打。
     * 拦"数在途那一发时没有下限"：领先量为 0 时会数出 -1 发，凭空多出一发。
     */
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    feedAllHealthy(Shooter.trigger_motor.encoderRaw);
    step(KNOB_OFF);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
}

/* ---- 停止语义：急停、遥控离线、关摩擦轮都是 PID 刹车，故障才清零 ---- */

/** @brief 摩擦轮和拨盘都转起来：VrB 在拨弹区，摩擦轮 3000 rpm。 */
static void spinUp(void)
{
    uint32_t i;

    settleIdle();
    for (i = 0U; i < 10U; i++)
    {
        feedFric(3000);
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
        step(KNOB_FIRE);
    }
    assert(Shooter.fric_request == 1U);
}

/**
 * @brief VrB 拧回下半程：摩擦轮用速度环刹到 0。
 *
 * 拦三种写法：
 *   - 原来的"电流给 0 滑停"——刹车电流为 0；
 *   - 固定刹车电流——转速减半电流不减半，停稳后还在推，会把轮子反向拖起来；
 *   - 刹车时积分不清——长时间刹车灌满的积分在停稳后仍然往反方向推。
 */
static void testFricOffBrakesWithPid(void)
{
    int16_t atHigh;
    int16_t atLow;
    uint32_t i;

    settleIdle();

    feedFric(3000);
    feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);
    assert(Shooter.fric_request == 0U);
    atHigh = Shooter.I_req[MOTOR_DJI_FRIC_1];
    assert(atHigh < 0); /* 与正转速反向 */

    feedFric(1500);
    feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);
    atLow = Shooter.I_req[MOTOR_DJI_FRIC_1];
    /* 转速减半，刹车电流也减半：是比例刹车，不是固定值。 */
    assert(fabsf(((float)atLow / (float)atHigh) - 0.5f) < 0.05f);
    for (i = 0U; i < APP_DJI_FRIC_COUNT; i++)
    {
        assert(Shooter.I_req[i] == atLow);
    }

    /* 长时间顶着转速刹，再让它停稳：一点都不许再推。 */
    for (i = 0U; i < 1000U; i++)
    {
        feedFric(3000);
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
        step(KNOB_OFF);
    }
    feedFric(0);
    feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);
    for (i = 0U; i < APP_DJI_FRIC_COUNT; i++)
    {
        assert(Shooter.I_req[i] == 0);
    }

    /* 刹车电流真的发出去了（门开着的变体里）。 */
    if (fricGateOpen() != 0U)
    {
        feedFric(3000);
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE);
        step(KNOB_OFF);
        assert(Shooter.safe_flag == 0U);
        assert(Shooter.I[MOTOR_DJI_FRIC_1] == Shooter.I_req[MOTOR_DJI_FRIC_1]);
        assert(Shooter.I[MOTOR_DJI_FRIC_1] < 0);
    }
}

/** @brief 急停/离线后的共同断言：请求清零、摩擦轮在刹、拨盘最终电流为 0、安全门不关。 */
static void assertStopped(void)
{
    uint32_t i;

    assert(Shooter.stop_flag == 1U);
    assert(Shooter.fric_request == 0U);
    assert(Shooter.trigger_request == 0U);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    for (i = 0U; i < APP_DJI_FRIC_COUNT; i++)
    {
        assert(Shooter.I_req[i] < 0); /* 摩擦轮正转，刹车电流为负 */
    }
    /* 门不能关：摩擦轮要靠它把刹车电流发出去，关了就只能滑停。 */
    assert(Shooter.safe_flag == 0U);
    if (fricGateOpen() != 0U)
    {
        assert(Shooter.I[MOTOR_DJI_FRIC_1] == Shooter.I_req[MOTOR_DJI_FRIC_1]);
    }
    /*
     * 拨盘不刹车，最终电流直接为 0：翎控在 0xA1 转矩模式下会一直执行收到的最后一个 iq，
     * 云台板一停发（烧录、死机），刹车电流就变成开环的固定电流把拨盘拖着转
     * （2026-09-30 烧录时实机遇到拨盘疯转）。急停时发 0，最后一帧就是 0。
     * 先断言刹车请求不为 0（拨盘正在转），下面那条"为 0"才不是请求本来就是 0 的空过。
     */
    assert(Shooter.trigger_I_req < 0);
    assert(Shooter.trigger_I == 0);
}

/**
 * @brief 急停（SwC=UP）：VrB 还拧在拨弹区也不许打；摩擦轮走 PID 刹车，拨盘电流清零。
 *
 * 拦"发射机构不看 SwC"——改之前的代码就是这样，探针实测急停时四路照常全力输出
 * 5001/5001/5001/10000。遥控离线和急停必须同时停两部分，这是红线。
 */
static void testEstopBrakesShooter(void)
{
    spinUp();

    remoteEstop(KNOB_FIRE);
    feedFric(3000);
    feedTrigger(0U, 3600, 0, MOTOR_DJI_ERR_NONE);
    stepRemoteDt(TICK_S);

    assertStopped();
}

/**
 * @brief 遥控离线：与急停同样处理，判定看的是 online，不是碰巧回中的拨杆。
 *
 * 两种快照都要停：task_remote 实际给的全零快照，以及留着旧值、只有 online=0 的快照。
 * 后者拦"只看 SwC 不看 online"。
 */
static void testRemoteOfflineBrakes(void)
{
    spinUp();

    remoteOfflineStale(KNOB_FIRE);
    feedFric(3000);
    feedTrigger(0U, 3600, 0, MOTOR_DJI_ERR_NONE);
    stepRemoteDt(TICK_S);
    assertStopped();

    spinUp();
    memset(&stubRemote, 0, sizeof(stubRemote));
    feedFric(3000);
    feedTrigger(0U, 3600, 0, MOTOR_DJI_ERR_NONE);
    stepRemoteDt(TICK_S);
    assertStopped();
}

/**
 * @brief 急停打断正在进行的退弹：两层状态机复位，拨盘只刹不退。
 *
 * 拦"急停时卡弹状态机照跑"——BACKING 会继续把拨盘往回拖 300 ms。
 */
static void testEstopCancelsBackOff(void)
{
    uint32_t i;
    uint32_t confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    settleIdle();
    for (i = 0U; i < confirmMs + 5U; i++)
    {
        feedTriggerJam();
        step(KNOB_FIRE);
    }
    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);
    /* 退弹目标此时不在反馈角上，照跑的话拨盘会被拖着走。 */
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) > EPS);

    remoteEstop(KNOB_FIRE);
    feedAllHealthy(0U);
    stepRemoteDt(TICK_S);

    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);
    assert(Shooter.trigger_I_req == 0); /* 拨盘静止，刹车电流为 0 */
}

/**
 * @brief 急停时一台摩擦轮离线：离线那台清零，其余两台照样刹车。
 *
 * 离线电机的转速是旧值，拿它算"刹车"就等于一个固定电流，会把它往反方向拖，
 * 所以它自己的命令必须清零；但它不牵连别的电机——另外两台的反馈是新的，照刹。
 */
static void testOfflineMotorNotBraked(void)
{
    uint32_t i;

    if (fricGateOpen() == 0U)
    {
        return; /* 摩擦轮门关着，它离线本来就不计入故障 */
    }

    spinUp();
    for (i = 0U; i < APP_DJI_TIMEOUT_TICKS + 5U; i++)
    {
        remoteEstop(KNOB_FIRE);
        feedDji(APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2, 0U, 3000, 0, MOTOR_DJI_ERR_NONE);
        feedDji(APP_DJI_FRIC_3_RX_ID, APP_CAN_BUS_FDCAN2, 0U, 3000, 0, MOTOR_DJI_ERR_NONE);
        feedTrigger(0U, 3600, 0, MOTOR_DJI_ERR_NONE); /* 摩擦轮 2 不喂 */
        stepRemoteDt(TICK_S);
    }

    assert((Shooter.fric_fault[MOTOR_DJI_FRIC_2] & SHOOTER_FAULT_MOTOR) != 0U);
    assert(Shooter.I_req[MOTOR_DJI_FRIC_2] < 0); /* 请求照算，供 Watch 观察 */
    assert(Shooter.I[MOTOR_DJI_FRIC_2] == 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_1] < 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_3] < 0);
    /* 急停时拨盘不刹，最终电流为 0。 */
    assert(Shooter.trigger_I == 0);
}

/* ---- 发射部分按台隔离：哪台离线/报错只停哪台，互不牵连 ---- */

#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
/**
 * @brief 拨盘报堵转码：交给卡弹状态机，不算电调故障——摩擦轮照转，拨盘仍受串级环控制。
 *
 * 拦"任何非零错误码都计入 SHOOTER_FAULT_ESC"——改之前就是这样：拨盘一卡，
 * 安全门把发射部分全关了，摩擦轮停转，BACKING 算出来的电流也发不出去。
 * 门关着的变体里 Shooter.I 恒为 0，这条只能在门开着时断言实际下发量。
 */
static void testTriggerStallKeepsFriction(void)
{
    uint32_t i;
    uint32_t confirmMs = (uint32_t)(Shooter_Config.jam_confirm_s * 1000.0f);

    settleIdle();
    for (i = 0U; i < confirmMs + 20U; i++)
    {
        feedFric(3000);
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_STALL);
        step(KNOB_FIRE);
    }

    assert(Shooter.jam_state == SHOOTER_JAM_BACKING);
    assert(Shooter.fault == SHOOTER_FAULT_NONE);
    assert(Shooter.safe_flag == 0U);
    if (fricGateOpen() != 0U)
    {
        assert(Shooter.I[MOTOR_DJI_FRIC_1] > 0); /* 3000 rpm 追 5000 rpm，还在加速 */
    }
    /* 拨盘这一路照样在执行回拨：命令朝退弹方向，而且真的发出去了。 */
    assert((float)Shooter.trigger_I_req * APP_SHOOT_TRIGGER_DIR < 0.0f);
    if (triggerGateOpen() != 0U)
    {
        assert(Shooter.trigger_I == Shooter.trigger_I_req);
    }
}

/**
 * @brief 拨盘报堵转以外的错误码（三相未接）：只停拨盘，摩擦轮照转。
 *
 * 拦两头：堵转排除写得太宽（拨盘所有错误码都放过）；以及又退回"一台报错停整个部分"。
 */
static void testTriggerEscFaultStopsOnlyTrigger(void)
{
    if ((triggerGateOpen() == 0U) || (fricGateOpen() == 0U))
    {
        return;
    }

    spinUp();
    feedFric(3000);
    feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_PHASE_OPEN);
    step(KNOB_FIRE);

    assert((Shooter.trigger_fault & SHOOTER_FAULT_ESC) != 0U);
    assert(Shooter.trigger_I == 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_1] > 0);
}
#endif

/** @brief 拨盘离线：只停拨盘，摩擦轮照常出力。 */
static void testTriggerOfflineStopsOnlyTrigger(void)
{
    uint32_t i;

    if ((triggerGateOpen() == 0U) || (fricGateOpen() == 0U))
    {
        return;
    }

    spinUp();
    for (i = 0U; i < APP_DJI_TIMEOUT_TICKS + 5U; i++)
    {
        feedFric(3000); /* 拨盘一帧都不喂 */
        step(KNOB_FIRE);
    }

    assert((Shooter.trigger_fault & SHOOTER_FAULT_MOTOR) != 0U);
    assert(Shooter.trigger_I == 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_1] > 0); /* 3000 rpm 追 5000 rpm */
}

/** @brief 一台摩擦轮离线（运行中、没有急停）：只停它自己，另两台和拨盘照常。 */
static void testFricOfflineStopsOnlyItself(void)
{
    uint32_t i;

    if (fricGateOpen() == 0U)
    {
        return;
    }

    spinUp();
    for (i = 0U; i < APP_DJI_TIMEOUT_TICKS + 5U; i++)
    {
        feedDji(APP_DJI_FRIC_1_RX_ID, APP_CAN_BUS_FDCAN2, 0U, 3000, 0, MOTOR_DJI_ERR_NONE);
        feedDji(APP_DJI_FRIC_2_RX_ID, APP_CAN_BUS_FDCAN2, 0U, 3000, 0, MOTOR_DJI_ERR_NONE);
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE); /* 摩擦轮 3 不喂 */
        step(KNOB_FIRE);
    }

    assert((Shooter.fric_fault[MOTOR_DJI_FRIC_3] & SHOOTER_FAULT_MOTOR) != 0U);
    assert(Shooter.I[MOTOR_DJI_FRIC_3] == 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_1] > 0);
    assert(Shooter.I[MOTOR_DJI_FRIC_2] > 0);
    if (triggerGateOpen() != 0U)
    {
        assert(Shooter.trigger_I != 0); /* 拨盘还在打那一发 */
    }
}

/**
 * @brief 分步装机：摩擦轮门关着时，摩擦轮离线不计入故障，拨盘照常出力。
 *
 * 只在"摩擦轮门关、拨盘门开"的变体里有意义。拦"未使能总线也计入故障"——
 * 那样只装了拨盘时 fault 永远非零，发射机构永远不出力。
 */
static void testUnenabledBusNotCounted(void)
{
    uint32_t i;

    if ((fricGateOpen() != 0U) || (triggerGateOpen() == 0U))
    {
        return;
    }

    settleIdle();
    for (i = 0U; i < APP_DJI_TIMEOUT_TICKS + 5U; i++)
    {
        feedTrigger(0U, 0, 0, MOTOR_DJI_ERR_NONE); /* 摩擦轮一帧都不喂 */
        step(KNOB_FIRE);
    }

    assert(Shooter.fault == SHOOTER_FAULT_NONE);
    assert(Shooter.safe_flag == 0U);
    assert(Shooter.trigger_I != 0);
}

/* ---- 拨盘力矩限幅：防止把弹丸搅碎 ---- */

/**
 * @brief 速度环输出之后硬限幅；运行中改 Shooter.trigger_I_limit 立即生效；刹车也受它约束。
 *
 * 拦三种错：没有限幅（卡住时顶到电调满量程）；限幅只读 const 配置、
 * Watch 里改了不生效；急停刹车绕过限幅。
 */
static void testTriggerTorqueLimit(void)
{
    float saved = Shooter.trigger_I_limit;

    settleIdle();

    /* 一整发的位置误差，速度环会远远顶过 2 A。 */
    Shooter.trigger_I_limit = 2.0f;
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(Shooter.trigger_I_req == (int16_t)(2.0f * TRIGGER_RAW_PER_A * APP_SHOOT_TRIGGER_DIR));

    /* 运行中改小，下一拍就按新值夹。 */
    Shooter.trigger_I_limit = 1.0f;
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(Shooter.trigger_I_req == (int16_t)(1.0f * TRIGGER_RAW_PER_A * APP_SHOOT_TRIGGER_DIR));

    /*
     * 待发态拨盘还在高速正转：刹车电流同样只到 1 A，而且照发。
     * （急停时拨盘不刹、直接清零，所以这条用正常待发来测。
     *  ⚠ 不能用"打一发再松开"进待发：松开之后是收尾，在途那一发要打完，拨盘在往前追、不是刹车。）
     */
    settleIdle();
    feedFricHealthy();
    feedTrigger(0U, 3600, 0, MOTOR_DJI_ERR_NONE);
    step(KNOB_OFF);
    assert(Shooter.trigger_I_req == (int16_t)(-1.0f * TRIGGER_RAW_PER_A));
    assert(Shooter.trigger_I == Shooter.trigger_I_req);

    Shooter.trigger_I_limit = saved;
}

/**
 * @brief 卡弹的"电流饱和"判据跟着力矩限幅走。
 *
 * 拦阈值写死成电调满量程的 95%（改之前是 9500）：限幅一调到 2 A，
 * 反馈电流永远到不了 9.5 A，这条判据就形同虚设。
 */
static void testJamThresholdFollowsLimit(void)
{
    float saved = Shooter.trigger_I_limit;

    Shooter.trigger_I_limit = 2.0f;

    /* 顶到限幅的 97.5%：判卡弹。 */
    settleIdle();
    feedFricHealthy();
    feedTrigger(0U, 0, (int16_t)(0.975f * 2.0f * TRIGGER_RAW_PER_A), MOTOR_DJI_ERR_NONE);
    step(KNOB_FIRE);
    assert(Shooter.jam_state == SHOOTER_JAM_SUSPECT);

    /* 只有限幅的一半：正常推弹，不判。 */
    settleIdle();
    feedFricHealthy();
    feedTrigger(0U, 0, (int16_t)(0.5f * 2.0f * TRIGGER_RAW_PER_A), MOTOR_DJI_ERR_NONE);
    step(KNOB_FIRE);
    assert(Shooter.jam_state == SHOOTER_JAM_NORMAL);

    Shooter.trigger_I_limit = saved;
}

/* ---- 上膛锁：急停/离线解除后，VrB 必须先退出拨弹区才再打 ---- */

/**
 * @brief 急停解除时 VrB 还在拨弹区：一发都不打；退出拨弹区再进入，正好一发。
 *
 * 拦"一松急停就开火"——那时摩擦轮刚从 0 起转，立刻拨弹还容易卡弹。
 * 只锁拨盘：摩擦轮照常起转。
 */
static void testRearmAfterStop(void)
{
    float before;

    settleIdle();

    /* 急停中把 VrB 拧到底。 */
    remoteEstop(KNOB_FIRE);
    feedAllHealthy(0U);
    stepRemoteDt(TICK_S);

    /* 解除急停，VrB 还在拨弹区。 */
    before = Shooter.trigger_target;
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(Shooter.trigger_request == 0U);
    assert(fabsf(Shooter.trigger_target - before) < EPS);
    assert(Shooter.fric_request == 1U);

    /* 退出拨弹区（仍在摩擦轮区），再进入：正好一发。 */
    feedAllHealthy(0U);
    step(0.5f);
    before = Shooter.trigger_target;
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(fabsf((Shooter.trigger_target - before) -
                 APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) < EPS);
}

/**
 * @brief 上电就是锁着的：遥控一上线 VrB 就在拨弹区，也不许打。
 *
 * 拦锁存初值为 0——那样复位后第一拍就可能开火，全靠遥控同步那几帧离线碰巧先把它锁上。
 */
static void testLockedAtPowerUp(void)
{
    float before;

    initShooter();

    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(Shooter.trigger_request == 0U);
    /*
     * "没打"看目标角是否等于反馈角（待发态目标跟随反馈），不能跟 Init 之后的 0 比：
     * 设备层的累计角是上电以来的，不随 Shooter_Init 清零。
     */
    assert(Shooter.trigger_state == SHOOTER_TRIGGER_IDLE);
    assert(fabsf(Shooter.trigger_target - Shooter.trigger_angle) < EPS);

    feedAllHealthy(0U);
    step(KNOB_OFF);
    before = Shooter.trigger_target;
    feedAllHealthy(0U);
    step(KNOB_FIRE);
    assert(fabsf((Shooter.trigger_target - before) -
                 APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR) < EPS);
}

/**
 * @brief 上电后第一发从拨盘【当前角】起算：上电头几拍遥控还没上线（等同急停），
 *        这期间"上一发终点"一直跟着当前角。
 *
 * 拦"放弃（急停/离线）时不更新上一发终点"：上电时它是 0，可 M2006 的上电角取决于转子编码器读数，
 * 离 0 在容差以内时，第一发就会从一个从没打到过的 0 起算，少转一截。
 * ⚠ 要排在 testLockedAtPowerUp 之后：这条把拨盘停在 0 附近，而那条 Init 之后没有离线拍。
 */
static void testFirstShotAfterBootFromAngle(void)
{
    float before;
    uint16_t raw;

    settleIdle();
    advanceTrigger(countsOfRad(0.5f * Shooter_Config.trigger_arrive_rad * APP_SHOOT_TRIGGER_DIR -
                               Shooter.trigger_angle),
                   KNOB_OFF);
    /* ⚠ 先记下编码器读数：Init 会把 Shooter 里的反馈快照一起清零，再拿快照喂就成了 0。 */
    raw = Shooter.trigger_motor.encoderRaw;
    initShooter();

    memset(&stubRemote, 0, sizeof(stubRemote)); /* 遥控还没上线 */
    feedAllHealthy(raw);
    stepRemoteDt(TICK_S);
    feedAllHealthy(raw);
    step(KNOB_OFF); /* 上线、VrB 在不使能区：解开上电上膛锁 */

    before = Shooter.trigger_angle;
    assert(fabsf(before) > EPS); /* 拨盘确实停在 0 附近、又不正好是 0，否则测不出东西 */
    feedAllHealthy(raw);
    step(KNOB_FIRE);
    assert(fabsf(Shooter.trigger_target - (before + APP_SHOOT_RAD_PER_AMMO * APP_SHOOT_TRIGGER_DIR)) <
           EPS);
}

/**
 * @brief 只发选中的那台拨盘的帧：另一台的帧在 FDCAN3 上一帧都不能有。
 *
 * FDCAN3 是板间线，负载已经 71%~86%。拦"选了翎控却还在发 C610 的 0x200"，以及反过来。
 */
static void testOnlyChosenTriggerTransmits(void)
{
    if (triggerGateOpen() == 0U)
    {
        return;
    }

    txDjiTriggerCount = 0U;
    txLkTriggerCount = 0U;
    feedAllHealthy(0U);
    step(KNOB_OFF);

#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    assert(txLkTriggerCount == 1U);
    assert(txDjiTriggerCount == 0U);
#else
    assert(txDjiTriggerCount == 1U);
    assert(txLkTriggerCount == 0U);
#endif
}

int main(void)
{
    /* 先核对这一遍的输出门真的是声明的那样，理由见文件头：sed 没匹配上就是假通过。 */
    assert(Shooter_Config.output.fric_flag == TEST_FRIC_FLAG);
    assert(Shooter_Config.output.trigger_flag == TEST_TRIGGER_FLAG);

    initShooter();

    testSingleShotOnRisingEdge();
    testHoldBelowDelayStaysOneShot();
    testBurstAfterDelay();
    testReleaseFinishesInFlightShot();
    testNextShotFromLastTarget();
    testReenterStartsFromSingle();
    testTargetAccumulatesFromTargetNotFeedback();
    testTriggerFeedbackUnits();
    testCascadeDrivesTowardTarget();
    testShotCompletesWhenTriggerTurns();
    testBriefSpikeDoesNotJam();
#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
    testJamByErrorCodeBacksOff();
#endif
    testJamByCurrentSaturationAlone();
    testJamBacksOffFromFeedback();
    testJamBackOffDisabledStopsAtJam();
    testBackingResumesInFlightShot();
    testResumeKeepsShotWhenTriggerSlipsBack();
    testBackingEndsOnArrival();
    testRetryClearedWhenShotCompletes();
    testSuspectSkipsBurstBeats();
    testTriggerBypassedWhileBacking();
    testGiveUpAfterRetries();
    testGiveUpOnlyClearedByRelease();
    testJamWhileFinishingBacksOff();
    testGiveUpAfterReleaseGoesIdle();
    testFrictionFollowsRequest();
    testDirectionConstantIsApplied();
    testBurstRateIndependentOfDt();

    testFricOffBrakesWithPid();
    testEstopBrakesShooter();
    testRemoteOfflineBrakes();
    testEstopCancelsBackOff();
    testOfflineMotorNotBraked();

#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
    testTriggerStallKeepsFriction();
    testTriggerEscFaultStopsOnlyTrigger();
#endif
    testTriggerOfflineStopsOnlyTrigger();
    testFricOfflineStopsOnlyItself();
    testUnenabledBusNotCounted();

    testTriggerTorqueLimit();
    testJamThresholdFollowsLimit();

    testRearmAfterStop();
    testLockedAtPowerUp();
    testFirstShotAfterBootFromAngle();

    testOnlyChosenTriggerTransmits();

#if APP_TRIGGER_MOTOR == APP_TRIGGER_M2006
    printf("test_shooter[M2006]: 44 组用例全过\n");
#else
    printf("test_shooter[LK4005]: 41 组用例全过\n");
#endif
    return 0;
}
