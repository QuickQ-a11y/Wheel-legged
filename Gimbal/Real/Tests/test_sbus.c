/*
 * S.BUS 解析层单测。
 *
 * 编译（在 Gimbal/Real/ 下执行）：
 *   gcc -Wall -Wextra -IUser_File/1_Middleware/0_Common \
 *       -IUser_File/2_Device/Communication/FlySky \
 *       -o t Tests/test_sbus.c \
 *       User_File/2_Device/Communication/FlySky/device_sbus.c -lm
 *
 * 只需要 remote_input.h，不碰 HAL。
 */
#include "device_sbus.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-3f

/*
 * 把 16 路通道打包成一帧。
 *
 * ⚠ 这里【故意】用逐位写入，而不是复用 device_sbus.c 里那个窗口移位公式。
 * 打包和解包共用同一条公式的话，公式本身错了也能往返自洽，等于没测。
 */
static void testBuildFrame(const uint16_t channels[SBUS_CH_COUNT],
                           uint8_t flags,
                           uint8_t footer,
                           uint8_t frame[SBUS_FRAME_LEN])
{
    uint32_t channelIndex;
    uint32_t bitIndex;

    memset(frame, 0, SBUS_FRAME_LEN);
    frame[0] = SBUS_HEADER;

    for (channelIndex = 0U; channelIndex < SBUS_CH_COUNT; channelIndex++)
    {
        for (bitIndex = 0U; bitIndex < 11U; bitIndex++)
        {
            if (((channels[channelIndex] >> bitIndex) & 1U) != 0U)
            {
                uint32_t bitPos = (channelIndex * 11U) + bitIndex;

                frame[1U + (bitPos / 8U)] |= (uint8_t)(1U << (bitPos % 8U));
            }
        }
    }

    frame[23] = flags;
    frame[24] = footer;
}

static void testFillMid(uint16_t channels[SBUS_CH_COUNT])
{
    uint32_t index;

    for (index = 0U; index < SBUS_CH_COUNT; index++)
    {
        channels[index] = SBUS_CH_MID;
    }
}

/** @brief 正常帧能完整还原 16 路通道。 */
static void testValidFrame(void)
{
    uint16_t channels[SBUS_CH_COUNT];
    uint8_t frame[SBUS_FRAME_LEN];
    sbus_data_t data;
    uint32_t index;

    /* 每路给一个互不相同的值，铺满 11 位量程。 */
    for (index = 0U; index < SBUS_CH_COUNT; index++)
    {
        channels[index] = (uint16_t)(100U + (index * 107U));
    }
    testBuildFrame(channels, 0U, 0x00U, frame);

    assert(SBUS_ParseFrame(frame, &data) == 1U);
    for (index = 0U; index < SBUS_CH_COUNT; index++)
    {
        assert(data.channel[index] == channels[index]);
    }
    assert(data.frameLost == 0U);
    assert(data.failsafe == 0U);
}

/**
 * @brief 位域互不串扰：一次只把一路设成特征值，其余全 0。
 *
 * ⚠ 这是本文件最重要的一条。移位算错时，"所有通道设成同一个值再往返比较"
 * 照样能过（错位读到的还是同一个值），只有逐路隔离才抓得到。
 * 特征值取 0x7FF / 0x555 / 0x2AA：全 1、交替 01、交替 10，
 * 相邻位错一位就会读出不同的数。
 */
static void testBitPackingIsolation(void)
{
    static const uint16_t patterns[] = {0x7FFU, 0x555U, 0x2AAU};
    uint32_t patternIndex;
    uint32_t slot;

    for (patternIndex = 0U;
         patternIndex < (sizeof(patterns) / sizeof(patterns[0]));
         patternIndex++)
    {
        for (slot = 0U; slot < SBUS_CH_COUNT; slot++)
        {
            uint16_t channels[SBUS_CH_COUNT] = {0};
            uint8_t frame[SBUS_FRAME_LEN];
            sbus_data_t data;
            uint32_t index;

            channels[slot] = patterns[patternIndex];
            testBuildFrame(channels, 0U, 0x00U, frame);
            assert(SBUS_ParseFrame(frame, &data) == 1U);

            for (index = 0U; index < SBUS_CH_COUNT; index++)
            {
                assert(data.channel[index] ==
                       ((index == slot) ? patterns[patternIndex] : 0U));
            }
        }
    }
}

/** @brief 帧头不对必须拒收；帧尾的各种变体都必须接受。 */
static void testHeaderAndFooter(void)
{
    uint16_t channels[SBUS_CH_COUNT];
    uint8_t frame[SBUS_FRAME_LEN];
    sbus_data_t data;

    testFillMid(channels);

    /* 帧头错 -> 拒收。 */
    testBuildFrame(channels, 0U, 0x00U, frame);
    frame[0] = 0x0EU;
    assert(SBUS_ParseFrame(frame, &data) == 0U);
    frame[0] = 0x00U;
    assert(SBUS_ParseFrame(frame, &data) == 0U);

    /*
     * ⚠ 帧尾【不校验】。标准是 0x00，但带 SBUS2 遥测的接收机会填
     * 0x04/0x14/0x24/0x34。严格判 0x00 会把那些接收机的合法帧全拒掉，
     * 现象是"波特率对、rxEventCount 正常涨，却一帧都解不出来"。
     */
    testBuildFrame(channels, 0U, 0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    testBuildFrame(channels, 0U, 0x04U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    testBuildFrame(channels, 0U, 0x14U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    testBuildFrame(channels, 0U, 0x34U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
}

/** @brief 标志字节的四个位各自独立解出，且不影响通道值。 */
static void testFlagBits(void)
{
    uint16_t channels[SBUS_CH_COUNT];
    uint8_t frame[SBUS_FRAME_LEN];
    sbus_data_t data;

    testFillMid(channels);

    testBuildFrame(channels, SBUS_FLAG_FRAME_LOST, 0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    assert(data.frameLost == 1U);
    assert(data.failsafe == 0U);

    testBuildFrame(channels, SBUS_FLAG_FAILSAFE, 0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    assert(data.frameLost == 0U);
    assert(data.failsafe == 1U);

    testBuildFrame(channels,
                   (uint8_t)(SBUS_FLAG_FRAME_LOST | SBUS_FLAG_FAILSAFE),
                   0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    assert(data.frameLost == 1U);
    assert(data.failsafe == 1U);

    /* CH17/CH18 这两个数字通道本工程不用，但不能让它们污染另外两位。 */
    testBuildFrame(channels,
                   (uint8_t)(SBUS_FLAG_CH17 | SBUS_FLAG_CH18),
                   0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    assert(data.frameLost == 0U);
    assert(data.failsafe == 0U);

    /* 标志位不能改变通道值——末路 CH16 与标志字节相邻，最容易串。 */
    assert(data.channel[SBUS_CH_COUNT - 1U] == SBUS_CH_MID);
}

static void testAxisNormalization(void)
{
    const int16_t deadband = 13;
    const int16_t span = SBUS_AXIS_MAX;

    /* 死区内一律归零。 */
    assert(SBUS_NormalizeAxis(0, deadband) == 0.0f);
    assert(SBUS_NormalizeAxis(deadband, deadband) == 0.0f);
    assert(SBUS_NormalizeAxis((int16_t)-deadband, deadband) == 0.0f);

    /* 满杆到 ±1。 */
    assert(fabsf(SBUS_NormalizeAxis(span, deadband) - 1.0f) < TEST_TOLERANCE);
    assert(fabsf(SBUS_NormalizeAxis((int16_t)-span, deadband) + 1.0f) <
           TEST_TOLERANCE);

    /* 超出物理量程按端点限幅，不允许算出大于 1 的值。 */
    assert(fabsf(SBUS_NormalizeAxis((int16_t)(span + 300), deadband) - 1.0f) <
           TEST_TOLERANCE);
    assert(fabsf(SBUS_NormalizeAxis((int16_t)(-span - 300), deadband) + 1.0f) <
           TEST_TOLERANCE);

    /* 刚出死区符号正确。 */
    assert(SBUS_NormalizeAxis((int16_t)(deadband + 1), deadband) > 0.0f);
    assert(SBUS_NormalizeAxis((int16_t)(-deadband - 1), deadband) < 0.0f);

    /* 死区大于等于量程时退化为恒零，不应出现除零。 */
    assert(SBUS_NormalizeAxis(span, span) == 0.0f);
}

/**
 * @brief 拨杆方向：低值 = UP，高值 = DOWN。
 *
 * ⚠⚠ 这条是安全判据，不是风格问题。SwC 是使能级，【上=急停、下=运行】；
 * 判反了会让操作者以为在打急停、实际在选运行。i6X 的拨杆是"上小下大"
 * （UP=172 / MID=992 / DOWN=1811），与"值大=位置高"的直觉相反，
 * 所以特别容易被"顺手改回来"。改反了这里必须红。
 */
static void testSwitchThresholds(void)
{
    assert(SBUS_ConvertSwitch(SBUS_CH_MIN) == REMOTE_SWITCH_UP);
    assert(SBUS_ConvertSwitch(SBUS_CH_MID) == REMOTE_SWITCH_MID);
    assert(SBUS_ConvertSwitch(SBUS_CH_MAX) == REMOTE_SWITCH_DOWN);

    /* 阈值两侧各取一个点，确认判据边界。 */
    assert(SBUS_ConvertSwitch(SBUS_SW_LOW_MAX) == REMOTE_SWITCH_UP);
    assert(SBUS_ConvertSwitch(SBUS_SW_LOW_MAX + 1U) == REMOTE_SWITCH_MID);
    assert(SBUS_ConvertSwitch(SBUS_SW_HIGH_MIN) == REMOTE_SWITCH_DOWN);
    assert(SBUS_ConvertSwitch(SBUS_SW_HIGH_MIN - 1U) == REMOTE_SWITCH_MID);

    /* 通道值残留 0 时落在 UP 一侧——退化方向必须是安全的那边。 */
    assert(SBUS_ConvertSwitch(0U) == REMOTE_SWITCH_UP);
}

/**
 * @brief 量程常量的自洽性。
 *
 * ⚠ 先说清这条【测不了什么】，免得过度信任它：
 *
 * 1. SBUS_CH_MIN/MID/MAX 是在真机上量出来的硬件事实，主机测试没有真值可比。
 *    三个值被【整体】换成另一组自洽的数（比如照抄 Futaba 惯例的 172/992/1811），
 *    本文件所有用例照样全绿。那种错只能靠实机复测发现。
 * 2. 单个端点偏几个计数也抓不到（对称性容差是 1%，约 8 个计数）。那种偏差
 *    只会让满杆读到 0.995 而不是 1.0，实际无影响；收得更紧的话，重新实测时
 *    正常的 2~3 计数偏差反而会误报。
 *
 * 能测的是【部分修改】——改了中位忘了改端点，或者反过来。那才是现实中最容易
 * 犯的错，而且后果实实在在：本机实测两侧半幅对称到 1 个计数以内，任一个值被
 * 单独动过，对称性就会明显破掉。
 */
static void testRangeConstants(void)
{
    const int32_t lowerSpan = (int32_t)SBUS_CH_MID - (int32_t)SBUS_CH_MIN;
    const int32_t upperSpan = (int32_t)SBUS_CH_MAX - (int32_t)SBUS_CH_MID;
    int32_t asymmetry = lowerSpan - upperSpan;

    /* 顺序不能乱，阈值必须严格落在中位两侧。 */
    assert(SBUS_CH_MIN < SBUS_SW_LOW_MAX);
    assert(SBUS_SW_LOW_MAX < SBUS_CH_MID);
    assert(SBUS_CH_MID < SBUS_SW_HIGH_MIN);
    assert(SBUS_SW_HIGH_MIN < SBUS_CH_MAX);

    /* 11 位协议装得下。 */
    assert(SBUS_CH_MAX <= 2047U);

    /* 两侧半幅对称：实测差 1 个计数，这里留到 1% 的余量。 */
    if (asymmetry < 0)
    {
        asymmetry = -asymmetry;
    }
    assert(asymmetry * 100 < upperSpan);

    /* AXIS_MAX 必须就是正半幅，别被写成常数。 */
    assert((int32_t)SBUS_AXIS_MAX == upperSpan);

    /* 拨杆阈值取半幅的一半，与 i-BUS 的 ±250/500 同比例，允许取整误差。 */
    assert(((int32_t)SBUS_CH_MID - (int32_t)SBUS_SW_LOW_MAX) * 2 - upperSpan <= 2);
    assert(((int32_t)SBUS_SW_HIGH_MIN - (int32_t)SBUS_CH_MID) * 2 - upperSpan <= 2);
}

/*
 * 映射一致性：每个通道给互不相同的值，检查 Remote_t 的每个字段确实取自
 * 常量声明的那个通道。重新映射常量后本用例仍然成立。
 */
static void testRemoteMapping(void)
{
    uint16_t channels[SBUS_CH_COUNT];
    uint8_t frame[SBUS_FRAME_LEN];
    sbus_data_t data;
    Remote_t remote;
    uint32_t index;

    /*
     * 偏移取互不相同的值，期望值【由 SBUS_AXIS_MAX 推导】而不是写死小数——
     * 端点是实测出来的，将来换接收机或改发射机行程还会变，写死 0.2/0.4 之类
     * 会让这条用例因为与本用例无关的原因崩掉。归一化算术本身由
     * testAxisNormalization 负责，这里只管"哪个通道落到哪个字段、符号对不对"。
     */
    const int16_t offsetSmall = (int16_t)(SBUS_AXIS_MAX / 5);  /* 约 0.2 */
    const int16_t offsetMid = (int16_t)(SBUS_AXIS_MAX / 2);    /* 约 0.5 */
    const int16_t offsetLarge = (int16_t)(SBUS_AXIS_MAX / 4 * 3); /* 约 0.75 */
    const int16_t offsetFull = (int16_t)(SBUS_AXIS_MAX / 8 * 7); /* 约 0.875 */

    testFillMid(channels);
    /* 四个摇杆轴各给一个不同的偏移，符号也各不相同 */
    channels[SBUS_CH_RIGHT_X] = (uint16_t)(SBUS_CH_MID + offsetSmall);
    channels[SBUS_CH_RIGHT_Y] = (uint16_t)(SBUS_CH_MID + offsetMid);
    channels[SBUS_CH_LEFT_X] = (uint16_t)(SBUS_CH_MID - offsetSmall);
    channels[SBUS_CH_LEFT_Y] = (uint16_t)(SBUS_CH_MID - offsetMid);
    /* 两个旋钮给不同的值和不同的符号，防止 A/B 接反了还能过 */
    channels[SBUS_CH_KNOB_A] = (uint16_t)(SBUS_CH_MID + offsetLarge);
    channels[SBUS_CH_KNOB_B] = (uint16_t)(SBUS_CH_MID - offsetFull);
    /* 四个拨杆给两种不同位置，交错排布，接错任意两路都会被抓到 */
    channels[SBUS_CH_SW_A] = SBUS_CH_MAX;
    channels[SBUS_CH_SW_B] = SBUS_CH_MIN;
    channels[SBUS_CH_SW_C] = SBUS_CH_MIN;
    channels[SBUS_CH_SW_D] = SBUS_CH_MAX;
    testBuildFrame(channels, 0U, 0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);

    SBUS_MakeRemote(&data, 0, &remote);

    /* 四个轴的相对大小关系必须和通道值一致，且符号正确 */
    assert(remote.rightStick.x > 0.0f);
    assert(remote.rightStick.y > remote.rightStick.x);
    assert(remote.leftStick.x < 0.0f);
    assert(remote.leftStick.y < remote.leftStick.x);
    assert(fabsf(remote.rightStick.x -
                 ((float)offsetSmall / (float)SBUS_AXIS_MAX)) < TEST_TOLERANCE);
    assert(fabsf(remote.rightStick.y -
                 ((float)offsetMid / (float)SBUS_AXIS_MAX)) < TEST_TOLERANCE);

    assert(fabsf(remote.knobA -
                 ((float)offsetLarge / (float)SBUS_AXIS_MAX)) < TEST_TOLERANCE);
    assert(fabsf(remote.knobB +
                 ((float)offsetFull / (float)SBUS_AXIS_MAX)) < TEST_TOLERANCE);

    /*
     * CH_MAX 是物理 DOWN，CH_MIN 是物理 UP，见 testSwitchThresholds。
     * ⚠ SwC 是使能级，它判反会让"急停位置"变成"运行位置"，所以这里钉死。
     */
    assert(remote.sw[REMOTE_SW_A] == REMOTE_SWITCH_DOWN);
    assert(remote.sw[REMOTE_SW_B] == REMOTE_SWITCH_UP);
    assert(remote.sw[REMOTE_SW_C] == REMOTE_SWITCH_UP);
    assert(remote.sw[REMOTE_SW_D] == REMOTE_SWITCH_DOWN);

    /* MakeRemote 不负责在线判定，那是任务层的事 */
    assert(remote.online == 0U);

    /* 全中位时所有归一化量回零，三档拨杆读出 MID */
    testFillMid(channels);
    testBuildFrame(channels, 0U, 0x00U, frame);
    assert(SBUS_ParseFrame(frame, &data) == 1U);
    SBUS_MakeRemote(&data, 0, &remote);
    assert(remote.rightStick.x == 0.0f);
    assert(remote.rightStick.y == 0.0f);
    assert(remote.leftStick.x == 0.0f);
    assert(remote.leftStick.y == 0.0f);
    assert(remote.knobA == 0.0f);
    assert(remote.knobB == 0.0f);
    for (index = 0U; index < (uint32_t)REMOTE_SW_COUNT; index++)
    {
        assert(remote.sw[index] == REMOTE_SWITCH_MID);
    }

    /* 映射用到的通道下标必须都在协议槽位范围内，且互不重复 */
    const uint32_t used[] = {
        SBUS_CH_RIGHT_X, SBUS_CH_RIGHT_Y, SBUS_CH_LEFT_X, SBUS_CH_LEFT_Y,
        SBUS_CH_KNOB_A, SBUS_CH_KNOB_B,
        SBUS_CH_SW_A, SBUS_CH_SW_B, SBUS_CH_SW_C, SBUS_CH_SW_D,
    };
    const uint32_t usedCount = (uint32_t)(sizeof(used) / sizeof(used[0]));

    for (index = 0U; index < usedCount; index++)
    {
        uint32_t other;

        assert(used[index] < SBUS_CH_COUNT);
        for (other = index + 1U; other < usedCount; other++)
        {
            assert(used[index] != used[other]);
        }
    }
}

int main(void)
{
    testValidFrame();
    testBitPackingIsolation();
    testHeaderAndFooter();
    testFlagBits();
    testAxisNormalization();
    testSwitchThresholds();
    testRangeConstants();
    testRemoteMapping();
    return 0;
}
