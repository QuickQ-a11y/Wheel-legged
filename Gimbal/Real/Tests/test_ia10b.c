/*
 * FS-iA10B（i-BUS）帧解析与归一化的主机单测。
 *
 * 编译（在 Gimbal/Real/ 下执行）：
 *   gcc -Wall -Wextra \
 *       -IUser_File/1_Middleware/0_Common \
 *       -IUser_File/2_Device/Communication/IA10B \
 *       -o t Tests/test_ia10b.c \
 *       User_File/2_Device/Communication/IA10B/device_ia10b.c -lm
 *
 * 只依赖 remote_input.h 里的 Remote_t，不需要 remote_input.c，也不碰 HAL。
 *
 * 通道到语义的映射用常量表达而不是写死下标：实机确认 AETR 顺序后回填常量，
 * 这些用例仍然成立——它们验的是「转换逻辑与声明的映射一致」，不是「映射本身对」。
 */
#include "device_ia10b.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-6f

/** @brief 按 i-BUS 格式组帧：0x20 0x40 + 14 个小端通道 + 小端校验和。 */
static void testBuildFrame(const uint16_t channels[IA10B_CH_COUNT],
                           uint8_t frame[IA10B_FRAME_LEN])
{
    uint16_t sum = 0U;
    uint16_t checksum;
    uint32_t index;

    frame[0] = 0x20U;
    frame[1] = 0x40U;
    for (index = 0U; index < IA10B_CH_COUNT; index++)
    {
        frame[2U + (index * 2U)] = (uint8_t)(channels[index] & 0x00FFU);
        frame[3U + (index * 2U)] = (uint8_t)(channels[index] >> 8U);
    }
    for (index = 0U; index < (IA10B_FRAME_LEN - 2U); index++)
    {
        sum = (uint16_t)(sum + frame[index]);
    }
    checksum = (uint16_t)(0xFFFFU - sum);
    frame[30] = (uint8_t)(checksum & 0x00FFU);
    frame[31] = (uint8_t)(checksum >> 8U);
}

/** @brief 按当前帧内容重算并写回校验和，用于构造"只有某字段坏"的帧。 */
static void testFixChecksum(uint8_t frame[IA10B_FRAME_LEN])
{
    uint16_t sum = 0U;
    uint16_t checksum;
    uint32_t index;

    for (index = 0U; index < (IA10B_FRAME_LEN - 2U); index++)
    {
        sum = (uint16_t)(sum + frame[index]);
    }
    checksum = (uint16_t)(0xFFFFU - sum);
    frame[30] = (uint8_t)(checksum & 0x00FFU);
    frame[31] = (uint8_t)(checksum >> 8U);
}

/** @brief 全部通道填中位的基准帧。 */
static void testFillMid(uint16_t channels[IA10B_CH_COUNT])
{
    uint32_t index;

    for (index = 0U; index < IA10B_CH_COUNT; index++)
    {
        channels[index] = IA10B_CH_MID;
    }
}

static void testValidFrame(void)
{
    uint16_t channels[IA10B_CH_COUNT];
    uint8_t frame[IA10B_FRAME_LEN];
    ia10b_data_t data;
    uint32_t index;

    /* 每个通道给一个互不相同的值，能查出错位和字节序反了 */
    for (index = 0U; index < IA10B_CH_COUNT; index++)
    {
        channels[index] = (uint16_t)(IA10B_CH_MIN + (index * 70U));
    }
    testBuildFrame(channels, frame);

    memset(&data, 0, sizeof(data));
    assert(IA10B_ParseFrame(frame, &data) == 1U);
    for (index = 0U; index < IA10B_CH_COUNT; index++)
    {
        assert(data.channel[index] == channels[index]);
    }
}

static void testRejectBadFrames(void)
{
    uint16_t channels[IA10B_CH_COUNT];
    uint8_t frame[IA10B_FRAME_LEN];
    ia10b_data_t data;

    testFillMid(channels);

    /* 长度字节错 */
    testBuildFrame(channels, frame);
    frame[0] = 0x21U;
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    /* 命令字节错 */
    testBuildFrame(channels, frame);
    frame[1] = 0x41U;
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    /* 校验和错：翻一个数据位，校验和不跟着改 */
    testBuildFrame(channels, frame);
    frame[5] ^= 0x01U;
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    /* 校验和字段本身被改坏 */
    testBuildFrame(channels, frame);
    frame[30] = (uint8_t)(frame[30] + 1U);
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    /* 全零帧：头就不对，必须拒 */
    memset(frame, 0, sizeof(frame));
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    /*
     * ⚠ 上面几个坏头的用例同时也破坏了校验和，所以即使完全不查帧头也会被
     * 校验和挡下来——那样帧头检查等于没验。这两个用例把校验和补回正确值，
     * 单独隔离出帧头判据。（变异测试发现的缺口）
     */
    testBuildFrame(channels, frame);
    frame[0] = 0x21U;
    testFixChecksum(frame);
    assert(IA10B_ParseFrame(frame, &data) == 0U);

    testBuildFrame(channels, frame);
    frame[1] = 0x41U;
    testFixChecksum(frame);
    assert(IA10B_ParseFrame(frame, &data) == 0U);
}

static void testAxisNormalization(void)
{
    const int16_t deadband = 8;
    const int16_t span = (int16_t)(IA10B_CH_MAX - IA10B_CH_MID);

    /* 中位与死区内一律为 0 */
    assert(IA10B_NormalizeAxis(0, deadband) == 0.0f);
    assert(IA10B_NormalizeAxis(deadband, deadband) == 0.0f);
    assert(IA10B_NormalizeAxis((int16_t)-deadband, deadband) == 0.0f);

    /* 满偏为 ±1，死区不该把满杆压下来 */
    assert(fabsf(IA10B_NormalizeAxis(span, deadband) - 1.0f) < TEST_TOLERANCE);
    assert(fabsf(IA10B_NormalizeAxis((int16_t)-span, deadband) + 1.0f) <
           TEST_TOLERANCE);

    /* 超量程按端点限幅，不允许算出 |v| > 1 */
    assert(fabsf(IA10B_NormalizeAxis((int16_t)(span + 300), deadband) - 1.0f) <
           TEST_TOLERANCE);
    assert(fabsf(IA10B_NormalizeAxis((int16_t)(-span - 300), deadband) + 1.0f) <
           TEST_TOLERANCE);

    /* 死区外紧邻处应当刚离开 0，符号正确 */
    assert(IA10B_NormalizeAxis((int16_t)(deadband + 1), deadband) > 0.0f);
    assert(IA10B_NormalizeAxis((int16_t)(-deadband - 1), deadband) < 0.0f);

    /* 死区为 0 时就是纯线性 */
    assert(fabsf(IA10B_NormalizeAxis((int16_t)(span / 2), 0) - 0.5f) <
           TEST_TOLERANCE);
}

static void testSwitchThresholds(void)
{
    /*
     * ⚠ i6X 拨杆【上小下大】：实测 UP=1000、MID=1500、DOWN=2000。
     * 这几条断言就是用来钉死这个方向的 —— 如果有人"顺手"把它改成
     * 值大=UP，这里必须红。方向错了会让急停位置变成运行位置。
     */
    assert(IA10B_ConvertSwitch(IA10B_CH_MIN) == REMOTE_SWITCH_UP);
    assert(IA10B_ConvertSwitch(IA10B_CH_MID) == REMOTE_SWITCH_MID);
    assert(IA10B_ConvertSwitch(IA10B_CH_MAX) == REMOTE_SWITCH_DOWN);

    /* 两档拨杆只会给出两端，不能被误判成 MID */
    assert(IA10B_ConvertSwitch(IA10B_SW_LOW_MAX) == REMOTE_SWITCH_UP);
    assert(IA10B_ConvertSwitch((uint16_t)(IA10B_SW_LOW_MAX + 1U)) ==
           REMOTE_SWITCH_MID);
    assert(IA10B_ConvertSwitch(IA10B_SW_HIGH_MIN) == REMOTE_SWITCH_DOWN);
    assert(IA10B_ConvertSwitch((uint16_t)(IA10B_SW_HIGH_MIN - 1U)) ==
           REMOTE_SWITCH_MID);

    /* 通道值残留 0 时落到 UP 一侧。UP 是拨杆能开机的位置，也将映射为卸力，
       所以这个方向的退化是安全的；换成 DOWN 就不安全了。 */
    assert(IA10B_ConvertSwitch(0U) == REMOTE_SWITCH_UP);
}

/*
 * 映射一致性：每个通道给互不相同的值，检查 Remote_t 的每个字段确实取自
 * 常量声明的那个通道。重新映射常量后本用例仍然成立。
 */
static void testRemoteMapping(void)
{
    uint16_t channels[IA10B_CH_COUNT];
    uint8_t frame[IA10B_FRAME_LEN];
    ia10b_data_t data;
    Remote_t remote;
    uint32_t index;

    testFillMid(channels);
    /* 四个摇杆轴各给一个不同的偏移，符号也各不相同 */
    channels[IA10B_CH_RIGHT_X] = (uint16_t)(IA10B_CH_MID + 100U);
    channels[IA10B_CH_RIGHT_Y] = (uint16_t)(IA10B_CH_MID + 200U);
    channels[IA10B_CH_LEFT_X] = (uint16_t)(IA10B_CH_MID - 100U);
    channels[IA10B_CH_LEFT_Y] = (uint16_t)(IA10B_CH_MID - 200U);
    channels[IA10B_CH_DIAL] = (uint16_t)(IA10B_CH_MID + 300U);
    channels[IA10B_CH_RIGHT_SW] = IA10B_CH_MAX;
    channels[IA10B_CH_LEFT_SW] = IA10B_CH_MIN;
    testBuildFrame(channels, frame);
    assert(IA10B_ParseFrame(frame, &data) == 1U);

    IA10B_MakeRemote(&data, 0, &remote);

    /* 四个轴的相对大小关系必须和通道值一致，且符号正确 */
    assert(remote.rightStick.x > 0.0f);
    assert(remote.rightStick.y > remote.rightStick.x);
    assert(remote.leftStick.x < 0.0f);
    assert(remote.leftStick.y < remote.leftStick.x);
    assert(fabsf(remote.rightStick.x - 0.2f) < 1.0e-3f);   /* 100/500 */
    assert(fabsf(remote.rightStick.y - 0.4f) < 1.0e-3f);   /* 200/500 */

    assert(remote.dialValid == 1U);
    assert(fabsf(remote.dial - 0.6f) < 1.0e-3f);           /* 300/500 */

    /* CH_MAX=2000 是物理 DOWN，CH_MIN=1000 是物理 UP，见 testSwitchThresholds */
    assert(remote.rightSwitch == REMOTE_SWITCH_DOWN);
    assert(remote.leftSwitch == REMOTE_SWITCH_UP);

    /* MakeRemote 不负责在线判定，那是任务层的事 */
    assert(remote.online == 0U);

    /* 全中位时所有归一化量回零 */
    testFillMid(channels);
    testBuildFrame(channels, frame);
    assert(IA10B_ParseFrame(frame, &data) == 1U);
    IA10B_MakeRemote(&data, 0, &remote);
    assert(remote.rightStick.x == 0.0f);
    assert(remote.rightStick.y == 0.0f);
    assert(remote.leftStick.x == 0.0f);
    assert(remote.leftStick.y == 0.0f);
    assert(remote.dial == 0.0f);
    assert(remote.rightSwitch == REMOTE_SWITCH_MID);

    /* 映射用到的通道下标必须都在协议槽位范围内 */
    const uint32_t used[] = {
        IA10B_CH_RIGHT_X, IA10B_CH_RIGHT_Y, IA10B_CH_LEFT_X,
        IA10B_CH_LEFT_Y, IA10B_CH_DIAL, IA10B_CH_RIGHT_SW, IA10B_CH_LEFT_SW,
    };
    for (index = 0U; index < (sizeof(used) / sizeof(used[0])); index++)
    {
        assert(used[index] < IA10B_CH_COUNT);
    }
}

int main(void)
{
    testValidFrame();
    testRejectBadFrames();
    testAxisNormalization();
    testSwitchThresholds();
    testRemoteMapping();
    return 0;
}
