#ifndef DEVICE_SBUS_H
#define DEVICE_SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remote_input.h"

#include <stdint.h>

/*
 * S.BUS 串行输出。本车用的是 FS-iA10B 接收机（把输出模式切到 PPM/S.BUS），
 * 但协议本身是 Futaba 的，FrSky / Radiomaster 等也通用。
 *
 * 帧格式（定长 25 字节）：
 *   [0]      0x0F        帧头
 *   [1..22]  16 x 11 bit 通道，LSB 在前，跨字节边界连续打包
 *   [23]     标志字节    见下面的 SBUS_FLAG_*
 *   [24]     帧尾        标准是 0x00，但见下面关于帧尾的说明
 *
 * ⚠⚠ 与 i-BUS 最大的区别：【S.BUS 没有校验和】。i-BUS 有一个
 * "0xFFFF - 前 30 字节之和" 的真校验，S.BUS 只有帧头和 UART 偶校验兜底，
 * 坏帧漏过解析层的概率明显更高。
 * 作为补偿，标志字节里的 failsafe 被接进了在线判定（见 task_remote.c），
 * 那是 S.BUS 唯一强过 i-BUS 的地方——接收机会主动告诉我们它丢了发射机。
 *
 * ⚠ 16 路是【协议固定】的，与遥控器实际通道数无关：i6X 只用前 10 个。
 * 和 i-BUS 的 14 路一样，别用槽位数去推断型号。
 */
#define SBUS_FRAME_LEN 25U
#define SBUS_CH_COUNT 16U
#define SBUS_HEADER 0x0FU

/*
 * 通道量程。S.BUS 传的是 11 位无量纲整数，不像 i-BUS 直接就是 us。
 *
 * ⚠⚠ 下面这组是 **2026-09-24 在本车 FS-iA10B 上实测的**，不是 Futaba 惯例值。
 * 富斯的 us -> 11bit 换算系数和 Futaba 不一样：
 *   Futaba 惯例  172 /  992 / 1811   ->  1.639 counts/us
 *   本机实测     240 / 1024 / 1807   ->  1.567 counts/us
 * 照抄网上那组 172/992/1811 会让满杆算出来只有 0.96、死区和拨杆阈值也跟着偏。
 *
 * 这组数可信的依据：两侧半幅对称到 1 个计数以内（1024-240=784、1807-1024=783），
 * 摇杆没推到底的话会明显不对称。而且与 i-BUS 的 1000/1500/2000 严格线性对应。
 *
 * SBUS_AXIS_MAX 取正半幅 783（两者取小的那个），满杆到负端会算出 -784/783
 * 再被限幅到 -1.0，正好。
 */
#define SBUS_CH_MIN 240U
#define SBUS_CH_MID 1024U
#define SBUS_CH_MAX 1807U
#define SBUS_AXIS_MAX ((int16_t)(SBUS_CH_MAX - SBUS_CH_MID))

/*
 * 三档拨杆的判据，取中位 ±392（半幅 783 的一半），与 i-BUS 的 ±250/500 同一比例。
 * 两档拨杆只落在两端，这两个阈值保证它不会被误判成 MID。
 *
 * ⚠ 拨杆方向和 i-BUS 一样是【上小下大】：UP=240、MID=1024、DOWN=1807。
 * 那是发射机的行为，不随协议改变（实测换 S.BUS 后增减方向没变）。所以
 * SBUS_ConvertSwitch 同样低值判 UP、高值判 DOWN，不要"顺手改回来"——
 * test_sbus.c 的 testSwitchThresholds 钉死了方向。
 */
#define SBUS_SW_LOW_MAX 632U
#define SBUS_SW_HIGH_MIN 1416U

/* 标志字节 frame[23] 的位。i-BUS 完全没有对应物。 */
#define SBUS_FLAG_CH17 0x01U
#define SBUS_FLAG_CH18 0x02U
#define SBUS_FLAG_FRAME_LOST 0x04U /* 本帧是接收机补的，上一帧没收到。 */
#define SBUS_FLAG_FAILSAFE 0x08U   /* 接收机已丢失发射机，通道值是预设值。 */

/*
 * 通道到语义的映射。下标与 i-BUS 完全相同——通道分配是发射机干的事
 * （系统菜单 → Aux. channels），与用哪个协议传输无关。
 *
 * **下标 4~9（两个旋钮 + 四个拨杆）已在 S.BUS 下实机确认**（2026-09-24）。
 * 下标 0~3 的四个摇杆轴没有逐项复测：它们是仅剩的四个槽，且与 i-BUS 同源，
 * 走一遍模式时顺带核对方向即可。
 *
 * 通道映射表和拨杆语义见 ../../../../../Codex文档/遥控键位与模式分配.md。
 */
#define SBUS_CH_RIGHT_X 0U /* CH1 */
#define SBUS_CH_RIGHT_Y 1U /* CH2 */
#define SBUS_CH_LEFT_Y 2U  /* CH3 */
#define SBUS_CH_LEFT_X 3U  /* CH4 */
#define SBUS_CH_KNOB_A 4U  /* CH5，VrA */
#define SBUS_CH_KNOB_B 5U  /* CH6，VrB */
#define SBUS_CH_SW_A 6U    /* CH7，两档 */
#define SBUS_CH_SW_B 7U    /* CH8，两档 */
#define SBUS_CH_SW_C 8U    /* CH9，唯一的三档开关 */
#define SBUS_CH_SW_D 9U    /* CH10，两档 */

/*
 * 本后端要求的 UART 参数。S.BUS 是 100000 8E2、反相电平。
 *
 * ⚠ 与 DR16 那组【只差停止位】——两者都是 100000、都是 9B+EVEN（即 8 数据位
 * 加 1 校验位）、都因为座子上的硬件反相器而不需要 MCU 再反一次。
 * S.BUS 是 2 位停止位，DBUS 是 1 位，这是唯一的区别。
 */
#define SBUS_UART_BAUD 100000U
#define SBUS_UART_WORDLENGTH UART_WORDLENGTH_9B
#define SBUS_UART_PARITY UART_PARITY_EVEN
#define SBUS_UART_STOPBITS UART_STOPBITS_2
/*
 * ⚠ 不能开 RX 反相。板上标 SBUS 的那个座子（信号脚 PD2 = UART5 RX）带【硬件
 * 反相器】，板子就是按 S.BUS / DBUS 这类反相电平设计的（手册 §6 把该脚标为
 * "反相 GPIO"）。S.BUS 本身反相，过一次反相器到 MCU 正好是正的，这边不能再翻。
 * i-BUS 那个后端相反：它是正常 TTL，被硬件翻反了，必须在 MCU 侧翻回来。
 */
#define SBUS_UART_RXINVERT UART_ADVFEATURE_RXINV_DISABLE

/** @brief 一帧解析出的原始通道值与链路标志。 */
typedef struct
{
    uint16_t channel[SBUS_CH_COUNT];
    uint8_t frameLost; /* 接收机报告上一帧丢了，本帧是补的。 */
    uint8_t failsafe;  /* 接收机已丢失发射机，通道值不可信。 */
} sbus_data_t;

/**
 * @brief 解码并校验一帧定长 S.BUS 数据。
 *
 * 只校验帧头，帧头不对返回 0 且不写 data。帧尾【不校验】，理由见 .c。
 */
uint8_t SBUS_ParseFrame(const uint8_t frame[SBUS_FRAME_LEN],
                        sbus_data_t *data);

/**
 * @brief 将已去中值的通道量转换为带死区的 -1..1 归一化量。
 *
 * 输入超出物理范围时按端点限幅；死区外重新映射，保证满杆仍为 1。
 * 与 IBUS_NormalizeAxis 同一口径，只是量程换成 S.BUS 的 ±819。
 */
float SBUS_NormalizeAxis(int16_t axis, int16_t deadband);

/**
 * @brief 将单个通道值转换为三档拨杆位置。
 */
Remote_Switch_t SBUS_ConvertSwitch(uint16_t channel);

/**
 * @brief 按上面的映射把原始通道转换为归一化遥控快照。
 *
 * 不设置 online——在线判定属于任务层，failsafe 也在那里消费。
 */
void SBUS_MakeRemote(const sbus_data_t *data,
                     int16_t deadband,
                     Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
