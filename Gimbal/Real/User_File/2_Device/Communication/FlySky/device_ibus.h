#ifndef DEVICE_IBUS_H
#define DEVICE_IBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remote_input.h"

#include <stdint.h>

/*
 * 富斯 FS-iA10B 接收机，i-BUS 串行输出，配 FS-i6X 发射机。
 *
 * 帧格式（定长 32 字节，约 143 Hz）：
 *   [0]      0x20        长度
 *   [1]      0x40        命令：通道数据
 *   [2..29]  14 x uint16 小端，单位 us，约 1000~2000，中位 1500
 *   [30..31] uint16      小端校验和 = 0xFFFF - 前 30 字节之和
 *
 * ⚠ 槽位数是【协议固定】的 14 个，与遥控器实际通道数无关：i6X 只用前 10 个，
 * 其余槽由接收机填中位。所以不要用槽位数去推断遥控器型号。
 */
#define IBUS_FRAME_LEN 32U
#define IBUS_CH_COUNT 14U
#define IBUS_HEADER_LEN 0x20U
#define IBUS_HEADER_CMD 0x40U

#define IBUS_CH_MIN 1000U
#define IBUS_CH_MID 1500U
#define IBUS_CH_MAX 2000U

/*
 * 三档拨杆的判据。两档拨杆只会落在两端，这两个阈值保证它不会被误判成 MID。
 * 取中位 ±250（量程的一半），远离任何一档的实际停留点。
 *
 * ⚠ 实测（2026-09-22）拨杆是【上小下大】：UP=1000、MID=1500、DOWN=2000。
 * 所以 IBUS_ConvertSwitch 里低值判 UP、高值判 DOWN，不要"顺手改回来"。
 * 三个档位各距阈值 250，余量充足。
 */
#define IBUS_SW_LOW_MAX 1250U
#define IBUS_SW_HIGH_MIN 1750U

/*
 * 通道到语义的映射。**摇杆和 VrA 已实机确认**（2026-09-22，i6X 出厂默认 Mode 2）：
 * 四个轴都是"右增 / 上增"，回中正好 1500，量程 1000~2000，与 Remote_t 的
 * "x 向右为正、y 向上为正" 同向，所以 IBUS_Axis 直接减中位即可，不加负号。
 *
 *   下标 0 (CH1) 右摇杆横      下标 4 (CH5) VrA 旋钮
 *   下标 1 (CH2) 右摇杆纵      下标 5 (CH6) VrB 旋钮（Remote_t 只有一个 dial，未用）
 *   下标 2 (CH3) 左摇杆纵
 *   下标 3 (CH4) 左摇杆横
 *
 * 拨杆同样已实机确认：出厂状态下 SwA~SwD 未分配通道，在发射机
 * 系统菜单 → Aux. channels 里把 SwA~SwD 依次分配到 CH7~CH10 之后，
 * 落在下标 6/7/8/9。
 *
 * ⚠ i6X 默认只有 SwC 是三档，SwA/SwB/SwD 是两档。整套模式语义就是按这个
 * 硬件配置排的：SwC 当使能级（需要三档），SwA/SwB/SwD 只用两档。所以
 * 【不需要】换三档开关模块，也不需要再改发射机菜单。
 * 完整键位表见 ../../../../../Codex文档/遥控键位与模式分配.md。
 */
#define IBUS_CH_RIGHT_X 0U /* CH1 */
#define IBUS_CH_RIGHT_Y 1U /* CH2 */
#define IBUS_CH_LEFT_Y 2U  /* CH3 */
#define IBUS_CH_LEFT_X 3U  /* CH4 */
#define IBUS_CH_KNOB_A 4U  /* CH5，VrA，已确认 */
#define IBUS_CH_KNOB_B 5U  /* CH6，VrB，已确认 */
#define IBUS_CH_SW_A 6U    /* CH7，两档 */
#define IBUS_CH_SW_B 7U    /* CH8，两档 */
#define IBUS_CH_SW_C 8U    /* CH9，唯一的三档开关 */
#define IBUS_CH_SW_D 9U    /* CH10，两档 */

/*
 * 本后端要求的 UART 参数。i-BUS 是 115200 8N1、正常电平，与 DR16 的
 * 100000 8E1 不同，所以 task_remote 在启动接收前要按这组值重新初始化 UART。
 * 放在设备层是因为"这个接收机怎么说话"属于设备自己的事实。
 */
#define IBUS_UART_BAUD 115200U
#define IBUS_UART_WORDLENGTH UART_WORDLENGTH_8B
#define IBUS_UART_PARITY UART_PARITY_NONE
#define IBUS_UART_STOPBITS UART_STOPBITS_1
/*
 * ⚠ 必须开 RX 反相。板上标 SBUS 的那个座子（信号脚 PD2 = UART5 RX）带【硬件反相器】
 * ——板子是按 S.BUS / DBUS 这类反相电平设计的（手册 §6 把该脚标为"反相 GPIO"）。
 * i-BUS 输出的是正常 TTL，过一次反相器到 MCU 就是反的，所以这边要再反一次抵消。
 * DR16 那边相反：DBUS 本身是反相电平，被硬件反相器翻正，MCU 侧不能再翻。
 *
 * 判据（不是靠 uartErrorCount 一个数判断，那个数会骗人）：
 *   极性错 -> 线路永远被当成起始位，既报 FE 又【等不到空闲】，
 *             所以 lastUartError 固定 4 且 rxEventCount 恒为 0。
 *   极性对 -> rxEventCount 以约 143 Hz 上涨，lastRxSize 稳定 32。
 */
#define IBUS_UART_RXINVERT UART_ADVFEATURE_RXINV_ENABLE

/** @brief 一帧解析出的原始通道值，单位 us。 */
typedef struct
{
    uint16_t channel[IBUS_CH_COUNT];
} ibus_data_t;

/**
 * @brief 解码并校验一帧定长 i-BUS 数据。
 *
 * 校验帧头两字节和校验和，任一不过返回 0 且不写 data。
 */
uint8_t IBUS_ParseFrame(const uint8_t frame[IBUS_FRAME_LEN],
                         ibus_data_t *data);

/**
 * @brief 将已去中值的通道量转换为带死区的 -1..1 归一化量。
 *
 * 输入超出物理范围时按端点限幅；死区外重新映射，保证满杆仍为 1。
 * 与 DR16_NormalizeAxis 同一口径，只是量程换成 i-BUS 的 ±500。
 */
float IBUS_NormalizeAxis(int16_t axis, int16_t deadband);

/**
 * @brief 将单个通道值转换为三档拨杆位置。
 */
Remote_Switch_t IBUS_ConvertSwitch(uint16_t channel);

/**
 * @brief 按上面的映射把原始通道转换为归一化遥控快照。
 *
 * 不设置 online——在线判定属于任务层。
 */
void IBUS_MakeRemote(const ibus_data_t *data,
                      int16_t deadband,
                      Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
