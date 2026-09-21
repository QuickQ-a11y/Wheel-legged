#ifndef DEVICE_IA10B_H
#define DEVICE_IA10B_H

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
#define IA10B_FRAME_LEN 32U
#define IA10B_CH_COUNT 14U
#define IA10B_HEADER_LEN 0x20U
#define IA10B_HEADER_CMD 0x40U

#define IA10B_CH_MIN 1000U
#define IA10B_CH_MID 1500U
#define IA10B_CH_MAX 2000U

/*
 * 三档拨杆的判据。两档拨杆只会落在两端，这两个阈值保证它不会被误判成 MID。
 * 取中位 ±250（量程的一半），远离任何一档的实际停留点。
 */
#define IA10B_SW_LOW_MAX 1250U
#define IA10B_SW_HIGH_MIN 1750U

/*
 * ⚠ 通道到语义的映射，按 i6X 出厂默认（Mode 2 / AETR，手册 7.13 与开关分配一节）。
 * 【这一组尚未实机确认，先按默认值填，上电看裸通道后回来改这里，不要改别处。】
 *   CH1 Aileron  -> 右摇杆横      CH5 VrA 旋钮
 *   CH2 Elevator -> 右摇杆纵      CH7..CH10 SwA/SwB/SwC/SwD
 *   CH3 Throttle -> 左摇杆纵
 *   CH4 Rudder   -> 左摇杆横
 *
 * ⚠ i6X 默认只有 SwC 是三档，SwA/SwB/SwD 是两档。所以挂在两档开关上的那一路
 * 读不出 REMOTE_SWITCH_MID——这是遥控器本身的限制，不是解析的 bug。要两个三档
 * 拨杆得在发射机里改开关配置。
 */
#define IA10B_CH_RIGHT_X 0U  /* CH1 */
#define IA10B_CH_RIGHT_Y 1U  /* CH2 */
#define IA10B_CH_LEFT_Y 2U   /* CH3 */
#define IA10B_CH_LEFT_X 3U   /* CH4 */
#define IA10B_CH_DIAL 4U     /* CH5，VrA */
#define IA10B_CH_RIGHT_SW 8U /* CH9，SwC，唯一的三档开关 */
#define IA10B_CH_LEFT_SW 9U  /* CH10，SwD，默认两档 */

/*
 * 本后端要求的 UART 参数。i-BUS 是 115200 8N1、正常电平，与 DR16 的
 * 100000 8E1 不同，所以 task_remote 在启动接收前要按这组值重新初始化 UART。
 * 放在设备层是因为"这个接收机怎么说话"属于设备自己的事实。
 */
#define IA10B_UART_BAUD 115200U
#define IA10B_UART_WORDLENGTH UART_WORDLENGTH_8B
#define IA10B_UART_PARITY UART_PARITY_NONE
#define IA10B_UART_STOPBITS UART_STOPBITS_1

/** @brief 一帧解析出的原始通道值，单位 us。 */
typedef struct
{
    uint16_t channel[IA10B_CH_COUNT];
} ia10b_data_t;

/**
 * @brief 解码并校验一帧定长 i-BUS 数据。
 *
 * 校验帧头两字节和校验和，任一不过返回 0 且不写 data。
 */
uint8_t IA10B_ParseFrame(const uint8_t frame[IA10B_FRAME_LEN],
                         ia10b_data_t *data);

/**
 * @brief 将已去中值的通道量转换为带死区的 -1..1 归一化量。
 *
 * 输入超出物理范围时按端点限幅；死区外重新映射，保证满杆仍为 1。
 * 与 DR16_NormalizeAxis 同一口径，只是量程换成 i-BUS 的 ±500。
 */
float IA10B_NormalizeAxis(int16_t axis, int16_t deadband);

/**
 * @brief 将单个通道值转换为三档拨杆位置。
 */
Remote_Switch_t IA10B_ConvertSwitch(uint16_t channel);

/**
 * @brief 按上面的映射把原始通道转换为归一化遥控快照。
 *
 * 不设置 online——在线判定属于任务层。
 */
void IA10B_MakeRemote(const ia10b_data_t *data,
                      int16_t deadband,
                      Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
