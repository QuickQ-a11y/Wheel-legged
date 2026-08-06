#ifndef DEVICE_DR16_H
#define DEVICE_DR16_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remote_input.h"

#include <stdint.h>

#define DR16_FRAME_LEN 18U
#define DR16_CH_MIN 364U
#define DR16_CH_MID 1024U
#define DR16_CH_MAX 1684U

typedef enum
{
    DR16_SWITCH_UNKNOWN = 0,
    DR16_SWITCH_UP = 1,
    DR16_SWITCH_DOWN = 2,
    DR16_SWITCH_MID = 3,
} dr16_switch_t;

typedef enum
{
    DR16_KEY_W = (1U << 0),
    DR16_KEY_S = (1U << 1),
    DR16_KEY_D = (1U << 2),
    DR16_KEY_A = (1U << 3),
    DR16_KEY_SHIFT = (1U << 4),
    DR16_KEY_CTRL = (1U << 5),
    DR16_KEY_Q = (1U << 6),
    DR16_KEY_E = (1U << 7),
    DR16_KEY_R = (1U << 8),
    DR16_KEY_F = (1U << 9),
    DR16_KEY_G = (1U << 10),
    DR16_KEY_Z = (1U << 11),
    DR16_KEY_X = (1U << 12),
    DR16_KEY_C = (1U << 13),
    DR16_KEY_V = (1U << 14),
    DR16_KEY_B = (1U << 15),
} dr16_key_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t leftPressed;
    uint8_t rightPressed;
} dr16_mouse_t;

typedef struct
{
    int16_t rightX;
    int16_t rightY;
    int16_t leftX;
    int16_t leftY;
    int16_t dial;
    dr16_switch_t rightSwitch; /* S1，DBUS字节5的bit4..5。 */
    dr16_switch_t leftSwitch;  /* S2，DBUS字节5的bit6..7。 */
    dr16_mouse_t mouse;
    uint16_t keyBits;
    uint8_t dialValid;
} dr16_data_t;

/**
 * @brief 解码并校验一帧固定长度的 DR16 DBUS 数据。
 */
uint8_t DR16_ParseFrame(const uint8_t frame[DR16_FRAME_LEN],
                        dr16_data_t *data);

/**
 * @brief 将已去中值的 DBUS 摇杆轴转换为带死区的 -1..1 归一化量。
 *
 * 输入超出物理范围时按端点限幅；死区外重新映射，保证满杆仍为 1。
 */
float DR16_NormalizeAxis(int16_t axis, int16_t deadband);

/**
 * @brief 将DR16物理控件转换为归一化遥控快照。
 */
void DR16_MakeRemote(const dr16_data_t *data,
                     int16_t deadband,
                     int16_t dialThreshold,
                     Remote_t *remote);

#ifdef __cplusplus
}
#endif

#endif
