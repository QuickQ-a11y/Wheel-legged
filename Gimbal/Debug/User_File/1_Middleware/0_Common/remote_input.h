#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 遥控后端发布给云台的外层模式请求。 */
typedef enum
{
    REMOTE_MODE_NONE = 0,
    REMOTE_MODE_ZERO_FORCE, /* 无力，电机不输出。 */
    REMOTE_MODE_MANUAL,     /* 遥控手动控制云台。 */
    REMOTE_MODE_VISION,     /* 视觉自瞄，目标由 USB 控制帧给出。 */
    REMOTE_MODE_IDENT,      /* 系统辨识，注入激励信号（仅 Debug 工程消费）。 */
} remote_mode_request_t;

/*
 * 拨杆到模式的映射表。重排拨杆只改这四行，不要动 device_dr16.c。
 *
 * 右拨杆是使能级别：下=急停（走 remote_stop_flag，不产生模式请求）、
 * 中=下面这一行、上=由左拨杆的三行决定。
 * 可填任意 REMOTE_MODE_*，填 REMOTE_MODE_NONE 表示该档位不改变模式。
 */
#define REMOTE_MAP_RIGHT_MID  REMOTE_MODE_ZERO_FORCE

#define REMOTE_MAP_LEFT_UP    REMOTE_MODE_VISION
#define REMOTE_MAP_LEFT_MID   REMOTE_MODE_MANUAL
#define REMOTE_MAP_LEFT_DOWN  REMOTE_MODE_IDENT

/** @brief 与接收机协议无关的三挡拨杆位置。 */
typedef enum
{
    REMOTE_SWITCH_UNKNOWN = 0,
    REMOTE_SWITCH_UP,
    REMOTE_SWITCH_DOWN,
    REMOTE_SWITCH_MID,
} Remote_Switch_t;

/** @brief 单个二维摇杆的归一化位置。 */
typedef struct
{
    float x;                            /* 向右为正，范围-1..1。 */
    float y;                            /* 向上为正，范围-1..1。 */
} Remote_Stick_t;

/** @brief 云台周期使用的完整遥控输入快照。 */
typedef struct
{
    Remote_Stick_t leftStick;           /* 左摇杆。 */
    Remote_Stick_t rightStick;          /* 右摇杆。 */
    Remote_Switch_t leftSwitch;         /* 左拨杆。 */
    Remote_Switch_t rightSwitch;        /* 右拨杆。 */
    float dial;                         /* 滚轮归一化位置，范围-1..1。 */
    uint8_t dialValid;                  /* 当前接收机提供有效滚轮字段。 */
    uint8_t online;                     /* 已完成同步且未超时。 */
    remote_mode_request_t modeRequest; /* 当前外层模式请求。 */
} Remote_t;

extern Remote_t Remote;

#ifdef __cplusplus
}
#endif

#endif
