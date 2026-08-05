#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 遥控后端发布给底盘的外层模式请求。 */
typedef enum
{
    REMOTE_MODE_NONE = 0,
    REMOTE_MODE_FOLLOW,
    REMOTE_MODE_BENCH,
    REMOTE_MODE_SELF_SAVE,
    REMOTE_MODE_TOP,
    REMOTE_MODE_STEP,
} remote_mode_request_t;

/** @brief 遥控后端发布给底盘的离散腿长请求。 */
typedef enum
{
    REMOTE_LEG_KEEP = 0,
    REMOTE_LEG_SHORT,
    REMOTE_LEG_MIDDLE,
    REMOTE_LEG_LONG,
} remote_leg_request_t;

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

/** @brief 底盘周期使用的完整遥控输入快照。 */
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
    remote_leg_request_t legRequest;   /* 当前离散腿长请求。 */
} Remote_t;

extern Remote_t Remote;

#ifdef __cplusplus
}
#endif

#endif
