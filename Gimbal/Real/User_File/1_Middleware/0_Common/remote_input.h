#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 与接收机协议无关的三挡拨杆位置。 */
typedef enum
{
    REMOTE_SWITCH_UNKNOWN = 0,
    REMOTE_SWITCH_UP,
    REMOTE_SWITCH_DOWN,
    REMOTE_SWITCH_MID,
} Remote_Switch_t;

/*
 * 拨杆下标。名字跟着 FS-i6X 的丝印走（SwA~SwD），因为整套模式语义是按它排的。
 *
 * ⚠ 枚举值 0..3 同时是板间协议的位域编码（每个拨杆 2 bit），
 * 见 app_config.h 的板间通信注释块。往里加档位会撑破 2 bit。
 *
 * DR16/DT7 只有两个拨杆，映射到 SW_C 和 SW_A，SW_B/SW_D 恒为 UNKNOWN，
 * 见 device_dr16.c 顶部的降级说明。
 */
typedef enum
{
    REMOTE_SW_A = 0,
    REMOTE_SW_B,
    REMOTE_SW_C,
    REMOTE_SW_D,
    REMOTE_SW_COUNT,
} Remote_Switch_Index_t;

/** @brief 单个二维摇杆的归一化位置。 */
typedef struct
{
    float x;                            /* 向右为正，范围-1..1。 */
    float y;                            /* 向上为正，范围-1..1。 */
} Remote_Stick_t;

/**
 * @brief 与接收机协议无关的遥控输入快照。
 *
 * 只放原始归一化输入，不放派生语义。拨杆到模式的映射是各板自己的业务：
 * 底盘在 chassis_remote.c 里做（它要持有 VrA 滞回和跳跃锁存这些跨周期状态），
 * 云台在 gimbal_control.c 里做。因此本文件在两块板上逐字节相同。
 */
typedef struct
{
    Remote_Stick_t leftStick;           /* 左摇杆。 */
    Remote_Stick_t rightStick;          /* 右摇杆。 */
    Remote_Switch_t sw[REMOTE_SW_COUNT];/* SwA~SwD。 */
    float knobA;                        /* VrA 旋钮，-1..1。底盘用它选模式组。 */
    float knobB;                        /* VrB 旋钮，-1..1。云台控摩擦轮和拨盘，不下发底盘。 */
    uint8_t online;                     /* 已完成同步且未超时。 */
} Remote_t;

extern Remote_t Remote;

#ifdef __cplusplus
}
#endif

#endif
