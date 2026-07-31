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
} remote_mode_request_t;

/** @brief 遥控后端发布给底盘的离散腿长请求。 */
typedef enum
{
    REMOTE_LEG_KEEP = 0,
    REMOTE_LEG_SHORT,
    REMOTE_LEG_MIDDLE,
    REMOTE_LEG_LONG,
} remote_leg_request_t;

/** @brief 与具体接收机协议无关的底盘遥控输入。 */
typedef struct
{
    float forwardAxis;                 /* 前进轴，范围-1..1。 */
    float yawAxis;                     /* 航向轴，范围-1..1。 */
    uint8_t stop;                      /* 非零时封锁最终电机输出。 */
    remote_mode_request_t modeRequest; /* 当前外层模式请求。 */
    remote_leg_request_t legRequest;   /* 当前离散腿长请求。 */
} remote_input_t;

#ifdef __cplusplus
}
#endif

#endif
