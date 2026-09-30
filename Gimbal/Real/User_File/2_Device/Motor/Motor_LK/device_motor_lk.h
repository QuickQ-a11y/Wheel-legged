#ifndef DEVICE_MOTOR_LK_H
#define DEVICE_MOTOR_LK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

/*
 * 翎控 MG4005E-i10 拨盘，单电机，只用 0xA1 转矩闭环命令。
 *
 * 接口形态照 device_motor_dji.h（GetState / IsOnline / 命令打包），
 * 只有一台，所以不带下标。总线、ID、标度全在 app_config.h 的 APP_LK_*，
 * ⚠ 其中四个标度常量还没有实机核对过，见那里的注释。
 */
typedef struct
{
    uint16_t encoderRaw;     /* 回复帧编码器值，0..APP_LK_ENCODER_PER_ROUND-1。 */
    uint16_t lastEncoderRaw; /* 上一次编码器值，用于判断本帧是否跨过回绕点。 */
    int32_t totalRound;      /* 编码器累计整圈数，带符号。 */
    int32_t totalEncoder;    /* 编码器累计计数 = totalRound × APP_LK_ENCODER_PER_ROUND + encoderRaw。 */
    float angle;             /* 【输出轴】累计角，rad。已除减速比、乘编码器方向，上电清零。 */
    int16_t speedDps;        /* 转速，度/秒，【电机端】口径（手册：输出轴要除减速比）。 */
    int16_t currentRaw;      /* 反馈转矩电流 iq 原始值，口径见 APP_LK_IQ_PER_A。 */
    int8_t temperature;      /* 电机温度，℃。 */
    uint32_t feedbackCount;  /* 有效反馈帧累计数量。 */
    uint32_t lastUpdateTick; /* 最近一次反馈 HAL tick。 */
    uint8_t isOnline;        /* 收到有效反馈后置 1。 */
} motor_lk_state_t;

/**
 * @brief 解析一帧 0xA1 回复并刷新状态，匹配成功返回 1。
 *
 * 只认 FDCAN3 上 ID 为 APP_LK_TRIGGER_ID、data[0] 为 0xA1 的帧：
 * 别的命令的回复字节布局完全不同，按 0xA1 解会得到一串垃圾。
 */
uint8_t Motor_LK_UpdateFeedback(app_can_bus_t bus,
                                uint32_t identifier,
                                const uint8_t data[APP_LK_FRAME_LEN]);

/** @brief 读取状态快照。 */
void Motor_LK_GetState(motor_lk_state_t *state);

/** @brief 判断反馈是否仍在线。 */
uint8_t Motor_LK_IsOnline(uint32_t nowTick);

/**
 * @brief 写入转矩电流命令 iq，原始值口径。
 *
 * 超量程按 ±APP_LK_IQ_MAX 限幅——这是协议有效范围，发送编码本身需要。
 * 拨盘的力矩限幅在模块层（Shooter.trigger_I_limit），这里只是最后一道兜底。
 */
void Motor_LK_SetCommand(int16_t iqRaw);

/** @brief 安全门。置位后照发 0xA1 帧但 iq 恒为零，命令值仍保留供 Watch 观察。 */
void Motor_LK_SetSafe(uint8_t safe);

/**
 * @brief 把命令打包成 0xA1 帧写入 CAN 发送缓存，调用方负责按控制周期调用。
 *
 * ⚠ 安全时也照发（iq=0）：翎控只在收到命令时才回复，不发帧就没有反馈，在线判定会掉。
 * busEnable 为 0 时一帧都不发，理由同 Motor_DJI_UpdateTxFrames（分步装机）。
 */
void Motor_LK_UpdateTxFrames(uint8_t busEnable);

#ifdef __cplusplus
}
#endif

#endif
