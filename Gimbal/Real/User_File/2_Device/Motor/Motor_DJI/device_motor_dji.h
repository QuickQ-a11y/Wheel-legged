#ifndef DEVICE_MOTOR_DJI_H
#define DEVICE_MOTOR_DJI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

/*
 * 发射机构四台 M2006 + C610。数组下标按电调 ID 递增，与 0x200 帧的槽位顺序一致。
 *
 * ⚠ 前三台在 FDCAN2、拨盘在 FDCAN3，是【两条总线】，各自一个 0x200 帧。
 * 总线归属由 Motor_DJI_BusOf() 唯一决定，别在别处再判一次。
 */
typedef enum
{
    MOTOR_DJI_FRIC_1 = 0,  /* 摩擦轮 1，电调 ID 1，FDCAN2。 */
    MOTOR_DJI_FRIC_2,      /* 摩擦轮 2，电调 ID 2，FDCAN2。 */
    MOTOR_DJI_FRIC_3,      /* 摩擦轮 3，电调 ID 3，FDCAN2。 */
    MOTOR_DJI_TRIGGER,     /* 拨弹盘，电调 ID 4，FDCAN3。 */
} motor_dji_index_t;

/*
 * C610 反馈的电机错误码，取自反馈帧 data[7]（手册 §CAN 通信协议）。
 * ⚠ C620 的 data[7] 是空、data[6] 才是温度；C610 正好相反，data[6] 空。
 * 底盘的轮电机走 C620，那是另一份独立的 device_motor_dji.c。
 */
#define MOTOR_DJI_ERR_NONE 0U
#define MOTOR_DJI_ERR_OVERVOLTAGE 2U  /* 电调供电电压过高，仅开机自检一次。 */
#define MOTOR_DJI_ERR_PHASE_OPEN 3U   /* 电机三相线未接入。 */
#define MOTOR_DJI_ERR_SENSOR_LOST 4U  /* 4-Pin 位置传感器数据线信号丢失。 */
#define MOTOR_DJI_ERR_STALL 6U        /* 电机堵转。 */
#define MOTOR_DJI_ERR_CALIB_FAIL 7U   /* 电机校准失败。 */

typedef struct
{
    uint16_t encoderRaw;         /* 转子机械角，范围 0..8191 对应 0..360°。 */
    uint16_t lastEncoderRaw;     /* 上一次编码器值，用于判断本帧是否跨过回绕点。 */
    int32_t totalRound;          /* 转子累计整圈数，带符号。 */
    int32_t totalEncoder;        /* 转子累计计数 = totalRound × 8192 + encoderRaw。 */
    float angle;                 /* 【输出轴】累计角，rad。已除减速比，上电清零。 */
    /*
     * 转子转速，rpm。
     * ⚠ 换算到输出轴要除【该台电机自己的】减速比：摩擦轮的 P36 减速箱已经
     * 物理拆掉（1:1，speedRpm 就是轮速），拨盘的还在（36:1）。见 djiGearRatio[]。
     */
    int16_t speedRpm;
    int16_t currentRaw;          /* 实际转矩电流原始值，口径同 APP_DJI_CURRENT_MAX。 */
    uint8_t errorCode;           /* 电机错误码，见上面的 MOTOR_DJI_ERR_*。 */
    uint32_t feedbackCount;      /* 有效反馈帧累计数量。 */
    uint32_t lastUpdateTick;     /* 最近一次反馈 HAL tick。 */
    uint8_t isOnline;            /* 收到有效反馈后置 1。 */
} motor_dji_state_t;

/**
 * @brief 查询某台电机挂在哪条总线上。
 *
 * 总线归属只在这一处定义，接收端比对和发送端选帧都走它。
 */
app_can_bus_t Motor_DJI_BusOf(motor_dji_index_t index);

/**
 * @brief 解析一帧 C610 反馈并刷新对应电机状态，匹配成功返回 1。
 *
 * ⚠ 必须同时比对总线和 ID。0x201~0x203 在底盘的 FDCAN1 上是轮电调的反馈 ID，
 * 只比 ID 的话换线或将来合并总线时会读到别人的电机。
 */
uint8_t Motor_DJI_UpdateFeedback(app_can_bus_t bus,
                                 uint32_t identifier,
                                 const uint8_t data[APP_DJI_RX_LEN]);

/**
 * @brief 读取单台电机状态快照。
 */
void Motor_DJI_GetState(motor_dji_index_t index, motor_dji_state_t *state);

/**
 * @brief 判断电机反馈是否仍在线。
 */
uint8_t Motor_DJI_IsOnline(motor_dji_index_t index, uint32_t nowTick);

/**
 * @brief 写入单台电机的控制电流，单位与 APP_DJI_CURRENT_MAX 同口径。
 *
 * 超量程按 ±APP_DJI_CURRENT_MAX 限幅——这是发送编码本身需要的，
 * C610 对越界值的行为未定义。
 */
void Motor_DJI_SetCommand(motor_dji_index_t index, int16_t currentRaw);

/**
 * @brief 安全门。置位后发出去的电流恒为零，但命令值仍保留供 Watch 观察。
 */
void Motor_DJI_SetSafe(uint8_t safe);

/**
 * @brief 清空全部命令电流。
 */
void Motor_DJI_ClearCommands(void);

/**
 * @brief 把命令打包成两帧 0x200 写入 CAN 发送缓存，调用方负责按控制周期调用。
 *
 * ⚠ 两个使能参数不是多余的防御，它们控制【这条总线发不发帧】：
 * 某条总线上一个节点都没有时，发出去的帧没有任何节点应答 ACK，
 * 发送错误计数会一路爬到 BusOff，驱动随即 Stop/Start 恢复、再 BusOff，
 * 空转在错误恢复里。摩擦轮未装机时 FDCAN2 就是这种情况。
 * FDCAN3 上有 DM 电机和底盘板会 ACK，没有这个风险，但仍留开关以便分别调试。
 */
void Motor_DJI_UpdateTxFrames(uint8_t fricBusEnable, uint8_t triggerBusEnable);

#ifdef __cplusplus
}
#endif

#endif
