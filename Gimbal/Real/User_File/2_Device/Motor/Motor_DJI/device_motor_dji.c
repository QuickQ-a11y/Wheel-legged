#include "device_motor_dji.h"

#include "Limit.h"
#include "task_can.h"

#include "fdcan.h"
#include "stm32h7xx_hal.h"

#include <string.h>

/* 四台发射机构电机状态，反馈由 CAN 分发写入，上层只通过接口读取。 */
static motor_dji_state_t djiMotors[APP_DJI_COUNT];
static int16_t djiCommand[APP_DJI_COUNT];
static uint8_t djiSafe = 1U;

/* 下标 -> 反馈 ID。发送槽位由 APP_DJI_TX_SLOT 从同一个 ID 推出，收发不会分家。 */
static const uint32_t djiRxId[APP_DJI_COUNT] = {
    APP_DJI_FRIC_1_RX_ID,
    APP_DJI_FRIC_2_RX_ID,
    APP_DJI_FRIC_3_RX_ID,
    APP_DJI_TRIGGER_RX_ID,
};

app_can_bus_t Motor_DJI_BusOf(motor_dji_index_t index)
{
    /* 拨盘装在底盘上，只能走两板之间的 FDCAN3；三个摩擦轮在云台本地 FDCAN2。 */
    return (index == MOTOR_DJI_TRIGGER) ? APP_CAN_BUS_FDCAN3 : APP_CAN_BUS_FDCAN2;
}

uint8_t Motor_DJI_UpdateFeedback(app_can_bus_t bus,
                                 uint32_t identifier,
                                 const uint8_t data[APP_DJI_RX_LEN])
{
    uint32_t index;

    for (index = 0U; index < APP_DJI_COUNT; index++)
    {
        motor_dji_state_t *motor;
        int32_t delta;

        if ((identifier != djiRxId[index]) ||
            (bus != Motor_DJI_BusOf((motor_dji_index_t)index)))
        {
            continue;
        }

        motor = &djiMotors[index];
        motor->lastEncoderRaw = motor->encoderRaw;
        motor->encoderRaw =
            (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);

        /*
         * 编码器回绕累加。拨盘位置环要的是"一共转过了多少"，不是 0..8191 的
         * 机械角——不累加的话每过一次边界位置就跳 8191 个计数，位置环会当成
         * 一个巨大误差猛冲。
         *
         * ⚠ 首帧的 lastEncoderRaw 还是 0，那个 delta 是假的，所以只有已经在线
         * 之后才判回绕。
         */
        if (motor->isOnline != 0U)
        {
            delta = (int32_t)motor->encoderRaw - (int32_t)motor->lastEncoderRaw;
            if (delta < -(APP_DJI_ENCODER_PER_ROUND / 2))
            {
                motor->totalRound++;
            }
            else if (delta > (APP_DJI_ENCODER_PER_ROUND / 2))
            {
                motor->totalRound--;
            }
        }
        motor->totalEncoder = (motor->totalRound * APP_DJI_ENCODER_PER_ROUND) +
                              (int32_t)motor->encoderRaw;
        /* 转子计数 -> 输出轴弧度。⚠ 少除减速比会差 36 倍，数值上看不出来。 */
        motor->angle = (float)motor->totalEncoder * (2.0f * APP_PI_F) /
                       ((float)APP_DJI_ENCODER_PER_ROUND * APP_DJI_GEAR_RATIO);

        motor->speedRpm =
            (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
        motor->currentRaw =
            (int16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
        /*
         * ⚠ C610 的错误码在 data[7]，data[6] 是空字节。C620 正好相反
         * （data[6] 温度、data[7] 空），照抄 C620 的解法会永远读到 0。
         */
        motor->errorCode = data[7];
        motor->feedbackCount++;
        motor->lastUpdateTick = HAL_GetTick();
        motor->isOnline = 1U;
        return 1U;
    }

    return 0U;
}

void Motor_DJI_GetState(motor_dji_index_t index, motor_dji_state_t *state)
{
    *state = djiMotors[index];
}

uint8_t Motor_DJI_IsOnline(motor_dji_index_t index, uint32_t nowTick)
{
    if (djiMotors[index].isOnline == 0U)
    {
        return 0U;
    }

    return ((nowTick - djiMotors[index].lastUpdateTick) <=
            APP_DJI_TIMEOUT_TICKS) ? 1U : 0U;
}

void Motor_DJI_SetCommand(motor_dji_index_t index, int16_t currentRaw)
{
    djiCommand[index] =
        (int16_t)Algorithm_LimitSymmetric((float)currentRaw,
                                          (float)APP_DJI_CURRENT_MAX);
}

void Motor_DJI_SetSafe(uint8_t safe)
{
    djiSafe = (safe != 0U) ? 1U : 0U;
}

void Motor_DJI_ClearCommands(void)
{
    memset(djiCommand, 0, sizeof(djiCommand));
}

/** @brief 把一台电机的命令电流按其电调 ID 写进所属帧的槽位。 */
static void Motor_DJI_PackOne(uint8_t data[APP_DJI_TX_LEN], uint32_t index)
{
    uint8_t slot = APP_DJI_TX_SLOT(djiRxId[index]);
    int16_t current = (djiSafe != 0U) ? 0 : djiCommand[index];

    data[slot] = (uint8_t)((uint16_t)current >> 8U);
    data[slot + 1U] = (uint8_t)((uint16_t)current);
}

void Motor_DJI_UpdateTxFrames(uint8_t fricBusEnable, uint8_t triggerBusEnable)
{
    uint8_t fricData[APP_DJI_TX_LEN] = {0};
    uint8_t triggerData[APP_DJI_TX_LEN] = {0};
    uint32_t index;

    /*
     * 两条总线各打一帧。帧 ID 都是 0x200，但挂在不同的 FDCAN 上，互不干扰：
     * 摩擦轮那帧只有 [0..5] 非零，拨盘那帧只有 [6..7] 非零，各自的空槽位
     * 寻址的电调在本条总线上不存在，填零没有副作用。
     */
    for (index = 0U; index < APP_DJI_COUNT; index++)
    {
        if (Motor_DJI_BusOf((motor_dji_index_t)index) == APP_CAN_BUS_FDCAN3)
        {
            Motor_DJI_PackOne(triggerData, index);
        }
        else
        {
            Motor_DJI_PackOne(fricData, index);
        }
    }

    /* 使能为 0 的总线一帧都不发，理由见头文件里的 BusOff 说明。 */
    if (fricBusEnable != 0U)
    {
        CAN_Task_UpdateTxFrame(&hfdcan2,
                               APP_DJI_TX_ID,
                               fricData,
                               APP_DJI_TX_LEN);
    }
    if (triggerBusEnable != 0U)
    {
        CAN_Task_UpdateTxFrame(&hfdcan3,
                               APP_DJI_TX_ID,
                               triggerData,
                               APP_DJI_TX_LEN);
    }
}
