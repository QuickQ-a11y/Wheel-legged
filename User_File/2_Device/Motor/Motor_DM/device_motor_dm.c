#include "device_motor_dm.h"

#include "Angle.h"
#include "Limit.h"
#include "task_can.h"
#include "fdcan.h"

#include <string.h>

typedef struct
{
    motor_dm_config_t config;    /* 固定硬件映射。 */
    motor_dm_state_t state;      /* 最近一次反馈状态。 */
    motor_dm_command_t command;  /* 待发送 MIT 力矩命令。 */
} motor_dm_object_t;

static motor_dm_object_t dmMotors[MOTOR_DM_COUNT];
static uint8_t dmSafe = 1U;
static uint8_t dmEnable;
static uint8_t dmModeRequest;
static uint32_t dmEnableTick[MOTOR_DM_COUNT];

/**
 * @brief 将物理量按 MIT 协议映射成无符号整数。
 */
static uint16_t Motor_DM_FloatToUint(float value,
                                     float minValue,
                                     float maxValue,
                                     uint8_t bits)
{
    float span = maxValue - minValue;
    float limitedValue = Algorithm_LimitRange(value, minValue, maxValue);
    float scaledValue = (limitedValue - minValue) *
                        (float)((1UL << bits) - 1UL) /
                        span;

    return (uint16_t)scaledValue;
}

/**
 * @brief 将 MIT 协议无符号整数还原为物理量。
 */
static float Motor_DM_UintToFloat(uint16_t value,
                                  float minValue,
                                  float maxValue,
                                  uint8_t bits)
{
    float span = maxValue - minValue;

    return ((float)value * span / (float)((1UL << bits) - 1UL)) + minValue;
}

/**
 * @brief 根据应用层 CAN 总线枚举取得 HAL 句柄。
 */
static FDCAN_HandleTypeDef *Motor_DM_GetCanHandle(app_can_bus_t bus)
{
    return (bus == APP_CAN_BUS_FDCAN2) ? &hfdcan2 : &hfdcan1;
}

/**
 * @brief 向发送缓存写入 DM 使能或失能特殊帧。
 */
static void Motor_DM_UpdateModeFrame(uint32_t index, uint8_t enable)
{
    uint8_t data[APP_DM_FRAME_LEN] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
        (enable != 0U) ? 0xFCU : 0xFDU,
    };
    FDCAN_HandleTypeDef *handle;

    handle = Motor_DM_GetCanHandle(dmMotors[index].config.bus);

    CAN_Task_UpdateTxFrame(handle,
                           dmMotors[index].config.commandId,
                           data,
                           APP_DM_FRAME_LEN);
}

/**
 * @brief 初始化四台 DM 的 CAN 映射并复位命令与安全状态。
 */
void Motor_DM_Init(void)
{
    memset(dmMotors, 0, sizeof(dmMotors));

    dmMotors[MOTOR_DM_LEFT_FRONT].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN2,
                            APP_DM_LF_ID,
                            APP_DM_LF_FB};
    dmMotors[MOTOR_DM_LEFT_BACK].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN2,
                            APP_DM_LB_ID,
                            APP_DM_LB_FB};
    dmMotors[MOTOR_DM_RIGHT_FRONT].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1,
                            APP_DM_RF_ID,
                            APP_DM_RF_FB};
    dmMotors[MOTOR_DM_RIGHT_BACK].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1,
                            APP_DM_RB_ID,
                            APP_DM_RB_FB};

    dmSafe = 1U;
    dmEnable = 0U;
    dmModeRequest = 0U;
    memset(dmEnableTick, 0, sizeof(dmEnableTick));
}

uint8_t Motor_DM_UpdateFeedback(app_can_bus_t bus,
                                uint32_t identifier,
                                const uint8_t data[APP_DM_FRAME_LEN])
{
    uint8_t feedbackMotorId;
    uint32_t nowTick;
    uint32_t index;

    feedbackMotorId = data[0] & 0x0FU;
    nowTick = HAL_GetTick();
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_object_t *motor = &dmMotors[index];

        if ((motor->config.bus != bus) ||
            (motor->config.feedbackId != identifier))
        {
            continue;
        }
        if ((motor->config.commandId & 0x0FU) != feedbackMotorId)
        {
            return 0U;
        }

        uint16_t positionRaw = ((uint16_t)data[1] << 8U) | (uint16_t)data[2];
        uint16_t velocityRaw = ((uint16_t)data[3] << 4U) | ((uint16_t)data[4] >> 4U);
        uint16_t torqueRaw = (((uint16_t)data[4] & 0x0FU) << 8U) | (uint16_t)data[5];
        float wrappedPositionRad =
            Motor_DM_UintToFloat(positionRaw,
                                 APP_DM_PMIN,
                                 APP_DM_PMAX,
                                 16U);

        motor->state.state = data[0] >> 4U;
        if ((motor->state.feedbackCount == 0U) ||
            ((nowTick - motor->state.lastUpdateTick) > APP_DM_TIMEOUT_TICKS))
        {
            motor->state.positionRad = wrappedPositionRad;
        }
        else
        {
            motor->state.positionRad =
                Algorithm_AngleUnwrapRad(motor->state.positionWrappedRad,
                                         motor->state.positionRad,
                                         wrappedPositionRad);
        }
        motor->state.positionWrappedRad = wrappedPositionRad;
        motor->state.velocityRadps =
            Motor_DM_UintToFloat(velocityRaw,
                                 APP_DM_VEL_MIN,
                                 APP_DM_VEL_MAX,
                                 12U);
        motor->state.torqueNm =
            Motor_DM_UintToFloat(torqueRaw,
                                 APP_DM_TOR_MIN,
                                 APP_DM_TOR_MAX,
                                 12U);
        motor->state.mosTemperature = data[6];
        motor->state.rotorTemperature = data[7];
        motor->state.feedbackCount++;
        motor->state.lastUpdateTick = nowTick;
        motor->state.isOnline = 1U;

        return 1U;
    }

    return 0U;
}

void Motor_DM_SetCommand(motor_dm_index_t index,
                         const motor_dm_command_t *command)
{
    dmMotors[index].command = *command;
}

void Motor_DM_GetState(motor_dm_index_t index,
                       motor_dm_state_t *state)
{
    *state = dmMotors[index].state;
}

void Motor_DM_SetSafe(uint8_t safe)
{
    dmSafe = (safe != 0U) ? 1U : 0U;
}

void Motor_DM_SetEnable(uint8_t enable)
{
    uint8_t nextEnable = (enable != 0U) ? 1U : 0U;

    if (dmEnable != nextEnable)
    {
        dmEnable = nextEnable;
        dmModeRequest = 1U;
    }
}

uint8_t Motor_DM_IsOnline(motor_dm_index_t index, uint32_t nowTick)
{
    if (dmMotors[index].state.isOnline == 0U)
    {
        return 0U;
    }

    return ((nowTick - dmMotors[index].state.lastUpdateTick) <= APP_DM_TIMEOUT_TICKS) ? 1U : 0U;
}

void Motor_DM_ClearCommands(void)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        memset(&dmMotors[index].command, 0, sizeof(dmMotors[index].command));
    }
}

void Motor_DM_UpdateTxFrames(void)
{
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;

    if (dmModeRequest != 0U)
    {
        for (index = 0U; index < MOTOR_DM_COUNT; index++)
        {
            Motor_DM_UpdateModeFrame(index, dmEnable);
        }
        if (dmEnable != 0U)
        {
            for (index = 0U; index < MOTOR_DM_COUNT; index++)
            {
                dmEnableTick[index] = nowTick;
            }
        }
        dmModeRequest = 0U;
        return;
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        uint8_t data[APP_DM_FRAME_LEN] = {0};
        motor_dm_object_t *motor = &dmMotors[index];
        motor_dm_command_t command = motor->command;
        FDCAN_HandleTypeDef *handle = Motor_DM_GetCanHandle(motor->config.bus);

        /*
         * ERR=0 表示电机仍处于失能状态。使能请求按电机独立重发；
         * ERR=1 或 8~E 时不重发，避免用使能命令掩盖实际故障。
         */
        if ((dmEnable != 0U) &&
            (motor->state.state == 0U) &&
            ((nowTick - dmEnableTick[index]) >= APP_DM_EN_RETRY))
        {
            Motor_DM_UpdateModeFrame(index, 1U);
            dmEnableTick[index] = nowTick;
            continue;
        }

        if (dmSafe != 0U)
        {
            memset(&command, 0, sizeof(command));
        }

        /*
         * 关节电机采用 SPR 工程的 MIT 力矩发送方式：
         * 前 6 字节直接置 0，最后 2 字节放 12 bit 力矩编码。
         * 这表示当前固件只使用 DM MIT 的力矩通道，不在发送帧中下发位置、速度、KP、KD。
         */
        uint16_t torqueRaw = Motor_DM_FloatToUint(command.torqueNm,
                                                   APP_DM_TOR_MIN,
                                                   APP_DM_TOR_MAX,
                                                   12U);

        data[6] = (uint8_t)(torqueRaw >> 8U);
        data[7] = (uint8_t)torqueRaw;

        CAN_Task_UpdateTxFrame(handle,
                               motor->config.commandId,
                               data,
                               APP_DM_FRAME_LEN);
    }
}
