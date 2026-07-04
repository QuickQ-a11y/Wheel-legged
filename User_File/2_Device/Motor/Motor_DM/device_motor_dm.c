#include "device_motor_dm.h"

#include "task_can.h"
#include "fdcan.h"

#include <string.h>

#define MOTOR_DM_POSITION_MIN_RAD (-12.5f)
#define MOTOR_DM_POSITION_MAX_RAD 12.5f
#define MOTOR_DM_VELOCITY_MIN_RADPS (-45.0f)
#define MOTOR_DM_VELOCITY_MAX_RADPS 45.0f
#define MOTOR_DM_TORQUE_MIN_NM (-40.0f)
#define MOTOR_DM_TORQUE_MAX_NM 40.0f

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

/**
 * @brief 对浮点值限幅。
 */
static float Motor_DM_LimitFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

/**
 * @brief 将物理量按 MIT 协议映射成无符号整数。
 */
static uint16_t Motor_DM_FloatToUint(float value,
                                     float minValue,
                                     float maxValue,
                                     uint8_t bits)
{
    float span = maxValue - minValue;
    float limitedValue = Motor_DM_LimitFloat(value, minValue, maxValue);
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
    if (bus == APP_CAN_BUS_FDCAN1)
    {
        return &hfdcan1;
    }

    if (bus == APP_CAN_BUS_FDCAN2)
    {
        return &hfdcan2;
    }

    return NULL;
}

/**
 * @brief 向发送缓存写入 DM 使能或失能特殊帧。
 */
static void Motor_DM_UpdateModeFrames(void)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        uint8_t data[APP_CONFIG_DM_FRAME_LENGTH] = {
            0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
            (dmEnable != 0U) ? 0xFCU : 0xFDU,
        };
        FDCAN_HandleTypeDef *handle = Motor_DM_GetCanHandle(dmMotors[index].config.bus);

        if (handle != NULL)
        {
            CAN_Task_UpdateTxFrame(handle,
                                   dmMotors[index].config.txId,
                                   data,
                                   APP_CONFIG_DM_FRAME_LENGTH);
        }
    }
}

void Motor_DM_Init(void)
{
    memset(dmMotors, 0, sizeof(dmMotors));

    dmMotors[MOTOR_DM_LEFT_FRONT].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1, APP_CONFIG_DM_LEFT_FRONT_TX_ID, APP_CONFIG_DM_LEFT_FRONT_RX_ID, APP_CONFIG_DM_LEFT_DIRECTION};
    dmMotors[MOTOR_DM_LEFT_BACK].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1, APP_CONFIG_DM_LEFT_BACK_TX_ID, APP_CONFIG_DM_LEFT_BACK_RX_ID, APP_CONFIG_DM_LEFT_DIRECTION};
    dmMotors[MOTOR_DM_RIGHT_FRONT].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1, APP_CONFIG_DM_RIGHT_FRONT_TX_ID, APP_CONFIG_DM_RIGHT_FRONT_RX_ID, APP_CONFIG_DM_RIGHT_DIRECTION};
    dmMotors[MOTOR_DM_RIGHT_BACK].config =
        (motor_dm_config_t){APP_CAN_BUS_FDCAN1, APP_CONFIG_DM_RIGHT_BACK_TX_ID, APP_CONFIG_DM_RIGHT_BACK_RX_ID, APP_CONFIG_DM_RIGHT_DIRECTION};

    dmSafe = 1U;
    dmEnable = 0U;
    dmModeRequest = 0U;
}

void Motor_DM_UpdateFeedback(app_can_bus_t bus,
                             uint32_t rxId,
                             const uint8_t data[APP_CONFIG_DM_FRAME_LENGTH])
{
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_object_t *motor = &dmMotors[index];

        if ((motor->config.bus != bus) || (motor->config.rxId != rxId))
        {
            continue;
        }

        uint16_t positionRaw = ((uint16_t)data[1] << 8U) | (uint16_t)data[2];
        uint16_t velocityRaw = ((uint16_t)data[3] << 4U) | ((uint16_t)data[4] >> 4U);
        uint16_t torqueRaw = (((uint16_t)data[4] & 0x0FU) << 8U) | (uint16_t)data[5];
        float direction = (float)motor->config.direction;

        motor->state.state = data[0] >> 4U;
        motor->state.positionRad =
            Motor_DM_UintToFloat(positionRaw, MOTOR_DM_POSITION_MIN_RAD, MOTOR_DM_POSITION_MAX_RAD, 16U) * direction;
        motor->state.velocityRadps =
            Motor_DM_UintToFloat(velocityRaw, MOTOR_DM_VELOCITY_MIN_RADPS, MOTOR_DM_VELOCITY_MAX_RADPS, 12U) * direction;
        motor->state.torqueNm =
            Motor_DM_UintToFloat(torqueRaw, MOTOR_DM_TORQUE_MIN_NM, MOTOR_DM_TORQUE_MAX_NM, 12U) * direction;
        motor->state.mosTemperature = data[6];
        motor->state.rotorTemperature = data[7];
        motor->state.feedbackCount++;
        motor->state.lastUpdateTick = HAL_GetTick();
        motor->state.isOnline = 1U;

        return;
    }
}

void Motor_DM_SetCommand(motor_dm_index_t index,
                         const motor_dm_command_t *command)
{
    if ((index >= MOTOR_DM_COUNT) || (command == NULL))
    {
        return;
    }

    dmMotors[index].command = *command;
}

void Motor_DM_GetState(motor_dm_index_t index,
                       motor_dm_state_t *state)
{
    if ((index >= MOTOR_DM_COUNT) || (state == NULL))
    {
        return;
    }

    *state = dmMotors[index].state;
}

void Motor_DM_SetSafe(uint8_t safe)
{
    dmSafe = (safe != 0U) ? 1U : 0U;
}

uint8_t Motor_DM_IsSafe(void)
{
    return dmSafe;
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

uint8_t Motor_DM_IsEnabled(void)
{
    return dmEnable;
}

uint8_t Motor_DM_IsOnline(motor_dm_index_t index, uint32_t nowTick)
{
    if (index >= MOTOR_DM_COUNT)
    {
        return 0U;
    }

    if (dmMotors[index].state.isOnline == 0U)
    {
        return 0U;
    }

    return ((nowTick - dmMotors[index].state.lastUpdateTick) <= APP_CONFIG_DM_ONLINE_TIMEOUT_TICKS) ? 1U : 0U;
}

void Motor_DM_ZeroAll(void)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        memset(&dmMotors[index].command, 0, sizeof(dmMotors[index].command));
    }
}

void Motor_DM_UpdateTxFrames(void)
{
    uint32_t index;

    if (dmModeRequest != 0U)
    {
        Motor_DM_UpdateModeFrames();
        dmModeRequest = 0U;
        return;
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        uint8_t data[APP_CONFIG_DM_FRAME_LENGTH] = {0};
        motor_dm_object_t *motor = &dmMotors[index];
        motor_dm_command_t command = motor->command;
        FDCAN_HandleTypeDef *handle = Motor_DM_GetCanHandle(motor->config.bus);
        float direction = (float)motor->config.direction;

        if (handle == NULL)
        {
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
        uint16_t torqueRaw = Motor_DM_FloatToUint(command.torqueNm * direction,
                                                  MOTOR_DM_TORQUE_MIN_NM,
                                                  MOTOR_DM_TORQUE_MAX_NM,
                                                  12U);

        data[6] = (uint8_t)(torqueRaw >> 8U);
        data[7] = (uint8_t)torqueRaw;

        CAN_Task_UpdateTxFrame(handle, motor->config.txId, data, APP_CONFIG_DM_FRAME_LENGTH);
    }
}
