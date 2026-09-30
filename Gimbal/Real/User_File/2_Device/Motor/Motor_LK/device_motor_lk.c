#include "device_motor_lk.h"

#include "Limit.h"
#include "task_can.h"

#include "fdcan.h"
#include "stm32h7xx_hal.h"

/* 拨盘状态，反馈由 CAN 分发写入，上层只通过接口读取。 */
static motor_lk_state_t lkMotor;
static int16_t lkCommand;
static uint8_t lkSafe = 1U;

uint8_t Motor_LK_UpdateFeedback(app_can_bus_t bus,
                                uint32_t identifier,
                                const uint8_t data[APP_LK_FRAME_LEN])
{
    int32_t delta;

    /*
     * ⚠ 三样都要比：FDCAN3 是板间线，同一条线上还有云台 DM 和板间帧；
     * data[0] 是这条回复对应的命令字节，别的命令的回复布局完全不同。
     */
    if ((bus != APP_CAN_BUS_FDCAN3) ||
        (identifier != APP_LK_TRIGGER_ID) ||
        (data[0] != APP_LK_CMD_TORQUE))
    {
        return 0U;
    }

    /* ⚠ 翎控是【小端】：低字节在前。DJI 的反馈是大端，照抄就全反了。 */
    lkMotor.temperature = (int8_t)data[1];
    lkMotor.currentRaw = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8U));
    lkMotor.speedDps = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8U));
    lkMotor.lastEncoderRaw = lkMotor.encoderRaw;
    lkMotor.encoderRaw = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8U));

    /*
     * 编码器回绕累加，写法与 device_motor_dji.c 相同。
     * ⚠ 首帧的 lastEncoderRaw 还是 0，那个 delta 是假的，所以只有已经在线之后才判回绕。
     */
    if (lkMotor.isOnline != 0U)
    {
        delta = (int32_t)lkMotor.encoderRaw - (int32_t)lkMotor.lastEncoderRaw;
        if (delta < -(APP_LK_ENCODER_PER_ROUND / 2))
        {
            lkMotor.totalRound++;
        }
        else if (delta > (APP_LK_ENCODER_PER_ROUND / 2))
        {
            lkMotor.totalRound--;
        }
    }
    lkMotor.totalEncoder = (lkMotor.totalRound * APP_LK_ENCODER_PER_ROUND) +
                           (int32_t)lkMotor.encoderRaw;
    /* 累计计数 -> 输出轴弧度：除减速比、乘编码器方向，两个都还没实机核对，见 app_config.h。 */
    lkMotor.angle = (float)lkMotor.totalEncoder * (2.0f * APP_PI_F) /
                    ((float)APP_LK_ENCODER_PER_ROUND * APP_LK_TRIGGER_GEAR_RATIO) *
                    APP_LK_ENCODER_DIR;

    lkMotor.feedbackCount++;
    lkMotor.lastUpdateTick = HAL_GetTick();
    lkMotor.isOnline = 1U;
    return 1U;
}

void Motor_LK_GetState(motor_lk_state_t *state)
{
    *state = lkMotor;
}

uint8_t Motor_LK_IsOnline(uint32_t nowTick)
{
    if (lkMotor.isOnline == 0U)
    {
        return 0U;
    }

    return ((nowTick - lkMotor.lastUpdateTick) <= APP_LK_TIMEOUT_TICKS) ? 1U : 0U;
}

void Motor_LK_SetCommand(int16_t iqRaw)
{
    lkCommand = (int16_t)Algorithm_LimitSymmetric((float)iqRaw, (float)APP_LK_IQ_MAX);
}

void Motor_LK_SetSafe(uint8_t safe)
{
    lkSafe = (safe != 0U) ? 1U : 0U;
}

void Motor_LK_UpdateTxFrames(uint8_t busEnable)
{
    uint8_t data[APP_LK_FRAME_LEN] = {APP_LK_CMD_TORQUE, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    int16_t iq = (lkSafe != 0U) ? 0 : lkCommand;

    /* ⚠ 小端：低字节在 data[4]。 */
    data[4] = (uint8_t)((uint16_t)iq & 0xFFU);
    data[5] = (uint8_t)((uint16_t)iq >> 8U);

    /* 总线门为 0 时一帧都不发，理由见头文件。 */
    if (busEnable != 0U)
    {
        CAN_Task_UpdateTxFrame(&hfdcan3, APP_LK_TRIGGER_ID, data, APP_LK_FRAME_LEN);
    }
}
