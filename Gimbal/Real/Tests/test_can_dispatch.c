/*
 * CAN 接收分发单测：FDCAN3 上三方（云台 DM、拨盘、板间）的反馈各进各的设备层，互不串台。
 *
 * 分发层本身不比 ID（见 task_can_dispatch.c 顶部注释），总线归属只在各设备层定义一次。
 * 所以这里链的是【真的】三个设备层，只桩掉 HAL_GetTick 和 CAN 发送。
 *
 * 编译（在 Gimbal/Real/ 下执行）：
 *   gcc -Wall -Wextra -ITests/stub -IUser_File/5_Task \
 *       -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/2_Device/Motor/Motor_DM -IUser_File/2_Device/Motor/Motor_DJI \
 *       -IUser_File/2_Device/Motor/Motor_LK \
 *       -o t Tests/test_can_dispatch.c User_File/5_Task/task_can_dispatch.c \
 *       User_File/2_Device/Motor/Motor_DM/device_motor_dm.c \
 *       User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c \
 *       User_File/2_Device/Motor/Motor_LK/device_motor_lk.c \
 *       User_File/1_Middleware/2_Algorithm/Angle.c -lm
 */
#include "task_can.h"

#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "device_motor_lk.h"
#include "fdcan.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 桩 ---- */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                            const uint8_t *data, uint8_t length)
{
    (void)handle;
    (void)identifier;
    (void)data;
    (void)length;
}

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

/* ---- 辅助 ---- */

typedef struct
{
    uint32_t dmYaw;
    uint32_t djiTrigger;
    uint32_t lk;
} feedback_counts_t;

static feedback_counts_t counts(void)
{
    feedback_counts_t c;
    motor_dm_state_t dm;
    motor_dji_state_t dji;
    motor_lk_state_t lk;

    Motor_DM_GetState(MOTOR_DM_YAW, &dm);
    Motor_DJI_GetState(MOTOR_DJI_TRIGGER, &dji);
    Motor_LK_GetState(&lk);
    c.dmYaw = dm.feedbackCount;
    c.djiTrigger = dji.feedbackCount;
    c.lk = lk.feedbackCount;
    return c;
}

static void deliver(app_can_bus_t bus, uint32_t identifier, const uint8_t data[8])
{
    task_can_rx_message_t message;

    memset(&message, 0, sizeof(message));
    message.bus = bus;
    message.identifier = identifier;
    message.length = 8U;
    memcpy(message.data, data, 8U);
    CAN_Task_RxMessageCallback(&message);
}

/* ---- 用例 ---- */

/**
 * @brief 翎控回复进翎控设备层，而且只进它。
 *
 * 拦分发层漏接翎控（拨盘永远离线，发射部分永远 fault）。
 */
static void testLkReplyReachesLkOnly(void)
{
    /* 0xA1 回复，小端：编码器 0x1234。 */
    const uint8_t reply[8] = {0xA1U, 30U, 0U, 0U, 0U, 0U, 0x34U, 0x12U};
    feedback_counts_t before = counts();
    feedback_counts_t after;
    motor_lk_state_t lk;

    stubTick = 100U;
    deliver(APP_CAN_BUS_FDCAN3, 0x141U, reply);

    after = counts();
    assert(after.lk == before.lk + 1U);
    assert(after.dmYaw == before.dmYaw);
    assert(after.djiTrigger == before.djiTrigger);
    Motor_LK_GetState(&lk);
    assert(lk.encoderRaw == 0x1234U);
}

/**
 * @brief 同一条 FDCAN3 上，DM 反馈和 C610 反馈各进各的，翎控不抢。
 *
 * 拦"新加的翎控分支认领了不属于它的 8 字节帧"——那样云台 DM 会一直离线。
 */
static void testFdcan3NeighboursDoNotCrossTalk(void)
{
    /* DM yaw 反馈：ID 0x011，data[0] 低 4 位是电机 ID 1，高 4 位 ERR=1（使能）。 */
    const uint8_t dmFeedback[8] = {0x11U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 30U, 30U};
    /* 拨盘 C610 反馈（大端），ID 0x204。 */
    const uint8_t djiFeedback[8] = {0x10U, 0x00U, 0x00U, 0x64U, 0x00U, 0x00U, 0x00U, 0x00U};
    feedback_counts_t before;
    feedback_counts_t after;

    stubTick = 200U;
    before = counts();
    deliver(APP_CAN_BUS_FDCAN3, APP_DM_YAW_FB, dmFeedback);
    after = counts();
    assert(after.dmYaw == before.dmYaw + 1U);
    assert(after.lk == before.lk);

    before = after;
    deliver(APP_CAN_BUS_FDCAN3, APP_DJI_TRIGGER_RX_ID, djiFeedback);
    after = counts();
    assert(after.djiTrigger == before.djiTrigger + 1U);
    assert(after.lk == before.lk);
    assert(after.dmYaw == before.dmYaw);
}

int main(void)
{
    Motor_DM_Init();

    testLkReplyReachesLkOnly();
    testFdcan3NeighboursDoNotCrossTalk();

    printf("test_can_dispatch: 2 组用例全过\n");
    return 0;
}
