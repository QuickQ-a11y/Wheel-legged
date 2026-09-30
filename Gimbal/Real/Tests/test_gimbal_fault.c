/*
 * 云台部分的故障与安全门单测：yaw、pitch 任一台离线或报故障，云台两台一起清零；
 * 遥控离线、急停（SwC=UP）也清零。
 *
 * 链【真的】gimbal_control.c / gimbal_config.c / PID.c / Angle.c，只桩掉它下面的
 * DM 设备层、IMU 任务、遥控任务、板间通信和 CAN 请求。发射部分的代码一行都不链——
 * 能编过、能链上，本身就是"云台不依赖发射部分"的证据。
 *
 * 编译（在 Gimbal/Real/ 下执行；Tests/stub 必须排在最前）：
 *   gcc -Wall -Wextra -ITests/stub -IUser_File/5_Task \
 *       -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
 *       -IUser_File/1_Middleware/1_Driver/SPI -IUser_File/2_Device/IMU/BMI088 \
 *       -IUser_File/2_Device/Motor/Motor_DM -IUser_File/2_Device/Communication/Board \
 *       -IUser_File/2_Device/Communication/FlySky \
 *       -IUser_File/3_Chariot/1_Module/Gimbal \
 *       -o t Tests/test_gimbal_fault.c \
 *       User_File/3_Chariot/1_Module/Gimbal/gimbal_control.c \
 *       User_File/3_Chariot/1_Module/Gimbal/gimbal_config.c \
 *       User_File/1_Middleware/2_Algorithm/PID.c \
 *       User_File/1_Middleware/2_Algorithm/Angle.c -lm
 *
 * ⚠ 断言看的是安全门之后的 Gimbal.yaw.T / pitch.T，不是 T_req：
 *   T_req 故障时照样算（供 Watch 观察），清零发生在 Gimbal_Command_Send。
 */
#include "gimbal_control.h"

#include "device_board.h"
#include "task_can.h"
#include "task_imu.h"
#include "task_remote.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- 桩 ---- */

static uint32_t stubTick;
uint32_t HAL_GetTick(void) { return stubTick; }

/* 全局 Remote 的定义在 task_remote.c，测试里自己提供。 */
Remote_t Remote;
static Remote_t stubRemote;
void Remote_Task_GetRemote(Remote_t *remote) { *remote = stubRemote; }

static task_imu_state_t stubImu;
void IMU_Task_GetState(task_imu_state_t *state) { *state = stubImu; }

/* DM 设备层：状态由测试直接给，命令只记录。 */
static motor_dm_state_t stubDm[MOTOR_DM_COUNT];
static uint8_t stubDmOnline[MOTOR_DM_COUNT];

void Motor_DM_Init(void) {}
void Motor_DM_SetSafe(uint8_t safe) { (void)safe; }
void Motor_DM_SetEnable(uint8_t enable) { (void)enable; }
void Motor_DM_UpdateTxFrames(void) {}
void Motor_DM_GetState(motor_dm_index_t index, motor_dm_state_t *state)
{
    *state = stubDm[index];
}
uint8_t Motor_DM_IsOnline(motor_dm_index_t index, uint32_t nowTick)
{
    (void)nowTick;
    return stubDmOnline[index];
}
void Motor_DM_SetCommand(motor_dm_index_t index, const motor_dm_command_t *command)
{
    (void)index;
    (void)command;
}

void Board_UpdateTxFrames(const Remote_t *remote, float yaw_rel, uint8_t autoaim_flag)
{
    (void)remote;
    (void)yaw_rel;
    (void)autoaim_flag;
}

void CAN_Task_RequestTx(void) {}
uint32_t CAN_Task_GetTxErrorCount(void) { return 0U; }

/* ---- 辅助 ---- */

/* DM 反馈状态码（DM 手册"反馈帧 ERR"一节）：1 使能，0xA 过电流。 */
#define DM_STATE_ENABLED 0x1U
#define DM_STATE_OVERCURRENT 0xAU

/** @brief 一切健康：IMU 就绪、两台 DM 在线且使能、遥控在线运行档、右摇杆推满。 */
static void allHealthy(void)
{
    memset(&stubImu, 0, sizeof(stubImu));
    stubImu.isInitialized = 1U;
    stubImu.isAttitudeReady = 1U;

    memset(stubDm, 0, sizeof(stubDm));
    stubDm[MOTOR_DM_YAW].state = DM_STATE_ENABLED;
    stubDm[MOTOR_DM_PITCH].state = DM_STATE_ENABLED;
    stubDmOnline[MOTOR_DM_YAW] = 1U;
    stubDmOnline[MOTOR_DM_PITCH] = 1U;

    memset(&stubRemote, 0, sizeof(stubRemote));
    stubRemote.online = 1U;
    stubRemote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    /* 杆量是角速度语义：推满让目标一直领先反馈，两轴都有力矩请求。 */
    stubRemote.rightStick.x = 1.0f;
    stubRemote.rightStick.y = 1.0f;
}

/** @brief 跑一个云台控制周期。 */
static void step(void)
{
    stubTick++;
    Gimbal.dt = 0.001f;
    Gimbal_Feedback_Update();
    Gimbal_State_Update();
    Gimbal_Control();
    Gimbal_Command_Send();
}

/** @brief 跑够几十拍，让目标积分出明显的误差、两轴都有请求力矩。 */
static void run(uint32_t ticks)
{
    uint32_t i;

    for (i = 0U; i < ticks; i++)
    {
        step();
    }
}

static void assertBothZero(void)
{
    assert(Gimbal.safe_flag == 1U);
    assert(Gimbal.yaw.T == 0.0f);
    assert(Gimbal.pitch.T == 0.0f);
}

/* ---- 用例 ---- */

/**
 * @brief 全健康时两台都出力。
 *
 * 没有这条，后面那些"两台都为 0"的断言可能是空过——比如 T_req 本来就是 0。
 */
static void testHealthyBothOutput(void)
{
    allHealthy();
    run(50U);
    assert(Gimbal.fault == GIMBAL_FAULT_NONE);
    assert(Gimbal.safe_flag == 0U);
    assert(fabsf(Gimbal.yaw.T) > 0.0f);
    assert(fabsf(Gimbal.pitch.T) > 0.0f);
}

/** @brief yaw 离线：pitch 也停。云台部分内任一台离线，两台一起清零。 */
static void testYawOfflineStopsBoth(void)
{
    allHealthy();
    run(50U);
    stubDmOnline[MOTOR_DM_YAW] = 0U;
    step();
    assert((Gimbal.fault & GIMBAL_FAULT_DM) != 0U);
    assertBothZero();
}

/**
 * @brief pitch 报 DM 故障码（过电流 0xA），虽然仍在线：yaw 也停。
 *
 * 拦"云台只看在线、不看故障码"——改之前就是这样。DM 报故障后会自己退出使能，
 * 但仍按帧率回反馈，在线判定看不出来；按部分停的规则，另一台也要停。
 */
static void testDmErrorStopsBoth(void)
{
    allHealthy();
    run(50U);
    stubDm[MOTOR_DM_PITCH].state = DM_STATE_OVERCURRENT;
    step();
    assert((Gimbal.fault & GIMBAL_FAULT_DM_ERR) != 0U);
    assertBothZero();
}

/**
 * @brief DM 失能（状态码 0）不算故障：它是上电还没使能上的正常过程，由重发使能处理。
 *
 * 拦判据写成"状态码不等于 1"——那样上电那几十毫秒会被误判成故障。
 */
static void testDmDisabledIsNotError(void)
{
    allHealthy();
    stubDm[MOTOR_DM_YAW].state = 0x0U;
    run(5U);
    assert((Gimbal.fault & GIMBAL_FAULT_DM_ERR) == 0U);
}

/** @brief 遥控离线：两台都停（红线）。 */
static void testRemoteOfflineStopsBoth(void)
{
    allHealthy();
    run(50U);
    stubRemote.online = 0U;
    step();
    assert((Gimbal.fault & GIMBAL_FAULT_REMOTE) != 0U);
    assertBothZero();
}

/** @brief 急停（SwC=UP）：两台都卸力（红线）。 */
static void testEstopStopsBoth(void)
{
    allHealthy();
    run(50U);
    stubRemote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UP;
    step();
    assert(Gimbal.state == GIMBAL_ZERO_FORCE);
    assertBothZero();
}

int main(void)
{
    Gimbal_Init();

    testHealthyBothOutput();
    testYawOfflineStopsBoth();
    testDmErrorStopsBoth();
    testDmDisabledIsNotError();
    testRemoteOfflineStopsBoth();
    testEstopStopsBoth();

    printf("test_gimbal_fault: 6 组用例全过\n");
    return 0;
}
