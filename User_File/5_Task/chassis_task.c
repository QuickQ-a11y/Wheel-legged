#include "chassis_task.h"

#include "app_config.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "task_can.h"
#include "task_imu.h"
#include "task_remote.h"

#include "cmsis_os2.h"

#include <string.h>

static const osThreadAttr_t chassisTaskAttributes = {
    .name = "ChassisTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 从 IMU、DM、DJI 和 CAN 任务读取本轮底盘反馈。
 */
static void Chassis_Feedback_Update(void)
{
    task_imu_state_t imuState = {0};
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;

    /* IMU任务已经完成传感器坐标到整车右手系的转换。 */
    IMU_Task_GetState(&imuState);
    Chassis.imu.init_flag = imuState.isInitialized;
    Chassis.imu.attitude_flag = imuState.isAttitudeReady;
    Chassis.imu.error_code = imuState.lastErrorCode;
    Chassis.imu.roll = imuState.rollRad;
    Chassis.imu.pitch = imuState.pitchRad;
    Chassis.imu.yaw_total = imuState.yawTotalRad;
    memcpy(Chassis.imu.gyro,
           imuState.filteredGyroRadps,
           sizeof(Chassis.imu.gyro));
    memcpy(Chassis.imu.body_accel,
           imuState.bodyMotionAccMps2,
           sizeof(Chassis.imu.body_accel));
    memcpy(Chassis.imu.accel,
           imuState.motionAccMps2,
           sizeof(Chassis.imu.accel));
    /* 原始加速度保持传感器坐标，倒地姿态判据要的是重力方向，不能扣重力。 */
    memcpy(Chassis.imu.accel_raw,
           imuState.bmi088Data.accMps2,
           sizeof(Chassis.imu.accel_raw));

    /* 遥控输入在任务层转换为模式和物理目标，不接触LQR或电机输出。 */
    Remote_Task_Update();
    Chassis_Remote_Update(&Remote);

    /* DM状态保留最后一次反馈值，online只表示本周期是否超时。 */
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t motorState = {0};

        Motor_DM_GetState((motor_dm_index_t)index, &motorState);
        Chassis.dm_motor[index].online_flag =
            Motor_DM_IsOnline((motor_dm_index_t)index, nowTick);
        Chassis.dm_motor[index].err_state = motorState.state;
        Chassis.dm_motor[index].position_rad = motorState.positionRad;
        Chassis.dm_motor[index].speed_radps = motorState.velocityRadps;
        Chassis.dm_motor[index].torque_nm = motorState.torqueNm;
    }

    /* DJI轮电机同样分开保存反馈值和在线判定。 */
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        motor_dji_state_t wheelState = {0};

        Motor_DJI_GetState((motor_dji_index_t)index, &wheelState);
        Chassis.wheel_motor[index].online_flag =
            Motor_DJI_IsOnline((motor_dji_index_t)index, nowTick);
        Chassis.wheel_motor[index].speed_rpm = wheelState.speedRpm;
        Chassis.wheel_motor[index].current = wheelState.currentRaw;
    }

    Chassis.can_error_count = CAN_Task_GetTxErrorCount();
}

/**
 * @brief 将底盘最终命令写入 DM 设备层和 DJI CAN 发送缓存。
 */
static void Chassis_Command_Send(void)
{
    uint32_t index;

    /*
     * output.safe_flag是发送前最后安全门。触发后只清最终命令，
     * T_joint_req和I_wheel_req继续供Watch观察。
     */
    
    Motor_DM_SetSafe(Chassis.output.safe_flag);
    if (Chassis.output.safe_flag != 0U)
    {
        memset(Chassis.output.T_joint,
               0,
               sizeof(Chassis.output.T_joint));
        memset(Chassis.output.I_wheel,
               0,
               sizeof(Chassis.output.I_wheel));
        Motor_DM_ClearCommands();
    }

    /* 最终关节数组逐项写入DM设备层命令缓存。 */
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_command_t command = {
            .torqueNm = Chassis.output.T_joint[index],
        };

        Motor_DM_SetCommand((motor_dm_index_t)index, &command);
    }

    /* 更新两类发送缓存后只提交最新命令，不等待ACK或重发旧帧。 */
    // Chassis.output.I_wheel[0] = 400;
    // Chassis.output.I_wheel[0] = 0;
    // Chassis.output.I_wheel[1] = 0;
    CAN_Task_SetDjiCurrent(Chassis.output.I_wheel);
    Motor_DM_UpdateTxFrames();
    CAN_Task_RequestTx();
}

/**
 * @brief 周期执行底盘反馈、状态选择、控制计算和命令发送。
 *
 * DM协议上电使能只用于取得反馈；非零输出仍由底盘输出许可、设备
 * 状态、分路开关和output.safe_flag共同决定。
 */
static void Chassis_Task_Entry(void *argument)
{
    const float tickSec = 1.0f / (float)osKernelGetTickFreq();
    uint32_t controlLastTick = 0U;
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
    /*
     * DM 必须进入协议使能状态才会持续反馈。协议使能后仍由 safe 和
     * 底盘最终输出数组双重保证零力矩，不等同于开放底盘动力输出。
     */
    Motor_DM_SetEnable(1U);

    for (;;)
    {
        uint32_t controlTick = osKernelGetTickCount();

        /* 底盘PID、速度Kalman和位移积分只使用底盘自己的实际周期。 */
        if (controlLastTick == 0U)
        {
            Chassis.dt = Chassis_Config.default_dt;
        }
        else
        {
            Chassis.dt =
                (float)(controlTick - controlLastTick) * tickSec;
            if ((Chassis.dt < Chassis_Config.dt_min) ||
                (Chassis.dt > Chassis_Config.dt_max))
            {
                Chassis.dt = Chassis_Config.default_dt;
            }
        }
        controlLastTick = controlTick;

        /* 反馈 -> 状态选择 -> 控制 -> 电机命令，保持单向数据流。 */
        Chassis_Feedback_Update();
        Chassis_Leg_Update();
        Chassis_State_Update();

        /* 内部state只决定本周期调用哪条控制链，外部mode由遥控模块拥有。 */
        switch (Chassis.state)
        {
        case CHASSIS_STANDING:
            Chassis_Control();
            break;

        case CHASSIS_FALLEN:
        case CHASSIS_FALLING_TO_STAND:
            Chassis_Recovery();
            break;
        
        case CHASSIS_BENCH:
            Chassis_Bench();
            break;

        case CHASSIS_STEP:
            Chassis_Step();
            break;

        case CHASSIS_ZERO_FORCE:
        default:
            Chassis_Zero_Output();
            break;
        }

        Chassis_Command_Send();

        wakeTick += APP_CTRL_TICKS;
        if ((int32_t)(osKernelGetTickCount() - wakeTick) >= 0)
        {
            wakeTick = osKernelGetTickCount() + APP_CTRL_TICKS;
        }
        (void)osDelayUntil(wakeTick);
    }
}

void Chassis_Task_Init(void)
{
    Chassis_Init();
    (void)osThreadNew(Chassis_Task_Entry, NULL, &chassisTaskAttributes);
}
