#include "chassis_task.h"

#include "app_config.h"
#include "chassis_mpc.h"
#include "device_board.h"
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

/*
 * MPC 求解任务。单独一个任务、优先级低于底盘，是这套架构的关键：
 * 实测单次求解是毫秒级，放在 1kHz 控制环里会把那一拍撑爆（见 3caf81a）。
 * 独立任务后求解随时可被底盘任务抢占，控制环的 deadline 不再受它影响，
 * 求解慢了只会让自己降频，表现为 Chassis_MPC.solve_count 增速变慢、age 变大。
 *
 * 栈 4096：实测 Release(-Os) 最深一支 Chassis_MPC_Solve(112) -> solve(256)
 * -> backward_pass_grad(800) -> Eigen(424) 约 1.8 KB。Debug(-O0) 因为 Eigen
 * 模板不展开要到 3.8 KB，也还装得下。溢出的表现不是报错而是整个任务冻结
 * （vApplicationStackOverflowHook 里 taskDISABLE_INTERRUPTS(); while(1);），
 * 极难和"求解超时"区分，所以宁可给足。
 */
static const osThreadAttr_t mpcTaskAttributes = {
    .name = "MpcTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityLow,
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

    /* 板间链路和电机一样，在线判定在任务层做，模块层只读标志不碰HAL。 */
    Chassis.board_online_flag = Board_IsOnline(nowTick);
    Chassis.gimbal_yaw_rel = Board_GetYawRel();
    Chassis.autoaim_flag = Board_GetAutoaim();

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
    /* 栈余量1秒查一次就够（历史最小值只增不减），每拍查会扫上千个字，白费周期。 */
    uint32_t stackCheckTick = 0U;

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
            /*
             * dt_raw 先留一份未钳位的：下面的越界回退会把"严重超时"换成
             * default_dt，两者在 dt 上分不开，判不了任务是不是被某一轮拖住了。
             */
            Chassis.dt_raw =
                (float)(controlTick - controlLastTick) * tickSec;
            if (Chassis.dt_raw > Chassis.dt_raw_max)
            {
                Chassis.dt_raw_max = Chassis.dt_raw;
            }

            Chassis.dt = Chassis.dt_raw;
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

        if (++stackCheckTick >= 1000U)
        {
            stackCheckTick = 0U;
            Chassis.task_stack_free = osThreadGetStackSpace(osThreadGetId());
        }

        wakeTick += APP_CTRL_TICKS;
        if ((int32_t)(osKernelGetTickCount() - wakeTick) >= 0)
        {
            wakeTick = osKernelGetTickCount() + APP_CTRL_TICKS;
        }
        (void)osDelayUntil(wakeTick);
    }
}

/**
 * @brief 按 MPC 自己的周期反复求解，吃控制环发布的最新输入。
 *
 * 用 osDelayUntil 自定时，不和底盘任务做握手：控制环每拍只管发布 x0、读 F，
 * 两边谁也不等谁。求解超过一个周期时下面的补偿分支会把节拍拉回来，
 * 表现为求解频率自然下降，而不是把延迟传导给控制环。
 */
static void Chassis_MPC_Task_Entry(void *argument)
{
    const uint32_t periodTicks =
        (uint32_t)Chassis_Config.mpc.decimation * APP_CTRL_TICKS;
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        if (Chassis_Config.output.mpc_flag != 0U)
        {
            Chassis_MPC_Solve();
        }

        wakeTick += periodTicks;
        if ((int32_t)(osKernelGetTickCount() - wakeTick) >= 0)
        {
            wakeTick = osKernelGetTickCount() + periodTicks;
        }
        (void)osDelayUntil(wakeTick);
    }
}

void Chassis_Task_Init(void)
{
    Chassis_Init();
    (void)osThreadNew(Chassis_Task_Entry, NULL, &chassisTaskAttributes);
    /* 求解器已在 Chassis_Init() 里建好，这之后再放 MPC 任务出来。 */
    (void)osThreadNew(Chassis_MPC_Task_Entry, NULL, &mpcTaskAttributes);
}
