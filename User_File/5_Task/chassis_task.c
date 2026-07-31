#include "chassis_task.h"

#include "app_config.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "task_can.h"
#include "task_can_dispatch.h"
#include "task_imu.h"
#include "task_remote.h"

#include "cmsis_os2.h"

#include <string.h>

static const osThreadAttr_t chassis_task_attributes = {
    .name = "ChassisTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

static remote_mode_request_t previousModeRequest = REMOTE_MODE_NONE;
static uint8_t yawStickActive;

/** @brief 取得十维模型使用的连续整车航向。 */
static float Chassis_GetModelYawRad(void)
{
    return chassis.imu.yaw_total_rad * chassis_config.imu.yaw_angle_scale;
}

/** @brief 急停或离线时清零运动命令并从当前连续航向重新锚定。 */
static void Chassis_HoldRemoteMotion(void)
{
    float yawRad = Chassis_GetModelYawRad();

    chassis.motion_command.forward_speed_mps = 0.0f;
    chassis.motion_command.yaw_anchor_rad = yawRad;
    chassis.motion_command.yaw_target_rad = yawRad;
    yawStickActive = 0U;
}

/** @brief 将通用遥控输入转换为底盘物理运动目标。 */
static void Chassis_UpdateRemoteMotion(const remote_input_t *input)
{
    float yawAxis = input->yawAxis;
    float yawRad = Chassis_GetModelYawRad();

    chassis.motion_command.forward_speed_mps =
        input->forwardAxis * APP_RC_MAX_VEL;

    if (yawAxis != 0.0f)
    {
        if (yawStickActive == 0U)
        {
            chassis.motion_command.yaw_anchor_rad = yawRad;
        }
        yawStickActive = 1U;
        chassis.motion_command.yaw_target_rad =
            chassis.motion_command.yaw_anchor_rad - yawAxis * APP_RC_MAX_YAW;
    }
    else
    {
        if (yawStickActive != 0U)
        {
            chassis.motion_command.yaw_anchor_rad = yawRad;
        }
        yawStickActive = 0U;
        chassis.motion_command.yaw_target_rad =
            chassis.motion_command.yaw_anchor_rad;
    }

    switch (input->legRequest)
    {
    case REMOTE_LEG_SHORT:
        chassis.motion_command.leg_length_m = APP_RC_LEG_S;
        break;

    case REMOTE_LEG_MIDDLE:
        chassis.motion_command.leg_length_m = APP_RC_LEG_M;
        break;

    case REMOTE_LEG_LONG:
        chassis.motion_command.leg_length_m = APP_RC_LEG_L;
        break;

    case REMOTE_LEG_KEEP:
    default:
        break;
    }
}

/**
 * @brief 维护遥控重新就绪、急停和外层动作模式。
 *
 * 急停和离线只关闭输出许可并保留当前模式，使控制中间量继续更新。
 * 必须先收到FOLLOW请求才建立控制许可；SELF_SAVE只在边沿触发一次。
 */
void Chassis_SetRemoteInput(const remote_input_t *input, uint8_t online)
{
    remote_mode_request_t modeRequest;

    if (input == NULL)
    {
        online = 0U;
        modeRequest = REMOTE_MODE_NONE;
    }
    else
    {
        modeRequest = input->modeRequest;
    }

    chassis.remote_online = (online != 0U) ? 1U : 0U;
    chassis.remote_stop =
        ((input != NULL) && (input->stop != 0U)) ? 1U : 0U;

    if ((chassis.remote_online == 0U) || (chassis.remote_stop != 0U))
    {
        Chassis_SetOutputEnable(0U);
        chassis.remote_control_ready = 0U;
        Chassis_HoldRemoteMotion();
        previousModeRequest = modeRequest;
        return;
    }

    if (chassis.remote_control_ready == 0U)
    {
        Chassis_SetOutputEnable(0U);
        Chassis_HoldRemoteMotion();
        if (modeRequest != REMOTE_MODE_FOLLOW)
        {
            previousModeRequest = modeRequest;
            return;
        }

        chassis.remote_control_ready = 1U;
        chassis.remote_target_valid = 1U;
        chassis.motion_command.yaw_anchor_rad = Chassis_GetModelYawRad();
        chassis.motion_command.yaw_target_rad =
            chassis.motion_command.yaw_anchor_rad;
        if (chassis.mode != CHASSIS_MODE_SELF_SAVE)
        {
            Chassis_SetMode(CHASSIS_MODE_FOLLOW);
        }
    }

    Chassis_SetOutputEnable(1U);
    Chassis_UpdateRemoteMotion(input);

    /* 只有恢复状态机真正回到STANDING后，才自动结束SELF_SAVE请求。 */
    if ((chassis.mode == CHASSIS_MODE_SELF_SAVE) &&
        (chassis.last_mode == CHASSIS_MODE_SELF_SAVE) &&
        (chassis.state == CHASSIS_STANDING))
    {
        Chassis_SetMode(CHASSIS_MODE_FOLLOW);
    }

    if (chassis.mode == CHASSIS_MODE_SELF_SAVE)
    {
        previousModeRequest = modeRequest;
        return;
    }

    /* 自救结束后保持FOLLOW，直到重新收到FOLLOW请求才解除触发锁。 */
    if (chassis.remote_self_save_latched != 0U)
    {
        Chassis_SetMode(CHASSIS_MODE_FOLLOW);
        if (modeRequest == REMOTE_MODE_FOLLOW)
        {
            chassis.remote_self_save_latched = 0U;
        }
        previousModeRequest = modeRequest;
        return;
    }

    if (modeRequest == REMOTE_MODE_FOLLOW)
    {
        Chassis_SetMode(CHASSIS_MODE_FOLLOW);
    }
    else if (modeRequest == REMOTE_MODE_BENCH)
    {
        Chassis_SetMode(CHASSIS_MODE_BENCH);
    }
    else if ((modeRequest == REMOTE_MODE_SELF_SAVE) &&
             (previousModeRequest != REMOTE_MODE_SELF_SAVE) &&
             (previousModeRequest != REMOTE_MODE_NONE))
    {
        chassis.remote_self_save_latched = 1U;
        Chassis_SetMode(CHASSIS_MODE_SELF_SAVE);
    }

    previousModeRequest = modeRequest;
}

/**
 * @brief 从 IMU、DM、DJI 和 CAN 任务读取本轮底盘反馈。
 */
static void Chassis_FeedbackUpdate(void)
{
    task_imu_state_t imu_state = {0};
    task_remote_state_t remote_state = {0};
    uint32_t now_tick = HAL_GetTick();
    uint32_t index;

    /* IMU任务已经完成传感器坐标到整车右手系的转换。 */
    IMU_Task_GetState(&imu_state);
    chassis.imu.initialized = imu_state.isInitialized;
    chassis.imu.attitude_ready = imu_state.isAttitudeReady;
    chassis.imu.error_code = imu_state.lastErrorCode;
    chassis.imu.roll_rad = imu_state.rollRad;
    chassis.imu.pitch_rad = imu_state.pitchRad;
    chassis.imu.yaw_rad = imu_state.yawRad;
    chassis.imu.yaw_total_rad = imu_state.yawTotalRad;
    memcpy(chassis.imu.gyro_radps,
           imu_state.filteredGyroRadps,
           sizeof(chassis.imu.gyro_radps));
    memcpy(chassis.imu.motion_accel_mps2,
           imu_state.motionAccMps2,
           sizeof(chassis.imu.motion_accel_mps2));

    /* 遥控输入在任务层转换为模式和物理目标，不接触LQR或电机输出。 */
    Remote_Task_GetState(&remote_state);
    Chassis_SetRemoteInput(&remote_state.input, remote_state.online);

    /* DM状态保留最后一次反馈值，online只表示本周期是否超时。 */
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t motor_state = {0};

        Motor_DM_GetState((motor_dm_index_t)index, &motor_state);
        chassis.dm_motor[index].online =
            Motor_DM_IsOnline((motor_dm_index_t)index, now_tick);
        chassis.dm_motor[index].position_rad = motor_state.positionRad;
        chassis.dm_motor[index].speed_radps = motor_state.velocityRadps;
        chassis.dm_motor[index].torque_nm = motor_state.torqueNm;
    }

    /* DJI轮电机同样分开保存反馈值和在线判定。 */
    for (index = 0U; index < APP_WHEEL_COUNT; index++)
    {
        chassis.wheel_motor[index].online =
            Motor_DJI_IsOnline(&chassisDjiWheels[index], now_tick);
        chassis.wheel_motor[index].speed_rpm = chassisDjiWheels[index].speedRpm;
        chassis.wheel_motor[index].current = chassisDjiWheels[index].currentRaw;
    }

    chassis.can_tx_error_count = CAN_Task_GetTxErrorCount();
}

/**
 * @brief 将底盘最终命令写入 DM 设备层和 DJI CAN 发送缓存。
 */
static void Chassis_CommandSend(void)
{
    uint32_t index;

    /*
     * safe_output是任务发送前的最后安全门。触发后只清最终命令，
     * joint_torque_request_nm和wheel_current_request继续供Watch观察。
     */
    Motor_DM_SetSafe(chassis.safe_output);
    if (chassis.safe_output != 0U)
    {
        memset(chassis.joint_torque_nm,
               0,
               sizeof(chassis.joint_torque_nm));
        memset(chassis.wheel_current,
               0,
               sizeof(chassis.wheel_current));
        Motor_DM_ZeroAll();
    }

    /* 最终关节数组逐项写入DM设备层命令缓存。 */
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_command_t command = {
            .torqueNm = chassis.joint_torque_nm[index],
        };

        Motor_DM_SetCommand((motor_dm_index_t)index, &command);
    }

    /* 更新两类发送缓存后只提交最新命令，不等待ACK或重发旧帧。 */
    CAN_Task_SetDjiCurrent(chassis.wheel_current);
    Motor_DM_UpdateTxFrames();
    CAN_Task_RequestTx();
}

/**
 * @brief 周期执行底盘反馈、状态选择、控制计算和命令发送。
 *
 * DM协议上电使能只用于取得反馈；非零输出仍由底盘输出许可、设备
 * 状态、分路开关和safe_output共同决定。
 */
static void Chassis_TaskEntry(void *argument)
{
    const float tick_sec = 1.0f / (float)osKernelGetTickFreq();
    uint32_t control_last_tick = 0U;
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
    Motor_DM_ZeroAll();
    /*
     * DM 必须进入协议使能状态才会持续反馈。协议使能后仍由 safe 和
     * 底盘最终输出数组双重保证零力矩，不等同于开放底盘动力输出。
     */
    Motor_DM_SetEnable(1U);

    for (;;)
    {
        uint32_t control_tick = osKernelGetTickCount();

        /* 底盘PID、速度Kalman和位移积分只使用底盘自己的实际周期。 */
        if (control_last_tick == 0U)
        {
            chassis.control_dt_s = chassis_config.default_dt_s;
        }
        else
        {
            chassis.control_dt_s =
                (float)(control_tick - control_last_tick) * tick_sec;
            if ((chassis.control_dt_s < chassis_config.min_dt_s) ||
                (chassis.control_dt_s > chassis_config.max_dt_s))
            {
                chassis.control_dt_s = chassis_config.default_dt_s;
            }
        }
        control_last_tick = control_tick;

        /* 反馈 -> 状态选择 -> 控制 -> 电机命令，保持单向数据流。 */
        Chassis_FeedbackUpdate();
        Chassis_ControlUpdateLegState();
        Chassis_ControlUpdateState();

        /* 内部state只决定本周期调用哪条控制链，外部mode不会在此修改。 */
        switch (chassis.state)
        {
        case CHASSIS_STANDING:
            Chassis_ControlLoop();
            break;

        case CHASSIS_FALLEN:
        case CHASSIS_FALLING_TO_STAND:
            Chassis_RecoveryControlLoop();
            break;

        case CHASSIS_BENCH:
            Chassis_BenchControlLoop();
            break;

        case CHASSIS_ZERO_FORCE:
        default:
            Chassis_ZeroOutput();
            break;
        }

        Chassis_CommandSend();

        wake_tick += APP_CTRL_TICKS;
        if ((int32_t)(osKernelGetTickCount() - wake_tick) >= 0)
        {
            wake_tick = osKernelGetTickCount() + APP_CTRL_TICKS;
        }
        (void)osDelayUntil(wake_tick);
    }
}

void Chassis_Task_Init(void)
{
    Chassis_ControlInit();
    previousModeRequest = REMOTE_MODE_NONE;
    yawStickActive = 0U;
    (void)osThreadNew(Chassis_TaskEntry, NULL, &chassis_task_attributes);
}

void Chassis_SetOutputEnable(uint8_t enable)
{
    chassis.enabled = (enable != 0U) ? 1U : 0U;
}

void Chassis_SetMode(chassis_mode_t mode)
{
    if (mode <= CHASSIS_MODE_BENCH)
    {
        chassis.mode = mode;
    }
    else
    {
        chassis.mode = CHASSIS_MODE_ZERO_FORCE;
    }
}
