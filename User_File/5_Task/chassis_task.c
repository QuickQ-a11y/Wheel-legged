#include "chassis_task.h"

#include "app_config.h"
#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "task_can.h"
#include "task_can_dispatch.h"
#include "task_imu.h"

#include "cmsis_os2.h"

#include <string.h>

static const osThreadAttr_t chassis_task_attributes = {
    .name = "ChassisTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 从 IMU、DM、DJI 和 CAN 任务读取本轮底盘反馈。
 */
static void chassis_feedback_update(void)
{
    task_imu_state_t imu_state = {0};
    uint32_t now_tick = HAL_GetTick();
    uint32_t index;

    IMU_Task_GetState(&imu_state);
    chassis.imu.initialized = imu_state.isInitialized;
    chassis.imu.attitude_ready = imu_state.isAttitudeReady;
    chassis.imu.error_code = imu_state.lastErrorCode;
    chassis.imu.roll_rad = imu_state.rollRad;
    chassis.imu.pitch_rad = imu_state.pitchRad;
    chassis.imu.yaw_rad = imu_state.yawRad;
    memcpy(chassis.imu.gyro_radps,
           imu_state.filteredGyroRadps,
           sizeof(chassis.imu.gyro_radps));
    memcpy(chassis.imu.motion_accel_mps2,
           imu_state.motionAccMps2,
           sizeof(chassis.imu.motion_accel_mps2));

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
static void chassis_cmd_send(void)
{
    uint32_t index;

    Motor_DM_SetSafe(chassis.safe_output);
    if (chassis.safe_output != 0U)
    {
        Motor_DM_ZeroAll();
    }

    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_command_t command = {
            .torqueNm = chassis.joint_torque_nm[index],
        };

        Motor_DM_SetCommand((motor_dm_index_t)index, &command);
    }

    CAN_Task_SetDjiCurrent(chassis.wheel_current);
    Motor_DM_UpdateTxFrames();
    CAN_Task_RequestTx();
}

static void chassis_task(void *argument)
{
    const float tick_sec = 1.0f / (float)osKernelGetTickFreq();
    uint32_t control_last_tick = 0U;
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    Motor_DM_Init();
    Motor_DM_SetEnable(0U);

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
        chassis_feedback_update();
        chassis_control_update_leg_state();
        chassis_control_update_state();

        switch (chassis.state)
        {
        case CHASSIS_STANDING:
            chassis_control_loop();
            break;

        case CHASSIS_FALLEN:
        case CHASSIS_FALLING_TO_STAND:
            chassis_recovery_control_loop();
            break;

        case CHASSIS_BENCH:
            chassis_bench_control_loop();
            break;

        case CHASSIS_ZERO_FORCE:
        default:
            chassis_zero_output();
            break;
        }

        chassis_cmd_send();

        wake_tick += APP_CTRL_TICKS;
        if ((int32_t)(osKernelGetTickCount() - wake_tick) >= 0)
        {
            wake_tick = osKernelGetTickCount() + APP_CTRL_TICKS;
        }
        (void)osDelayUntil(wake_tick);
    }
}

void chassis_task_init(void)
{
    chassis_control_init();
    (void)osThreadNew(chassis_task, NULL, &chassis_task_attributes);
}

void chassis_set_enable(uint8_t enable)
{
    chassis.enabled = (enable != 0U) ? 1U : 0U;
    Motor_DM_SetEnable(chassis.enabled);
}

void chassis_set_mode(chassis_mode_t mode)
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
