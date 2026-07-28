#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_CAN_DATA_MAX_BYTES 8U
#define APP_CAN_STD_ID_MAX 0x7FFU
#define APP_CAN_TX_CAP 16U
#define APP_CAN_RX_QUEUE_LEN 32U

#define APP_DM_COUNT 4U
#define APP_WHEEL_COUNT 2U
#define APP_IMU_AXIS_COUNT 3U

#define APP_DM_FRAME_LEN 8U
#define APP_DJI_RX_LEN 8U
#define APP_DJI_TX_LEN 8U

#define APP_DJI_TX_ID 0x200U
#define APP_DJI_LEFT_RX_ID 0x201U
#define APP_DJI_RIGHT_RX_ID 0x202U

#define APP_DM_LEFT_FRONT_TX_ID 0x002U
#define APP_DM_LEFT_FRONT_RX_ID 0x002U
#define APP_DM_LEFT_BACK_TX_ID 0x001U
#define APP_DM_LEFT_BACK_RX_ID 0x001U
#define APP_DM_RIGHT_FRONT_TX_ID 0x004U
#define APP_DM_RIGHT_FRONT_RX_ID 0x004U
#define APP_DM_RIGHT_BACK_TX_ID 0x003U
#define APP_DM_RIGHT_BACK_RX_ID 0x003U

#define APP_DM_LEFT_DIR 1
#define APP_DM_RIGHT_DIR (-1)

/* 当前髋关节电机 MIT 协议的线性映射量程，不是机械安全限幅。 */
#define APP_DM_POS_MIN_RAD (-12.5f)
#define APP_DM_POS_MAX_RAD 12.5f
#define APP_DM_VEL_MIN_RADPS (-45.0f)
#define APP_DM_VEL_MAX_RADPS 45.0f
#define APP_DM_TORQUE_MIN_NM (-40.0f)
#define APP_DM_TORQUE_MAX_NM 40.0f

#define APP_CAN_PERIOD_TICKS 1U
/* IMU和底盘控制统一按1 kHz运行，延迟时由各任务使用实际dt补偿。 */
#define APP_CTRL_TICKS 1U
#define APP_CTRL_DT_S 0.001f

#define APP_USB_RX_BUF_SIZE 2048U
#define APP_USB_TX_CAP 32U
#define APP_USB_TX_BATCH 16U
#define APP_USB_WAIT_TICKS 1U
#define APP_USB_STATUS_TICKS 1U

/*
 * 控制器第一阶段只计算中间状态和安全输出。
 * 该宏保持 0 时，即使控制器配置允许输出，也不会解除零力矩/零电流封锁。
 */
#define APP_CHASSIS_OUTPUT_ENABLE 0U

#define APP_DM_TIMEOUT_TICKS 50U
#define APP_DJI_TIMEOUT_TICKS 50U
#define APP_CAN_TX_ERROR_MAX 1000U

#define APP_IMU_INIT_RETRY_TICKS 100U
#define APP_IMU_SPI_TIMEOUT_MS 2U
#define APP_IMU_BIAS_SAMPLES 200U
#define APP_IMU_ACCEL_LPF_S 0.01f
#define APP_IMU_EKF_QUAT_NOISE 10.0f
#define APP_IMU_EKF_BIAS_NOISE 0.001f
#define APP_IMU_EKF_ACCEL_NOISE 10000000.0f
#define APP_IMU_EKF_QUAT_COV 100000.0f
#define APP_IMU_EKF_BIAS_COV 10000.0f
#define APP_IMU_EKF_ACCEL_LPF_S 0.0085f
#define APP_IMU_EKF_ACCEL_MIN_MPS2 9.3f
#define APP_IMU_EKF_ACCEL_MAX_MPS2 10.3f
#define APP_IMU_EKF_GYRO_STABLE_RADPS 0.3f
#define APP_IMU_EKF_BIAS_CORR_RADPS 0.00001f

#define APP_IMU_TEMP_TARGET_C 45.0f
#define APP_IMU_TEMP_PROTECT_C 48.0f
#define APP_IMU_TEMP_STABLE_C 1.0f
#define APP_IMU_TEMP_PWM_MAX 1500.0f
#define APP_IMU_TEMP_KP 100.0f
#define APP_IMU_TEMP_KI 25.0f
#define APP_IMU_TEMP_KD 0.0f
/* 公共 PID 的 integralLimit 限制的是积分状态，20 * ki=500，对应 SPR 的积分输出限幅。 */
#define APP_IMU_TEMP_I_LIMIT 20.0f

#define APP_IMU_Z_BIAS_LPF_S 10.0f
#define APP_IMU_Z_BIAS_GYRO_MAX_RADPS 0.3f

typedef enum
{
    APP_CAN_BUS_UNKNOWN = 0,
    APP_CAN_BUS_FDCAN1,
    APP_CAN_BUS_FDCAN2,
} app_can_bus_t;

#ifdef __cplusplus
}
#endif

#endif
