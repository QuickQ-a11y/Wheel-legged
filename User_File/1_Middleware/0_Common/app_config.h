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
#define APP_DJI_LEFT_RX_ID 0x202U
#define APP_DJI_RIGHT_RX_ID 0x201U

/*
 * C620 的 0x200 控制帧按电调 ID 分配字节：ID=n 占 data[2(n-1)] 和 data[2(n-1)+1]。
 * 发送字节位置必须由上面的反馈 ID 推出，不能写死，否则改电调 ID 时只有接收
 * 侧跟着变，会出现读一个轮子、控另一个轮子的交叉。
 */
#define APP_DJI_TX_SLOT(rx_id) ((uint8_t)(((rx_id) - APP_DJI_TX_ID - 1U) * 2U))

_Static_assert(APP_DJI_TX_SLOT(APP_DJI_LEFT_RX_ID) + 1U < APP_DJI_TX_LEN,
               "left wheel ESC id out of 0x200 frame range");
_Static_assert(APP_DJI_TX_SLOT(APP_DJI_RIGHT_RX_ID) + 1U < APP_DJI_TX_LEN,
               "right wheel ESC id out of 0x200 frame range");
_Static_assert(APP_DJI_TX_SLOT(APP_DJI_LEFT_RX_ID) !=
                   APP_DJI_TX_SLOT(APP_DJI_RIGHT_RX_ID),
               "left and right wheel ESC id must differ");

/*
 * DM 电机 ID 与所在总线，反馈 ID = 0x010 + 电机 ID，两者都在电机上位机里设定。
 *   右前 1、右后 2 -> FDCAN1（与 DJI 轮电调共用该总线，ID 不冲突）
 *   左后 3、左前 4 -> FDCAN2
 * 总线归属在 device_motor_dm.c 的 Motor_DM_Init 中指定，改 ID 时要一起核对。
 */
#define APP_DM_LF_ID 0x004U
#define APP_DM_LB_ID 0x003U
#define APP_DM_RF_ID 0x001U
#define APP_DM_RB_ID 0x002U

#define APP_DM_LF_FB 0x014U
#define APP_DM_LB_FB 0x013U
#define APP_DM_RF_FB 0x011U
#define APP_DM_RB_FB 0x012U

/* DM MIT 控制帧和反馈帧的 16 bit 位置字段映射范围，单位 rad。 */
#define APP_DM_PMIN (-12.5f)
#define APP_DM_PMAX 12.5f

/* 速度单位 rad/s，力矩单位 N*m；范围按当前 J4310 协议配置。 */
#define APP_DM_VEL_MIN (-30.0f)
#define APP_DM_VEL_MAX 30.0f
#define APP_DM_TOR_MIN (-10.0f)
#define APP_DM_TOR_MAX 10.0f

#define APP_CAN_PERIOD_TICKS 1U
/* IMU和底盘控制统一按1 kHz运行，延迟时由各任务使用实际dt补偿。 */
#define APP_CTRL_TICKS 1U
#define APP_CTRL_DT_S 0.001f

#define APP_USB_RX_BUF_SIZE 2048U
#define APP_USB_TX_CAP 32U
#define APP_USB_TX_BATCH 16U
#define APP_USB_WAIT_TICKS 1U
#define APP_USB_STATUS_TICKS 1U

#define APP_REMOTE_WAIT_TICKS 10U
#define APP_REMOTE_TIMEOUT_TICKS 100U
#define APP_REMOTE_SYNC_FRAMES 2U

/* DR16输入整形参数；轴值已经由DBUS解析为约-660..660。 */
#define APP_DR16_DB 10
#define APP_DR16_DIAL 400

/*
 * 与具体遥控协议无关的底盘运动目标。
 * 站立模式下摇杆给的都是速度量：前进速度和偏航角速度；位移和航向目标
 * 由控制层积分得到，松杆后锁位置、锁航向，因此不需要目标斜坡。
 */
#define APP_RC_MAX_VEL 3.0f   /* 满杆前进速度，m/s。 */
#define APP_RC_MAX_YAW 1.0f   /* 满杆偏航角速度，rad/s。 */
#define APP_RC_VEL_RATE 1.0f  /* 爬台阶接近段的速度目标斜率，m/s^2。 */
#define APP_RC_LEG_S 0.08f
#define APP_RC_LEG_M 0.10f
#define APP_RC_LEG_L 0.15f

/*
 * 控制器第一阶段只计算中间状态和安全输出。
 * 该宏保持 0 时，即使控制器配置允许输出，也不会解除零力矩/零电流封锁。
 */
#define APP_CHASSIS_OUTPUT_ENABLE 1U

#define APP_DM_TIMEOUT_TICKS 50U
/* DM 反馈 ERR=0 时重发使能帧的周期，单位 HAL tick。 */
#define APP_DM_EN_RETRY 20U
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
