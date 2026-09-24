#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
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

static_assert(APP_DJI_TX_SLOT(APP_DJI_LEFT_RX_ID) + 1U < APP_DJI_TX_LEN,
               "left wheel ESC id out of 0x200 frame range");
static_assert(APP_DJI_TX_SLOT(APP_DJI_RIGHT_RX_ID) + 1U < APP_DJI_TX_LEN,
               "right wheel ESC id out of 0x200 frame range");
static_assert(APP_DJI_TX_SLOT(APP_DJI_LEFT_RX_ID) !=
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

/*
 * 速度单位 rad/s，力矩单位 N*m；范围按当前 J8009P 协议配置。
 *
 * ⚠⚠ 这三对量程必须与达妙上位机烧进电机的 PMAX/VMAX/TMAX 寄存器【逐位一致】。
 * MIT 帧里位置/速度/力矩都是无量纲整数（力矩是 12 bit），两端各自用自己的量程
 * 还原，所以量程不一致 = 命令和反馈同时被按两者之比缩放。
 *
 * 【本工程电机烧的是 TMAX = 40】，固件必须等于它，不要为了"限力矩"去改小这里
 * ——限力矩是 Chassis_Config.output.joint_T_limit 的职责，那一项才是真实 N*m。
 *
 * ⚠⚠⚠ 这个坑在本工程实际发生过，而且藏了很久（2026-09-12 才定位）：
 * 固件曾长期是 ±15、后来 ±25，而电机一直是 ±40。造成
 *   命令方向：实际输出 = 命令 × (80/固件量程跨度)   ±15 时 2.67 倍、±25 时 1.60 倍
 *   反馈方向：读到值   = 真实 × (固件量程跨度/80)   ±15 时 0.375 倍、±25 时 0.625 倍
 * 而这两个误差【在往返上精确相消】：命令 10 -> 实际 15.99 -> 读回 9.99。
 * 所以把 output.T_joint 和 dm_motor[].torque_nm 摆在一起看永远是对的,
 * 【这个最自然的检查恰恰是唯一查不出本类错配的】。查它只能用绝对力参照：
 * 在轮轴挂一个称过的重物，看 Chassis.ground.force[].F0 的变化量对不对得上 m*g。
 *
 * 后果不止是力矩不准：K 的轮子两行走 DJI 电调、口径一直是对的，而腿摆两行走
 * DM，被同一个系数放大过，所以历史整定里轮子和腿摆的相对增益是失配的。
 */
#define APP_DM_VEL_MIN (-30.0f)
#define APP_DM_VEL_MAX 30.0f
#define APP_DM_TOR_MIN (-40.0f)
#define APP_DM_TOR_MAX 40.0f

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

/* DR16输入整形死区；轴值已经由DBUS解析为约-660..660。 */
#define APP_DR16_DB 10

/*
 * VrA 选模式组的阈值，比较的是归一化后的值（中位对应 0，两端 ±1）。
 * 双阈值滞回：超过 +HYST 进高组、低于 -HYST 进低组，中间带保持当前组。
 * 不加滞回的话旋钮停在中值附近时噪声会让模式反复横跳。
 */
#define APP_RC_KNOB_HYST 0.15f

/*
 * 与具体遥控协议无关的底盘运动目标。
 * 站立模式下摇杆给的都是速度量：前进速度和偏航角速度；位移和航向目标
 * 由控制层积分得到，松杆后锁位置、锁航向，因此不需要目标斜坡。
 */
#define APP_RC_MAX_VEL 2.0f   /* 满杆前进速度，m/s。重整定期的保守值。 */
#define APP_RC_MAX_YAW 2.0f   /* 满杆偏航角速度，rad/s。 */
#define APP_RC_VEL_RATE 1.0f  /* 爬台阶接近段的速度目标斜率，m/s^2。 */
/*
 * 腿长目标。原先拨轮选三档（0.28/0.18/0.38），FS-i6X 把 VrA 征用成模式组选择器后
 * 三档取消：正常行驶恒为 APP_RC_LEG_M，只有高组 SwA 打下去（压缩腿）才换成
 * APP_RC_LEG_CROUCH。APP_RC_LEG_M 就是原来拨轮中位的值，也是上电默认值。
 *
 * ⚠ 名字里的 M 指的是【原拨轮中位档】不是"中等腿长"，它其实是三档里最短的。
 * 留着这个名字是为了不动历史整定数据的口径。
 */
#define APP_RC_LEG_M 0.18f
/*
 * 压缩腿的腿长。现在先等于行驶腿长，即"压缩腿"实际不压——真实压缩量要等
 * 跳跃蓄力那套实现时按实机定，提前填一个猜的值只会让人以为它调过。
 * 取值必须落在 lqr.L0_min = 0.10 和 lqr.L0_max = 0.35 之间，否则 K 拟合会被夹紧
 * 并置 Chassis.lqr.limit_flag。
 */
#define APP_RC_LEG_CROUCH 0.18f

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

/*
 * 板间通信：云台板 -> 底盘板，走 FDCAN3 专用总线，两帧同频发送，
 * 频率由 APP_BOARD_SEND_DIV 分频得到。16 位字段低字节在前，与本工程 USB 协议一致。
 *
 * ⚠ FDCAN3 配的是 FDCAN_FRAME_CLASSIC（Core/Src/fdcan.c），帧长封顶 8 字节。
 * 加字段只能在这 8 个字节里挤，不能指望 FD 的 64 字节——换 FD 要重新过 CubeMX
 * 并重算两条总线的位定时。
 *
 * 0x3 摇杆帧：[0..1]leftX [2..3]leftY [4..5]rightX [6..7]rightY
 *             各 int16 = 归一化值 * APP_BOARD_SCALE。
 * 0x4 状态帧：[0]    拨杆位域，每个拨杆 2 bit，下标即 REMOTE_SW_*：
 *                    SwA bit0-1 / SwB bit2-3 / SwC bit4-5 / SwD bit6-7。
 *                    编码直接就是 Remote_Switch_t 的 0..3，见 remote_input.h。
 *             [1]    flags，见下面的 APP_BOARD_FLAG_*。
 *             [2..3] VrA 旋钮 * APP_BOARD_SCALE，int16。
 *             [4]    seq，底盘据此判丢帧。
 *             [5]    预留，填 0。
 *             [6..7] 云台 YAW 关节角（归一化到 ±pi）* APP_BOARD_SCALE，int16。
 *
 * VrB 不下发：摩擦轮和拨盘纯属云台事务，底盘用不上。
 */
#define APP_BOARD_STICK_ID 0x3U
#define APP_BOARD_STATE_ID 0x4U
#define APP_BOARD_FRAME_LEN 8U

/* 归一化量和角度统一的定点比例；-1..1 正好用满 int16，pi*10000=31416 也不溢出。 */
#define APP_BOARD_SCALE 10000.0f

/* 状态帧拨杆位域：每个拨杆 2 bit。 */
#define APP_BOARD_SW_BITS 2U
#define APP_BOARD_SW_MASK 0x03U

/* 状态帧 flags 位。 */
#define APP_BOARD_FLAG_REMOTE_ONLINE 0x01U
/*
 * 自瞄【实际激活】，不是 SwD 的原始位置——上位机掉线时 SwD 打下去不该让底盘锁航向。
 * 判定在云台侧做，底盘只读结论。
 */
#define APP_BOARD_FLAG_AUTOAIM 0x02U

/* 板间帧超时，单位 HAL tick。 */
#define APP_BOARD_TIMEOUT_TICKS 50U

/* 云台主循环 1 kHz，分频到 200 Hz 发送。 */
#define APP_BOARD_SEND_DIV 5U

typedef enum
{
    APP_CAN_BUS_UNKNOWN = 0,
    APP_CAN_BUS_FDCAN1,
    APP_CAN_BUS_FDCAN2,
    APP_CAN_BUS_FDCAN3,
} app_can_bus_t;

#ifdef __cplusplus
}
#endif

#endif
