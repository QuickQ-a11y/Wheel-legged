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

#define APP_DM_COUNT 2U
#define APP_DJI_COUNT 3U
#define APP_IMU_AXIS_COUNT 3U

#define APP_DM_FRAME_LEN 8U
#define APP_DJI_RX_LEN 8U
#define APP_DJI_TX_LEN 8U

/*
 * 发射机构三台 DJI 电调，全部挂 FDCAN2，共用 0x200 控制帧。
 * 电调 ID 在电调上标定，反馈 ID = 0x200 + 电调 ID。
 */
#define APP_DJI_TX_ID 0x200U
#define APP_DJI_TRIGGER_RX_ID 0x201U /* M2006 拨弹盘，电调 ID 1。 */
#define APP_DJI_FRIC_L_RX_ID 0x202U  /* M3508 左摩擦轮，电调 ID 2。 */
#define APP_DJI_FRIC_R_RX_ID 0x203U  /* M3508 右摩擦轮，电调 ID 3。 */

/*
 * C620/C610 的 0x200 控制帧按电调 ID 分配字节：ID=n 占 data[2(n-1)] 和 data[2(n-1)+1]。
 * 发送字节位置必须由上面的反馈 ID 推出，不能写死，否则改电调 ID 时只有接收
 * 侧跟着变，会出现读一台电机、控另一台的交叉。
 */
#define APP_DJI_TX_SLOT(rx_id) ((uint8_t)(((rx_id) - APP_DJI_TX_ID - 1U) * 2U))

static_assert(APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) + 1U < APP_DJI_TX_LEN,
               "trigger ESC id out of 0x200 frame range");
static_assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_L_RX_ID) + 1U < APP_DJI_TX_LEN,
               "left friction ESC id out of 0x200 frame range");
static_assert(APP_DJI_TX_SLOT(APP_DJI_FRIC_R_RX_ID) + 1U < APP_DJI_TX_LEN,
               "right friction ESC id out of 0x200 frame range");
static_assert((APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) !=
                   APP_DJI_TX_SLOT(APP_DJI_FRIC_L_RX_ID)) &&
                   (APP_DJI_TX_SLOT(APP_DJI_FRIC_L_RX_ID) !=
                        APP_DJI_TX_SLOT(APP_DJI_FRIC_R_RX_ID)) &&
                   (APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) !=
                        APP_DJI_TX_SLOT(APP_DJI_FRIC_R_RX_ID)),
               "shooter ESC ids must differ");

/*
 * 云台两台 DM4310，均挂 FDCAN3，与板间通信共用一条物理总线。
 * 反馈 ID = 0x010 + 电机 ID，两者都在达妙调试助手里设定，改 ID 时要一起核对。
 *
 * 底盘自己的 DM 髋关节 ID 恰好也是 0x001~0x004 / 0x011~0x014，与这里完全撞号，
 * 靠的是 Motor_DM_UpdateFeedback() 先比 bus 再比 feedbackId：那几台挂 FDCAN1/2，
 * 匹配不上 FDCAN3 的帧。同理 Board_UpdateFeedback() 只认板间两个 ID，其余在打
 * 时间戳之前就返回。这两条是共用总线的前提，改分发逻辑时不要破坏。
 *
 * 总线负载：2 台 DM x (发+收) x 1 kHz = 4000 帧/s，加板间 400 帧/s，
 * 1 Mbps 下约 57%。若实机发现云台控制环抖，先把 APP_BOARD_SEND_DIV 调大降频。
 */
#define APP_DM_YAW_ID 0x001U
#define APP_DM_PITCH_ID 0x002U

#define APP_DM_YAW_FB 0x011U
#define APP_DM_PITCH_FB 0x012U

/* ==========================================================================
 * 以下六个值必须与达妙调试助手里两台 DM4310 的实际设定逐位一致，否则反馈帧解出来
 * 的位置、速度、力矩全是错的，发送帧的力矩编码也会错。
 *
 * 位置量程不在这里：它按电机分化（YAW 要连续转、PITCH 不会回绕），已挪进
 * motor_dm_config_t 的 positionMin/positionMax，取值和理由见 Motor_DM_Init。
 *
 * VEL/TOR 四个值已从调试助手读出核对：VMAX = 30、TMAX = 10，与下方一致。
 * 这四个暂不按电机分化——两台数值一致，真出现分化再照位置量程的样子挪过去。
 * ========================================================================== */
/* 速度单位 rad/s，力矩单位 N*m；已与调试助手实读值核对一致。
 * TOR 这一对发送和接收两侧都用（解码反馈力矩 + 编码下发力矩），改动要两头一起想。 */
#define APP_DM_VEL_MIN (-30.0f)
#define APP_DM_VEL_MAX 30.0f
#define APP_DM_TOR_MIN (-10.0f)
#define APP_DM_TOR_MAX 10.0f

#define APP_CAN_PERIOD_TICKS 1U
/* IMU和云台控制统一按1 kHz运行，延迟时由各任务使用实际dt补偿。 */
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

/*
 * 遥控后端选择。三种协议挂同一路 UART5，但帧格式、波特率、校验和反相设置都不同，
 * 所以 task_remote 在启动接收前会按所选后端重新初始化 UART。
 * 换协议只改这一个宏，然后重新编译烧录。
 *
 * SBUS 和 IBUS 用的是【同一台】FS-iA10B 接收机，靠发射机的输出模式菜单
 * （PWM/i-BUS ↔ PPM/S.BUS）切换，切完接收机要重新上电。
 */
#define APP_REMOTE_BACKEND_DR16 0
#define APP_REMOTE_BACKEND_IBUS 1
#define APP_REMOTE_BACKEND_SBUS 2
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_SBUS

/* DR16输入整形死区；轴值已经由DBUS解析为约-660..660。 */
#define APP_DR16_DB 10

/* FS-iA10B 输入整形死区；轴值由 i-BUS 解析为约 -500..500。 */
#define APP_IBUS_DB 8

/*
 * S.BUS 输入整形死区；轴值由 S.BUS 解析为约 -783..783（本机实测量程，
 * 不是 Futaba 惯例的 ±819，理由见 device_sbus.h）。
 * 取 13 是为了和 i-BUS 的 8/500 同比例：0.016 × 783 ≈ 12.5，向上取整。
 */
#define APP_SBUS_DB 13

/*
 * VrB 旋钮控摩擦轮和拨盘，比较的是【归一化后】的值，按旋钮行程的百分比说话：
 *
 *   行程下半      摩擦轮关、拨盘不使能
 *   过中位~98%    摩擦轮开
 *   98%~拧到底    拨盘使能
 *
 * APP_RC_TRIGGER_ON 取 0.979 是"满行程的 98%"，对 i6X 就是 1990 us。
 *
 * ⚠ 这个阈值【与协议无关】，换 i-BUS / S.BUS 都不用动：两个后端的归一化都是
 * "满行程线性映射到 ±1、死区按比例扣除"，同一个物理旋钮位置算出来的值相同。
 * 两边的量程完全不同，但触发点占行程的百分比一样（末端 2.07%）：
 *   i-BUS：原始 1989.7 起触发，端点 2000（量程 1500±500，死区 8）
 *   S.BUS：原始 1790.8 起触发，端点 1807（量程 1024±783，死区 13）
 *
 * ⚠ 触发带只有行程末端 2%，靠的是 VrB 拧到底确实能到端点。
 * i-BUS（到 2000）和 S.BUS（到 1807）都已实机确认能读到端点。
 * 换接收机或改发射机行程后要重新确认，否则拨盘永远使能不了。
 */
#define APP_RC_FRIC_ON 0.0f
#define APP_RC_TRIGGER_ON 0.979f

/*
 * 单发/连发的时间参数。语义：进入使能区间立刻发一发；持续保持超过
 * APP_SHOOT_BURST_DELAY_MS 后转连发，按 APP_SHOOT_BURST_HZ 连续发射；
 * 退出使能区间时计时复位。
 *
 * ⚠ 只是把名字和数值占下来，发射控制逻辑本轮【没有实现】——
 * Gimbal.I_dji_req[] 至今只被 memset。见 Codex文档/遥控键位与模式分配.md。
 */
#define APP_SHOOT_BURST_DELAY_MS 500U
#define APP_SHOOT_BURST_HZ 8U

/*
 * 控制器第一阶段只计算中间状态和安全输出。
 * 该宏保持 0 时，即使控制器配置允许输出，也不会解除零力矩/零电流封锁。
 */
#define APP_GIMBAL_OUTPUT_ENABLE 1U

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

/*
 * 云台主循环 1 kHz，不分频。上面那段总线负载估算里的"板间 400 帧/s"
 * 对应的是 DIV=5，这里取 1 就是 2000 帧/s，负载相应上去。
 * 云台控制环抖的话先调大这个值。
 */
#define APP_BOARD_SEND_DIV 1U

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
