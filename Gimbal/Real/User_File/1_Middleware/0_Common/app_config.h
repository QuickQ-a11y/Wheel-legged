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
#define APP_DJI_FRIC_COUNT 3U
#define APP_DJI_COUNT 4U
#define APP_IMU_AXIS_COUNT 3U

#define APP_DM_FRAME_LEN 8U
#define APP_DJI_RX_LEN 8U
#define APP_DJI_TX_LEN 8U

/*
 * 发射机构四台 M2006 + C610，【分在两条总线上】，各自有一个 0x200 控制帧：
 *
 *   FDCAN2  三个摩擦轮，电调 ID 1/2/3  —— 云台本地
 *   FDCAN3  拨盘，电调 ID 4           —— 拨盘物理装在底盘上，只能走两板之间那条线
 *
 * 电调 ID 用电调上的 SET 按键标定，反馈 ID = 0x200 + 电调 ID。
 *
 * ⚠ 拨盘那一帧只用到 [6..7]，[0..5] 寻址的 ID 1~3 在 FDCAN3 上不存在，填零无副作用。
 * 反过来摩擦轮那一帧的 [6..7] 也是零。
 */
#define APP_DJI_TX_ID 0x200U
#define APP_DJI_FRIC_1_RX_ID 0x201U  /* 摩擦轮 1，电调 ID 1，FDCAN2。 */
#define APP_DJI_FRIC_2_RX_ID 0x202U  /* 摩擦轮 2，电调 ID 2，FDCAN2。 */
#define APP_DJI_FRIC_3_RX_ID 0x203U  /* 摩擦轮 3，电调 ID 3，FDCAN2。 */
#define APP_DJI_TRIGGER_RX_ID 0x204U /* 拨弹盘，电调 ID 4，FDCAN3。 */

/*
 * C610 控制电流量程：±10000 对应转矩电流 ±10 A（手册 §CAN 通信协议）。
 * ⚠ C620（M3508）是 ±16384 对应 ±20 A，两者不能混用——底盘的轮电机走 C620，
 * 那边是另一份独立的 device_motor_dji.c，别把这个值抄过去。
 */
#define APP_DJI_CURRENT_MAX 10000

/*
 * 转子一圈的编码器计数。反馈里的 encoderRaw 和 speedRpm 都是【转子】口径，
 * 除以各自的减速比才是输出轴。
 */
#define APP_DJI_ENCODER_PER_ROUND 8192

/*
 * ⚠⚠ 四台电机的减速比【不一样】，别再当成一个全局常量：
 *
 *   三个摩擦轮  —— P36 减速箱【已经物理拆掉】，直驱，1:1。
 *                  所以它们的 speedRpm 就是轮子转速，【不要再除 36】。
 *   拨盘        —— P36 减速箱【还在】，36:1。
 *
 * 拨盘算错这个数，"一发"就差 36 倍；摩擦轮算错则是速度环的量纲整体偏 36 倍。
 * 减速比与电机下标的对应关系只在 device_motor_dji.c 的 djiGearRatio[] 里写一次。
 */
#define APP_DJI_FRIC_GEAR_RATIO 1.0f
#define APP_DJI_TRIGGER_GEAR_RATIO 36.0f

/* 算法文件里各自有一份 file-local 的 PI，这份是跨层共用的。 */
#define APP_PI_F 3.14159265358979323846f

/*
 * C620/C610 的 0x200 控制帧按电调 ID 分配字节：ID=n 占 data[2(n-1)] 和 data[2(n-1)+1]。
 * 发送字节位置必须由上面的反馈 ID 推出，不能写死，否则改电调 ID 时只有接收
 * 侧跟着变，会出现读一台电机、控另一台的交叉。
 */
#define APP_DJI_TX_SLOT(rx_id) ((uint8_t)(((rx_id) - APP_DJI_TX_ID - 1U) * 2U))

static_assert((APP_DJI_TX_SLOT(APP_DJI_FRIC_1_RX_ID) + 1U < APP_DJI_TX_LEN) &&
                  (APP_DJI_TX_SLOT(APP_DJI_FRIC_2_RX_ID) + 1U < APP_DJI_TX_LEN) &&
                  (APP_DJI_TX_SLOT(APP_DJI_FRIC_3_RX_ID) + 1U < APP_DJI_TX_LEN) &&
                  (APP_DJI_TX_SLOT(APP_DJI_TRIGGER_RX_ID) + 1U < APP_DJI_TX_LEN),
              "shooter ESC id out of 0x200 frame range");
/*
 * 只要求三个摩擦轮互不相同：它们共用 FDCAN2 的同一帧，撞 ID 会互相覆盖字节。
 * 拨盘在另一条总线上，ID 与摩擦轮重复在协议上是合法的（现取 4 只是为了看总线
 * trace 时不歧义），所以【故意】不把它纳入这条判据。
 * 槽位由 ID 线性推出，ID 互不相同即等价于槽位互不相同。
 */
static_assert((APP_DJI_FRIC_1_RX_ID != APP_DJI_FRIC_2_RX_ID) &&
                  (APP_DJI_FRIC_2_RX_ID != APP_DJI_FRIC_3_RX_ID) &&
                  (APP_DJI_FRIC_1_RX_ID != APP_DJI_FRIC_3_RX_ID),
              "friction ESC ids must differ");

/*
 * 翎控（LK-TECH）MG4005E-i10 拨盘，与 M2006 拨盘二选一（见下方 APP_TRIGGER_MOTOR），
 * 只用 0xA1 转矩闭环命令（协议 V2.3 §20）：串级环在云台板上算、电机只跟电流，
 * 这样拨盘力矩能在固件里硬限幅，不至于把弹丸搅碎。
 *
 *   FDCAN3  电机 ID 1  —— 拨盘装在底盘上，只能走两板之间那条线（与 M2006 拨盘同理）
 *
 * 命令和回复用【同一个】标准帧 ID：0x140 + 电机 ID。电机 ID 在上位机 LK Motor Tool
 * 里设；Broadcast Mode 必须关——多电机 0x280 命令和单电机命令不能同时用。
 * 负载与 M2006 拨盘相同：1 kHz 命令 + 1 kHz 回复 = 2000 帧/s，FDCAN3 合计不变。
 *
 * ⚠ 字节序是【小端】，DJI 的 0x200 帧是大端，两份驱动不能互抄。
 * ⚠ 0xA1 的 iq 不受上位机 Max Torque Current 限制（协议 V2.3 §20 备注），固件限幅
 *   就是唯一的软件限幅；也因此这里没有 DM 那种"寄存器量程必须与固件一致"的陷阱。
 */
#define APP_LK_FRAME_LEN 8U
#define APP_LK_TRIGGER_ID 0x141U /* 0x140 + 电机 ID 1，FDCAN3。 */
#define APP_LK_CMD_TORQUE 0xA1U  /* 转矩闭环命令；回复帧 data[0] 同值。 */
#define APP_LK_IQ_MAX 2000       /* 0xA1 iqControl 协议有效范围 ±2000。 */
#define APP_LK_TIMEOUT_TICKS 50U

/*
 * 下面四个标度，两份官方文档互相矛盾或根本没写，只能实机核对。
 * 2026-09-30 上机手转输出轴核对了后三个，结论与协议 V2.3 都不一样——4005E 是双编码器：
 *
 *   ENCODER_PER_ROUND  ✓ 回复帧的编码器在【输出端】：输出轴转一圈只回绕一次，最大值 65535。
 *                      所以它就是"输出轴一圈的计数"，换算角度【不除】减速比。
 *                      （协议 V2.3 写的"14 bit 电机编码器"不适用于这台。）
 *   ENCODER_DIR        ✓ trigger_angle 与 speedDps 同号，保持 +1。
 *                      留 #ifndef 让 test_motor_lk 翻过来再跑一遍，别删。
 *   GEAR_RATIO         ✓ 回复帧的转速却在【电机端】：输出轴约一圈每秒时 speedDps 三千多。
 *                      所以减速比【只】用来把 speedDps 换到输出轴，角度不用它。
 *   IQ_PER_A           ⚠⚠ 绝对值未核对。MG40-50 手册：反馈 ±2048 ↔ ±16.5 A；协议 V2.3：
 *                      ±2048 ↔ ±33 A，差一倍。取本机手册的值。它决定拨盘力矩限幅里的"A"
 *                      是不是真的 A——核对之前，Shooter.trigger_I_limit 只是个名义值。
 *                      已确认的只有【相对】关系：用手按住拨盘时命令 trigger_I 与读回的
 *                      currentRaw 接近（2026-09-30），即命令和反馈是同一个标度。
 */
#define APP_LK_IQ_PER_A (2048.0f / 16.5f)
#define APP_LK_ENCODER_PER_ROUND 65536
#ifndef APP_LK_ENCODER_DIR
#define APP_LK_ENCODER_DIR (+1.0f)
#endif
#define APP_LK_TRIGGER_GEAR_RATIO 10.0f

/*
 * 拨盘电机选型，编译期二选一（与 APP_REMOTE_BACKEND 同一个做法）：
 *
 *   APP_TRIGGER_M2006   M2006 + C610，电调 ID 4，走 Motor_DJI 的 MOTOR_DJI_TRIGGER
 *   APP_TRIGGER_LK4005  翎控 MG4005E-i10，ID 1（0x141），走 Motor_LK，0xA1 转矩闭环
 *
 * 两者都挂 FDCAN3、都是 1 kHz 命令 + 回复，总线负载相同。没选中的那台设备层照样编译、
 * 照样接在分发层上，只是 Shooter 不读它、也不发它的帧。
 *
 * ⚠ 取值故意从 1 开始：#if 里拼错的宏名会被预处理器当成 0，从 0 编号的话会
 *   悄悄选中其中一个（单测里真这样栽过一次），下面的 #error 也就拦不住。
 * 留 #ifndef 是为了 test_shooter 两种都跑。
 */
#define APP_TRIGGER_M2006 1
#define APP_TRIGGER_LK4005 2
#ifndef APP_TRIGGER_MOTOR
#define APP_TRIGGER_MOTOR APP_TRIGGER_LK4005
#endif
#if (APP_TRIGGER_MOTOR != APP_TRIGGER_M2006) && (APP_TRIGGER_MOTOR != APP_TRIGGER_LK4005)
#error "APP_TRIGGER_MOTOR must be APP_TRIGGER_M2006 or APP_TRIGGER_LK4005"
#endif

/* 拨盘电调每安培对应的原始计数，把 Shooter.trigger_I_limit（A）换算成电调口径。 */
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
#define APP_TRIGGER_RAW_PER_A APP_LK_IQ_PER_A
#else
#define APP_TRIGGER_RAW_PER_A ((float)APP_DJI_CURRENT_MAX / 10.0f) /* C610：±10000 = ±10 A */
#endif

/*
 * 云台两台 DM4310，均挂 FDCAN3，与板间通信共用一条物理总线。
 * 反馈 ID = 0x010 + 电机 ID，两者都在达妙调试助手里设定，改 ID 时要一起核对。
 *
 * 底盘自己的 DM 髋关节 ID 恰好也是 0x001~0x004 / 0x011~0x014，与这里完全撞号，
 * 靠的是 Motor_DM_UpdateFeedback() 先比 bus 再比 feedbackId：那几台挂 FDCAN1/2，
 * 匹配不上 FDCAN3 的帧。同理 Board_UpdateFeedback() 只认板间两个 ID，其余在打
 * 时间戳之前就返回。这两条是共用总线的前提，改分发逻辑时不要破坏。
 *
 * ⚠⚠ FDCAN3 上还挂着【拨盘】：M2006（电调 ID 4）或翎控 MG4005E（0x141），二选一，
 * 见 APP_TRIGGER_MOTOR。拨盘物理装在底盘上却由云台控制，只能走这条两板之间的线。
 * 三方共用一条 1 Mbps 总线，负载已经不宽裕：
 *
 *   2 台 DM x (发+收) x 1 kHz      4000 帧/s
 *   拨盘 (控制帧 + 反馈) x 1 kHz    2000 帧/s   （两种拨盘相同）
 *   板间两帧 x 200 Hz (DIV=5)        400 帧/s
 *   -------------------------------------------
 *   合计                           6400 帧/s   →  71% ~ 86%
 *
 * 标准帧 8 字节无位填充 111 位、最坏填充 135 位，1 Mbps 下上限只有 7400~9000 帧/s。
 * ⚠ 所以 APP_BOARD_SEND_DIV 【不能】再设成 1：那样是 8000 帧/s、最坏 108%，直接超容量。
 * 要再加节点先重算这张表。两块板都会收下 FDCAN3 的全部帧（滤波器是全收），只是各自丢弃。
 * ⚠ CAN_Task_GetTxErrorCount() 【不能】当总线判据：总线层的发送失败在驱动里都是静默丢帧，
 * 不进这个计数（见 gimbal_control.h 的 GIMBAL_FAULT_CAN）。
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
 * 退出使能区间时不再加发、计时复位，已经在转的那一发打完才停（它后面攒的扔掉）。
 *
 * 实现见 shooter_control.c 的拨弹状态机；两个数值【未经实机整定】。
 */
#define APP_SHOOT_BURST_DELAY_MS 500U
#define APP_SHOOT_BURST_HZ 8U

/*
 * 拨盘一圈的弹丸数，由【拨叉构型】决定——换拨叉就改这里，别的地方都从它推。
 *
 * ✓ 2026-09-30 实机核对：拨叉 8 槽，空打单发正好转一个槽位。
 * 换拨叉后填错的现象是单发转不到位或转过头，而摩擦轮一切正常，很容易往速度环上想。
 */
#define APP_SHOOT_AMMO_PER_ROUND 8

/* 一发对应的拨盘【输出轴】转角，rad。不写死，永远由上面那个数推出。 */
#define APP_SHOOT_RAD_PER_AMMO (2.0f * APP_PI_F / (float)APP_SHOOT_AMMO_PER_ROUND)

/*
 * 拨盘送弹方向：+1 表示"命令电流为正 = 送弹"，-1 表示反过来。
 *
 * ✓ 2026-09-30 翎控拨盘实机核对：+1。沿送弹方向手转时 trigger_angle 增大，
 * 空打单发也朝送弹方向转。
 * ⚠ 这个结论只对【翎控】成立。换回 M2006 要重新核对：电机和安装都不同，
 * 正电流的转向不一定一样。核对方法：拨盘【不装弹】空打单发看转向，反了改成 -1.0f；
 * 装弹之后再测的话，方向反了会把弹往回拨。
 *
 * ⚠ 这里留 #ifndef 是为了让 test_shooter 能用 -D 把它翻成 -1 再跑一遍：
 * 不翻过来的话"代码到底有没有乘这个常量"测不出来——DIR 为 +1 时
 * 漏乘和乘了完全一样，而实机上翻这个宏却毫无反应会非常难查。
 */
#ifndef APP_SHOOT_TRIGGER_DIR
#define APP_SHOOT_TRIGGER_DIR (+1.0f)
#endif

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
 * 云台主循环 1 kHz，分频到 200 Hz 发送。
 *
 * ⚠ 这个值曾被改成 1（即 1 kHz、2000 帧/s）。拨盘挂上 FDCAN3 之后**不能再那样**：
 * 加起来是 8000 帧/s、最坏负载 108%，超容量。算式见上面 DM4310 那段注释。
 *
 * 200 Hz 不丢信息：接收机帧率本来只有 DR16 71 Hz / i-BUS 129 Hz / S.BUS 143 Hz，
 * 再快也只是重复发同一份快照。底盘侧 APP_BOARD_TIMEOUT_TICKS = 50，
 * 200 Hz 下等于连丢 10 帧才判离线，余量够。
 */
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
