#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Kalman.h"
#include "chassis_config.h"
#include "chassis_observer.h"
#include "chassis_vmc.h"
#include "remote_input.h"

#include <stdint.h>

#define CHASSIS_FAULT_NONE 0x00000000UL
#define CHASSIS_FAULT_DISABLED 0x00000001UL       /* 非零输出许可关闭。 */
#define CHASSIS_FAULT_IMU 0x00000002UL            /* IMU未就绪或报错。 */
#define CHASSIS_FAULT_DM_MOTOR 0x00000004UL       /* 至少一个髋关节离线。 */
#define CHASSIS_FAULT_DJI_MOTOR 0x00000008UL      /* 至少一个轮电机离线。 */
#define CHASSIS_FAULT_CAN 0x00000010UL            /* CAN发送错误超过阈值。 */
#define CHASSIS_FAULT_CONTROL 0x00000020UL        /* 姿态、配置或计算保护。 */
#define CHASSIS_FAULT_KINEMATICS 0x00000040UL     /* 五连杆状态、逆解或力映射无效。 */
#define CHASSIS_FAULT_RECOVERY_TIMEOUT 0x00000080UL /* 恢复动作阶段超时。 */
#define CHASSIS_FAULT_REMOTE 0x00000100UL         /* 遥控器离线或收到急停请求。 */
#define CHASSIS_FAULT_DM_ERROR 0x00000400UL       /* 至少一个髋关节报错误状态。 */

/* 外部请求模式：表示操作者想让底盘执行的行为。 */
typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0, /* 主动请求零力矩。 */
    CHASSIS_MODE_FOLLOW,         /* 正常平衡和跟随模式。 */
    CHASSIS_MODE_TOP,            /* 小陀螺旋转和二维平移投影。 */
    CHASSIS_MODE_SELF_SAVE,      /* 触发重新站立动作链。 */
    CHASSIS_MODE_BENCH,          /* 保持独立小板凳姿态。 */
    CHASSIS_MODE_STEP,           /* 正向辅助爬台阶。 */
    CHASSIS_MODE_COUNT,
} Chassis_Mode_t;

/* 内部执行状态：表示本周期任务应调用哪一条控制流程。 */
typedef enum
{
    CHASSIS_STANDING = 0,        /* 十维LQR和VMC站立控制。 */
    CHASSIS_ZERO_FORCE,          /* 最终关节力矩和轮电流清零。 */
    CHASSIS_FALLEN,              /* 倒地后的转腿阶段。 */
    CHASSIS_FALLING_TO_STAND,    /* 收腿到小板凳并等待稳定。 */
    CHASSIS_BENCH,               /* 独立板凳位置环和轮LQR。 */
    CHASSIS_STEP,                /* 辅助爬台阶状态机。 */
} Chassis_State_t;

/*
 * 爬台阶五阶段。MOTION1/MOTION2 取自ZJU的Onestep子状态机：先后摆蓄势并把
 * 腿伸到中位把前轮送上台阶沿，再收腿到最短并转竖直把机体拉上去。
 * 这两段走关节位置串级(Joint_Control)，不经过LQR，轮力矩全程为零。
 */
typedef enum
{
    CHASSIS_STEP_PREPARE = 0, /* 伸腿到行驶高度，等待接近。 */
    CHASSIS_STEP_APPROACH,    /* 主动前进直到判定撞上台阶。 */
    CHASSIS_STEP_MOTION1,     /* 后摆蓄势并伸腿到中位。 */
    CHASSIS_STEP_MOTION2,     /* 收腿到最短并转到竖直向下。 */
    CHASSIS_STEP_RECOVER,     /* 恢复腿长和腿摆角，交接回下一级。 */
} Chassis_Step_Phase_t;

typedef struct
{
    uint8_t init_flag;           /* BMI088硬件初始化完成。 */
    uint8_t attitude_flag;       /* 零偏和姿态估计已经可用。 */
    uint32_t error_code;         /* IMU任务最近错误码。 */
    float roll;                  /* 整车右手系横滚角，rad。 */
    float pitch;                 /* 整车右手系俯仰角，rad。 */
    float yaw_total;             /* 本次上电期间连续偏航角，rad。 */
    float gyro[APP_IMU_AXIS_COUNT];       /* rad/s。 */
    float body_accel[APP_IMU_AXIS_COUNT]; /* 整车坐标运动加速度，m/s^2。 */
    float accel[APP_IMU_AXIS_COUNT];      /* 导航坐标运动加速度，m/s^2。 */
    /*
     * BMI088传感器原始坐标加速度，含重力，m/s^2。上面三个都是扣掉重力的
     * 运动加速度，判不了倒地姿态；这一份不做轴向镜像、不扣重力，只供
     * Chassis.fall_pitch 用。轴符号见 Chassis_Config.imu.fall_accel_*_scale。
     */
    float accel_raw[APP_IMU_AXIS_COUNT];
} Chassis_IMU_t;

typedef struct
{
    uint8_t online_flag;         /* 最近超时窗口内收到反馈。 */
    uint8_t err_state;           /* DM反馈状态位：0失能 1使能 8~E错误。 */
    float position_rad;          /* 连续展开电机角，单位 rad。 */
    float speed_radps;           /* 电机角速度，单位 rad/s。 */
    float torque_nm;             /* 电机反馈力矩，单位 N*m。 */
} Chassis_DM_Motor_t;

typedef struct
{
    uint8_t online_flag;         /* 最近超时窗口内收到反馈。 */
    int16_t speed_rpm;           /* DJI原始转速反馈，单位 rpm。 */
    int16_t current;             /* DJI原始电流反馈。 */
} Chassis_DJI_Motor_t;

/** @brief 任务层发布给正常站立控制的遥控运动目标。 */
typedef struct
{
    float d_s;               /* 期望前进速度，m/s。 */
    float d_y;               /* 小陀螺参考系横向速度，m/s。 */
    float d_fai;             /* 期望偏航角速度，rad/s。 */
    float L0;                /* 左右对称目标腿长，m。 */
    float bench_d_L0[CHASSIS_LEG_COUNT];   /* 板凳单腿腿长调节速率，m/s。 */
    float bench_d_phi0[CHASSIS_LEG_COUNT]; /* 板凳单腿腿角调节速率，rad/s。 */
} Chassis_Goal_t;

/** @brief 整车平动和姿态状态，名称与十维模型保持一致。 */
typedef struct
{
    float s;
    float d_s_raw;
    float d_s;
    float dd_s;
    float dd_s_fused;
    float fai;
    float d_fai;
    float wheel_speed[CHASSIS_LEG_COUNT]; /* 轮轴角速度，rad/s。 */
    float side_speed[CHASSIS_LEG_COUNT];  /* 单侧轮腿速度估计，m/s。 */
} Chassis_Body_t;

/** @brief 十维状态、目标、增益和四路广义输出。 */
typedef struct
{
    float x[CHASSIS_STATE_COUNT];
    float target[CHASSIS_STATE_COUNT];
    float error[CHASSIS_STATE_COUNT]; /* 限幅后实际进K点乘的误差。 */
    float scale[CHASSIS_STATE_COUNT];
    uint8_t limit_flag;               /* 拟合腿长被L0_min/L0_max夹紧。 */
} Chassis_LQR_t;

/** @brief 控制请求与通过安全门后的实际电机命令。 */
typedef struct
{
    float target_angle[APP_DM_COUNT];     /* rad。 */
    float target_speed[APP_DM_COUNT];     /* rad/s。 */
    float T_wheel[APP_WHEEL_COUNT];       /* LQR 轮力矩，N*m。 */
    float T_joint_req[APP_DM_COUNT];      /* N*m。 */
    int16_t I_wheel_req[APP_WHEEL_COUNT];
    float T_joint[APP_DM_COUNT];          /* N*m。 */
    int16_t I_wheel[APP_WHEEL_COUNT];
    uint8_t safe_flag;
} Chassis_Output_t;

struct Chassis
{
    /* 外部意图和内部状态机。 */
    uint8_t enable_flag;           /* 非零电机输出许可，不控制 DM 协议使能。 */
    Chassis_Mode_t mode;
    Chassis_Mode_t last_mode;
    Chassis_State_t state;

    /* 任务层每周期写入的设备反馈快照。 */
    Chassis_IMU_t imu;
    Chassis_DM_Motor_t dm_motor[APP_DM_COUNT];
    Chassis_DJI_Motor_t wheel_motor[APP_WHEEL_COUNT];
    uint8_t remote_online_flag;     /* 当前遥控输入后端处于在线状态。 */
    uint8_t remote_stop_flag;       /* 急停请求，只封锁最终电机输出。 */
    uint8_t yaw_stick_flag;         /* 航向摇杆已离开中位，供松杆边沿锁存航向。 */
    uint32_t can_error_count;
    float dt;                       /* 本轮实际控制周期，s。 */
    /*
     * 重力矢量给出的整车俯仰角，rad，范围±pi、倒过90度也不折返。
     * imu.pitch来自EKF的asinf，上界就是pi/2，车真趴下去以后会折返回来，
     * 符号不再代表倒地方向。凡是"我现在是不是躺着/往哪边躺"的判断都用这一份，
     * 站立控制和站立中判倒仍然用imu.pitch。由Chassis_State_Update()每周期更新。
     */
    float fall_pitch;

    Chassis_Goal_t goal;

    /* 物理状态、十维LQR和观测中间量。 */
    Chassis_Leg_t leg[CHASSIS_LEG_COUNT];
    Chassis_Body_t body;
    Chassis_LQR_t lqr;

    /*
     * 五套互不依赖的只读观测，各自拥有一份状态。
     * 前四套完全不参与控制；leso的扰动估计是唯一可按leso.comp_scale接入控制的一路，
     * 接入与否由leso.gate_flag表达，观测器自身仍然只写自己的结构体。
     */
    Chassis_Slip_t slip;
    Chassis_Ground_t ground;
    Chassis_Turn_t turn;
    Chassis_Stuck_t stuck;
    Chassis_Leso_t leso;

    /* 小陀螺与爬台阶模式的最小跨周期状态。 */
    float top_fai;
    float top_d_s;
    uint8_t top_exit_flag;          /* 刚退出小陀螺，角速度目标正在斜坡收敛，尚未交回摇杆。 */
    float step_fai;
    Chassis_Step_Phase_t step_phase;
    float contact_time[CHASSIS_LEG_COUNT];
    uint8_t step_contact_flag[CHASSIS_LEG_COUNT];
    uint8_t step_contact_latch_flag[CHASSIS_LEG_COUNT];
    /*
     * 姿态路碰撞判据的结论，整车级。轮力矩路和姿态路是并联的，
     * 这一位单独记录后者，Watch里才分得清是哪一路把车切进MOTION1的。
     */
    uint8_t step_posture_flag;

    /*
     * 整车离地动作的跨周期状态。起飞那一拍把腿长目标锁进latch，落地那一拍
     * 恢复回去——空中把腿推出去了，不还原就等于永久改了车身高度指令。
     */
    uint8_t last_all_off_flag;
    float off_ground_L0_latch[CHASSIS_LEG_COUNT];

    /* 恢复、板凳和站立控制共同使用的目标与请求量。 */
    float state_time;
    float stable_time;
    float recovery_stuck_time;      /* 转腿卡死条件已连续满足的时间，s。 */
    /*
     * 本次自救锁存的目标腿摆角，rad，0表示本轮还没锁存。符号由进入FALLEN
     * 那一拍的倒地方向决定，之后不再跟着姿态变——机体转过竖直位时
     * fall_pitch 会过零，每拍重算会让参考角来回翻符号、腿原地抖。
     */
    float recovery_theta_ref;
    float recovery_direction;       /* 当前转腿扫掠方向，+1或-1，0表示还没锁存，卡死时反号。 */
    Chassis_Output_t output;
    uint32_t fault;

    /* 需要跨控制周期保存的滤波器和PID状态。 */
    algorithm_kalman_t speed_kalman;
    algorithm_pid_state_t leg_length_pid;   /* 共模：车身高度，只此一份。 */
    algorithm_pid_state_t roll_pid;
    algorithm_pid_state_t joint_angle_pid[APP_DM_COUNT];
    algorithm_pid_state_t joint_speed_pid[APP_DM_COUNT];
};

/* 底盘唯一实际状态，也是 Watch 窗口的长期调试入口。 */
extern Chassis_t Chassis;

/**
 * @brief 初始化底盘控制状态、PID 和速度卡尔曼滤波器。
 */
void Chassis_Init(void);

/**
 * @brief 由本轮原始关节反馈更新左右五连杆状态。
 */
void Chassis_Leg_Update(void);

/**
 * @brief 根据模式边沿和当前姿态选择控制状态并维护输出故障。
 */
void Chassis_State_Update(void);

/**
 * @brief 执行一轮十维 LQR、支撑力和 VMC 控制计算。
 */
void Chassis_Control(void);

/**
 * @brief 执行倒地转腿和小板凳准备两段式重新站立控制。
 */
void Chassis_Recovery(void);

/**
 * @brief 使用关节位置控制保持小板凳姿态，并由当前十维 LQR 控制双轮。
 */
void Chassis_Bench(void);

/**
 * @brief 执行辅助爬台阶四阶段状态机并复用正常LQR和VMC控制链。
 */
void Chassis_Step(void);

/**
 * @brief 将底盘电机命令清零并重置运动融合状态。
 */
void Chassis_Zero_Output(void);

#ifdef __cplusplus
}
#endif

#endif
