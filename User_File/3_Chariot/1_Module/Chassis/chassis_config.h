#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "LQR.h"
#include "PID.h"

#include <stdint.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_HALF_PI 1.57079632679489661923f
#define CHASSIS_STATE_COUNT 10U   /* 十维整车LQR状态数量。 */
#define CHASSIS_STATE_MPC_COUNT 4U /* MPC状态数：alpha, d_alpha, H, d_H。 */
#define CHASSIS_MPC_INPUT_COUNT 2U /* MPC输入数：左右腿支撑力。 */
#define CHASSIS_OUTPUT_COUNT 4U   /* 左右轮力矩和左右腿摆力矩。 */

/* 左右腿数组下标，顺序必须与LQR和电机映射保持一致。 */
typedef enum
{
    CHASSIS_LEFT = 0,
    CHASSIS_RIGHT,
    CHASSIS_LEG_COUNT,
} Chassis_Side_t;

/* 单腿两根主动杆按五连杆共同符号定义的关节下标。 */
typedef enum
{
    CHASSIS_JOINT_PHI1 = 0,
    CHASSIS_JOINT_PHI4,
    CHASSIS_JOINT_COUNT,
} Chassis_Joint_t;

typedef enum
{
    CHASSIS_STATE_S = 0,       /* 整车水平位移 s，单位 m。 */
    CHASSIS_STATE_D_S,         /* 整车水平速度 d_s，单位 m/s。 */
    CHASSIS_STATE_FAI,         /* 整车偏航角 fai，单位 rad。 */
    CHASSIS_STATE_D_FAI,       /* 整车偏航角速度 d_fai，单位 rad/s。 */
    CHASSIS_STATE_THETA_L,     /* 左虚拟腿倾角 theta_l，单位 rad。 */
    CHASSIS_STATE_D_THETA_L,   /* 左虚拟腿倾角速度 d_theta_l，单位 rad/s。 */
    CHASSIS_STATE_THETA_R,     /* 右虚拟腿倾角 theta_r，单位 rad。 */
    CHASSIS_STATE_D_THETA_R,   /* 右虚拟腿倾角速度 d_theta_r，单位 rad/s。 */
    CHASSIS_STATE_THETA_B,     /* 机体俯仰角 theta_b，单位 rad。 */
    CHASSIS_STATE_D_THETA_B,   /* 机体俯仰角速度 d_theta_b，单位 rad/s。 */
} Chassis_State_Index_t;

typedef enum
{
    CHASSIS_OUTPUT_LEFT_WHEEL = 0,
    CHASSIS_OUTPUT_RIGHT_WHEEL,
    CHASSIS_OUTPUT_LEFT_LEG,
    CHASSIS_OUTPUT_RIGHT_LEG,
} Chassis_Output_Index_t;

/** @brief 单侧五连杆的机械尺寸。 */
typedef struct
{
    float l1;                         /* phi1主动杆长度，m。 */
    float l2;                         /* phi1侧从动杆长度，m。 */
    float l3;                         /* phi4侧从动杆长度，m。 */
    float l4;                         /* phi4主动杆长度，m。 */
    float l5;                         /* 两个主动杆输出铰点间距，m。 */
} Chassis_Geometry_Config_t;

/** @brief 关节电机索引以及电机量到几何量的符号映射。 */
typedef struct
{
    uint8_t motor_index;              /* 在DM反馈和命令数组中的索引。 */
    float angle_offset_rad;           /* 电机零位对应的几何角偏置，单位 rad。 */
    float scale;                      /* 电机与几何关节的方向，取+1或-1。 */
    float ratio;                      /* 几何关节角/电机角传动比。 */
} Chassis_Joint_Config_t;

/** @brief 单腿机械、关节映射和正常站立目标腿长。 */
typedef struct
{
    Chassis_Geometry_Config_t geometry;
    Chassis_Joint_Config_t joint[CHASSIS_JOINT_COUNT];
    float target_L0;                  /* 正常站立目标腿长，m。 */
} Chassis_Leg_Config_t;

/** @brief IMU任务输出到十维控制模型坐标的轴选择和符号比例。 */
typedef struct
{
    uint8_t pitch_rate_axis;          /* pitch角速度在gyro数组中的下标。 */
    uint8_t roll_rate_axis;           /* roll角速度在gyro数组中的下标。 */
    uint8_t yaw_rate_axis;            /* yaw角速度在gyro数组中的下标。 */
    float pitch_angle_scale;
    float pitch_rate_scale;
    float roll_angle_scale;
    float roll_rate_scale;
    float yaw_angle_scale;
    float yaw_rate_scale;
    uint8_t forward_accel_axis;       /* 前向运动加速度数组下标。 */
    float forward_accel_scale;        /* IMU前向加速度到整车X正方向的比例。 */
    uint8_t lateral_accel_axis;       /* 横向运动加速度数组下标，供转向观测。 */
    uint8_t vertical_accel_axis;      /* 竖直运动加速度数组下标，供支撑力观测。 */
    /*
     * 倒地方向判据的原始加速度轴符号，只被 Chassis.fall_pitch 消费：
     *   fall_pitch = atan2(x_scale * accel_raw[0], z_scale * accel_raw[2])
     * 默认 1.0/1.0 由现有代码推出：QuaternionEKF 内部用
     * pitch = atan2(-ax, sqrt(ay^2+az^2))，说明内部系直立静止时
     * accel_raw ≈ (0, 0, +9.8)；task_imu 发布到整车系时 pitch 再取反，
     * 两步合起来等价的全角形式就是 atan2(+ax, +az)。
     * 换IMU或改安装方向必须重新标定：直立时 fall_pitch≈0，小角度下与
     * Chassis.imu.pitch 同号同值，且倒过90度后单调不折返。
     */
    float fall_accel_x_scale;
    float fall_accel_z_scale;
} Chassis_IMU_Config_t;

/** @brief 轮组几何、反馈极性和轮力矩到DJI电流值的换算。 */
typedef struct
{
    float R;               /* 轮半径，m。 */
    float half_track;      /* 半轮距，m。 */
    float gear_ratio;      /* 电机转子转速/轮轴转速传动比，直驱为1。 */
    float left_scale;      /* 左轮反馈和命令到模型正方向的比例。 */
    float right_scale;     /* 右轮反馈和命令到模型正方向的比例。 */
    float T_limit;         /* 单轮力矩限幅，N*m。轮通道唯一限幅点。 */
    float T_to_I;          /* 轮力矩绝对方向到DJI原始电流的比例，已计入gear_ratio。 */
} Chassis_Wheel_Config_t;

/** @brief 前进速度和加速度二维Kalman滤波配置。 */
typedef struct
{
    uint8_t enable_flag;
    float initial_covariance[4];
    float process_noise[4];
    float measurement_noise[4];
    float position_d_s_limit;   /* 允许积分前进位移的速度上限，单位 m/s。 */
} Chassis_Kalman_Config_t;

/** @brief 分路输出开关和最终关节力矩限幅。 */
typedef struct
{
    uint8_t joint_flag;
    uint8_t wheel_flag;
    /*
     * 整车离地后的三项动作总开关：悬空腿下压伸腿、悬空侧轮力矩置零、
     * LQR掩码只留腿摆通道。默认关。
     * ⚠ 掩码那一项在 all_off_flag 误触发时会让站着的车瞬间失去位移、速度、
     * 航向和俯仰全部反馈直接倒地。上机前必须先只看Watch跑一轮正常行驶和
     * 小陀螺，确认 all_off_flag 一次都不误触发，再打开这个开关。
     */
    uint8_t off_ground_act_flag;
    /*
     * 用MPC接管车身高度和Roll，替换共模腿长PID+差模横滚PID那两路。默认关。
     * 关闭时MPC仍然每拍照算并写进Watch，只是不参与F0，方便实机对照两路输出。
     */
    uint8_t mpc_flag;
    float joint_T_limit;   /* 关节力矩限幅，N*m。关节通道唯一限幅点。 */
} Chassis_Output_Config_t;

/**
 * @brief 倒地自救、站立姿态保护和板凳模式的全部参数。
 *
 * 消费者有三处：Chassis_Recovery() 跑 TurnOver/Swing/DrawBack 三个阶段并做
 * 卡死反转，Chassis_State_Update() 拿姿态门判断能不能站/什么时候判倒，
 * Chassis_Bench() 用板凳项。姿态门吃的是 Chassis.fall_pitch 不是 imu.pitch。
 */
typedef struct
{
    /*
     * 三阶段的目标姿态。腿长走的是"伸长-伸长-收短"：翻身和摆腿都要长腿
     * （长腿力臂大、转动惯量大，机体才翻得动），最后收到 bench_L0 让轮子
     * 把机体撑起来。ZJU取 lmax=0.331 / lmin=0.10。
     * ⚠ 这三个数决定动作幅度，改之前先架空确认机构可达范围。
     */
    float turnover_L0;       /* 翻身和摆腿阶段的伸长腿长，m。取机构安全最长。 */
    float extend_L0;         /* 摆腿阶段的腿长目标，m。留作与turnover_L0分开整定。 */
    float bench_L0;          /* DrawBack收腿站起的腿长目标，m。取机构安全最短。 */
    float bench_phi0;        /* DrawBack的腿杆角目标，rad。 */

    /*
     * 翻身阶段（ZJU TurnOver）。退出判据用竖向加速度而不是 fall_pitch：
     * 倒地后四元数可能收敛到错误分支，重力方向的加速度投影始终可靠；
     * 而且 az 是标量阈值，没有 fall_pitch 在 ±pi 附近符号跳变的问题。
     */
    float turnover_L0_rate;  /* 翻身段伸腿速率，m/s。要比站立段快，腿先伸开才翻得动。 */
    float turnover_rate;     /* 翻身扫掠速率，rad/s。 */
    float turnover_az_ratio; /* 机体朝上判据：az > 该比例*gravity。ZJU取0.4。 */
    float turnover_hold;     /* 带惩罚计数达到多少秒算翻身成功。 */
    /*
     * 扫掠方向的机构符号，只取 +1 或 -1。哪一侧能把机体撬起来取决于机构，
     * 实机架空试出来填这里，不要回头改 Chassis_Recovery() 里的表达式。
     */
    float turnover_dir_sign;

    /* 收腿站起阶段（ZJU DrawBack）。ZJU用 1.2 m/s 收腿、200度/s 转竖直。 */
    float drawback_L0_rate;   /* 收腿速率，m/s。 */
    float drawback_phi0_rate; /* 摆到竖直向下的速率，rad/s。 */

    /* 目标斜坡：目标只按速率走、不跟随实际角，动作快慢全由这几项决定。 */
    float L0_rate;           /* 腿长目标斜率，m/s。站立段升腿也用它。 */
    float rotate_rate;       /* 腿杆角目标速率，rad/s。 */
    float lag_rate;          /* 双腿进度差过大时落后腿改用的速率，rad/s。 */
    float theta_diff;        /* 触发lag_rate的双腿theta差，rad。 */
    float rotate_lead_max;   /* 目标角领先实际角的上限，rad。腿卡住时防止目标跑飞。 */

    /*
     * 转腿卡死反转。腿转不动时目标一直往同方向推只会顶到超时，
     * 参考ZJU翻身阶段的做法：连续卡住就反向扫，并把目标重锁到当前实际角。
     */
    float stuck_d_phi0;      /* 判定腿卡住的机构角速度上限，rad/s。 */
    float stuck_time;        /* 卡住条件需连续满足的时间，s。 */

    /* 自救腿重力前馈：抵消腿自重，关节PID不必独自扛静态负载。 */
    float leg_cm_ratio;      /* 腿质心沿虚拟腿的位置比例，0~1。均质杆取0.5。 */
    float gravity_ff_scale;  /* 前馈整定系数，0关闭。实机从0往1.0加。 */

    /* 阶段推进判据。 */
    float theta_min;         /* 转腿完成的腿摆绝对角窗口下限，rad。 */
    float theta_max;         /* 窗口上限，rad；两者中点同时是theta取等价角的参考。 */
    float ready_pitch;       /* 判定阶段完成的pitch上限，rad。 */
    float ready_roll;        /* 判定阶段完成的roll上限，rad。侧躺时腿长腿角也能到位，只查pitch会误判。 */
    float L0_tol;            /* 板凳腿长到位误差，m。 */
    float L0_diff_tol;       /* 左右腿长差上限，m。两腿不等长起身会歪。 */
    float angle_tol;         /* 板凳腿角到位误差，rad。 */
    float stable_time;       /* 到位条件需连续保持的时间，s。 */
    float fallen_timeout;    /* FALLEN超时，s。超时置RECOVERY_TIMEOUT并零输出。 */
    float prepare_timeout;   /* FALLING_TO_STAND超时，s。同上。 */

    /* 姿态门：能不能跳过转腿直接站，以及站立中什么时候判倒。 */
    float fall_pitch_filter; /* fall_pitch一阶低通系数，0~1。原始加速度含运动分量，需去噪。 */
    float direct_pitch;      /* 允许直接进站立/跳过转腿的倾角上限，rad。判据用fall_pitch。 */
    float phi0_min;          /* 上面这道门的腿杆角区间下限，rad。 */
    float phi0_max;          /* 区间上限，rad。 */
    /*
     * 站立中判定倒地的pitch阈值，rad。判据吃的是EKF pitch，
     * 而EKF pitch = asinf(...)，上界就是 pi/2 = 1.5708，
     * 所以这一项必须小于 1.5708，否则整条判据是永不成立的死分支。
     */
    float pitch_limit;
    float stand_phi0_min;    /* 站立中腿杆角保护下限，rad。 */
    float stand_phi0_max;    /* 站立中腿杆角保护上限，rad。 */

    /* 板凳模式。当前遥控拨杆没有映射到BENCH，这四项实机走不到。 */
    float bench_L0_rate;     /* 摇杆满杆的腿长调节速率，m/s。 */
    float bench_phi0_rate;   /* 摇杆满杆的腿角调节速率，rad/s。 */
    float bench_L0_min;      /* 可调腿长下限，m。 */
    float bench_L0_max;      /* 可调腿长上限，m。 */

    /* 自救关节串级：角度环出目标速度，速度环出关节力矩。 */
    algorithm_pid_config_t joint_angle_pid;
    algorithm_pid_config_t joint_speed_pid;
} Chassis_Recovery_Config_t;

/** @brief 小陀螺运动边界和十维反馈缩放。 */
typedef struct
{
    float max_d_s;                    /* 满杆平移速度，m/s。d_s和d_y共用。 */
    /*
     * 固定自转角速度，rad/s，正为俯视逆时针。
     * 进入小陀螺即按它起转，与摇杆无关；方向不对直接把这里改成负数。
     */
    float spin_d_fai;
    float d_fai_rate;                 /* 自转角速度目标斜率，rad/s^2。 */
    float scale[CHASSIS_STATE_COUNT]; /* 十维误差逐状态缩放，0关闭该状态反馈。 */
} Chassis_Top_Config_t;

/** @brief 正向辅助爬台阶动作参数。 */
typedef struct
{
    float approach_L0;   /* PREPARE目标腿长：碰台阶前先伸到这个高度再前进，m。 */
    float retract_L0;    /* CLIMB目标腿长：碰到台阶后收腿到这个高度再摆腿，m。 */
    float approach_d_s;  /* APPROACH前进速度上限，摇杆前进速度会被夹到此值以内，m/s。 */
    /*
     * 台阶专用的腿长目标斜率，m/s。台阶要快，自起要柔，所以不再共用
     * recovery.L0_rate（那一项仍归自起和正常站立变腿长使用）。
     * ⚠ 本项同时管两段：PREPARE 从当前腿长伸到 approach_L0，和 CLIMB
     * 收到 retract_L0。调大能加快收腿，但进入台阶模式那一下伸腿也会
     * 同步变快，切入瞬间对平衡的扰动更大——先架空确认切入不发飘再落地。
     * 参考：HERO_LEG 的腿长目标完全没有斜坡(硬阶跃)，但它有 450N 气弹簧
     * 托底、腿长 PID 也到 kp=2000/限幅180N，数值不能直接照搬。
     */
    float L0_rate;
    /*
     * 碰撞判据：request/feedback/theta三者同时满足才算碰到台阶。
     * contact_T_req是控制器算出来的期望轮力矩，contact_T_fb是DJI实测
     * 电流换算的力矩，两者都超限才说明轮子是真的被卡住而不是控制器
     * 瞬时超调；contact_theta再确认腿摆角度也到位。
     */
    float contact_T_req;  /* 期望轮力矩绝对值下限，N*m。 */
    float contact_T_fb;   /* 实测轮力矩绝对值下限，N*m。 */
    float contact_theta;  /* 腿摆角theta绝对值下限，rad。 */
    float contact_time;   /* 上面三个条件需连续满足的时间，s，碰撞锁存消抖用。 */
    /*
     * 并联的第二路碰撞判据，取自ZJU台阶检测：主判据P（机体俯仰 且 任一腿摆角
     * 超限）与辅判据S（俯仰角速度 或 速度跟踪误差 或 更大的俯仰 或 更大的腿摆角）
     * 相与。ZJU明确指出轮力矩这类信号与颠簸、急减速、踩弹丸难以区分，而
     * "指令速度不为零却跟不上"只有真被挡住才会出现，是其中最关键的一条。
     * 本路是整车级结论，成立时两条腿一起置候选。P里的腿摆角门复用contact_theta。
     */
    float contact_pitch;       /* P：机体俯仰绝对值下限，rad。 */
    float contact_pitch_hard;  /* S：更严的机体俯仰门，rad。 */
    float contact_theta_hard;  /* S：更严的腿摆角门，rad。 */
    float contact_d_pitch;     /* S：机体俯仰角速度门，rad/s。 */
    float contact_d_s_err;     /* S：指令与实测前进速度之差的下限，m/s。 */
    float contact_d_s_min;     /* S：指令速度本身的下限，m/s，排除停车工况。 */
    float recover_theta;  /* RECOVER阶段十维腿摆角目标，rad。 */
    float L0_tol;         /* PREPARE和RECOVER判断腿长到位的容差，m。 */
    float angle_tol;      /* RECOVER阶段判断腿摆角到位的容差，rad。 */
    /*
     * MOTION1/MOTION2 两段动作，取自ZJU的Onestep。两段都是位置型：目标角和
     * 目标腿长各自按限速走，由 Joint_Control() 的关节串级跟踪，不给固定力矩。
     * phi0 为腿杆相对车体角，theta 为含pitch的腿摆绝对角，单位均为 rad。
     */
    /*
     * MOTION1的后摆目标腿杆角。ZJU原值1.45，本车用不了：step.phi0_max=2.90
     * 换算到相对车体角上限只有 2.90-pi/2=1.33，摆到1.45会越过台阶姿态保护
     * 直接打到零力矩。取1.00留余量。
     */
    float swing_phi0;
    float swing_phi0_rate;  /* MOTION1腿杆角限速，rad/s。ZJU用300度/s。 */
    float home_phi0_rate;   /* MOTION2腿杆角限速，rad/s。ZJU用270度/s。 */
    /*
     * MOTION1/MOTION2的腿长限速，m/s。刻意不复用L0_rate：那一项同时管
     * PREPARE伸腿，调快会让切入台阶模式那一下也变快、对平衡的扰动变大。
     * 拆开之后PREPARE仍然柔、两段动作可以按ZJU取到1.8。
     */
    float climb_L0_rate;
    float mid_L0;           /* MOTION1的腿长目标，m。ZJU取行程中位。 */
    float climb_L0_tol;     /* MOTION1/MOTION2判腿长到位的容差，m。 */
    /*
     * 两段的到位容差各按自己那段的目标口径命名，别混：
     * MOTION1目标是车体系机构角，容差量的是机构角误差(不含pitch)；
     * MOTION2目标是大地竖直，容差量的是含pitch的theta。
     */
    float swing_phi0_tol;   /* MOTION1判机构角到位的容差，rad。 */
    float home_theta_tol;   /* MOTION2判腿摆角到位的容差，rad。 */
    /*
     * 台阶模式姿态保护，比站立放宽，对应HERO_LEG磕台阶抬高倒地阈值。
     * pitch_limit 同样必须小于 pi/2 = 1.5708，理由见 recovery.pitch_limit。
     */
    float pitch_limit;
    float phi0_min;
    float phi0_max;
} Chassis_Step_Config_t;

/**
 * @brief 整车实测机械与质量参数，控制前馈和观测统一从这里取。
 *
 * 每项注释同时记录MATLAB模型 ABK_LQR.m 的对应取值。两边不一致时，
 * 固件里的K不是本车的最优增益，必须重新生成后才谈得上最终整定。
 */
typedef struct
{
    float gravity;     /* 重力加速度，m/s^2。对应 g_ac。 */
    float body_mass;   /* 机体质量，kg。对应 m_b_ac。 */
    float leg_mass;    /* 单腿质量，kg。对应 m_l_ac。 */
    float wheel_mass;  /* 单轮质量，kg。对应 m_w_ac。 */
    float cg_to_hip;   /* 机体质心到腿部关节中心距离，m。对应 l_c_ac。 */
} Chassis_Model_Config_t;

/**
 * @brief 整车总质量，与MATLAB模型的质量分解保持一致。
 *
 * 重力前馈和全部力类观测阈值都以它为基准，只在这里算一次。
 */
static inline float Chassis_Model_Mass(const Chassis_Model_Config_t *config)
{
    // return config->body_mass +
    //        2.0f * config->leg_mass +
    //        2.0f * config->wheel_mass;
    return config->body_mass;
}

/**
 * @brief 打滑、离地、转向和卡腿观测参数。
 *
 * 打滑、转向、卡腿三套仍然只读，只进Watch。离地这一套在
 * output.off_ground_act_flag 打开后会驱动控制，见 Chassis_Control()。
 */
typedef struct
{
    float residual_filter_s;
    float turn_filter_s;
    float normal_force_filter_s;
    /* 打滑：先过闸门再判进入，退出由起始轮速锁存决定，不再用退出计时。 */
    float slip_gate_yaw;      /* 闸门偏航残差阈值，rad/s。 */
    float slip_gate_v;        /* 闸门轮速与整车速度差阈值，m/s。 */
    float slip_v_enter;       /* 速度残差进入阈值，m/s。 */
    float slip_yaw_enter;     /* 偏航残差进入阈值，rad/s。 */
    float slip_dv_enter;      /* 轮速增量与加速度增量差进入阈值，m/s。 */
    float slip_enter_s;       /* 进入条件连续满足时间，s。 */
    /*
     * 离地。支撑力口径见 Observer_Static_Load()：单腿静载含腿和轮自重，
     * 站立时 Fn_ratio 约为 1.0，下面两个比例就是字面意义上的百分比。
     */
    float off_force_ratio;    /* 判离地的支撑力/标称静载比。 */
    float land_force_ratio;   /* 判落地的支撑力/标称静载比。 */
    float off_hold_s;         /* 离地条件需连续满足的时间，s。 */
    /*
     * 小陀螺下的离地消抖，s。取自ZJU：旋转带来的离心与侧向载荷转移会让
     * 支撑力估计周期性掉到门限以下，一般模式20ms、小陀螺放宽到100ms。
     * 本车 top.spin_d_fai 比ZJU快，这个风险只会更大。
     */
    float off_hold_spin_s;
    float land_hold_s;        /* 触地条件需连续满足的时间，s。 */
    /*
     * 触地判据的第二路，取自ZJU。ZJU明确不用支持力恢复判触地：空中主动
     * 伸腿过程中触地时，腿仍在伸长、速度不会立刻反向，只是被地面压制，
     * 只看支撑力会漏检。两条并联，满足其一即可：
     *   A 腿长速度反向：d_L0 < land_d_L0_reverse
     *   B 伸腿峰值回落：L0 < lqr.L0_max - land_L0_margin
     *                 且 d_L0_peak > land_d_L0_peak_min
     *                 且 d_L0_peak - d_L0 > land_d_L0_drop
     */
    float land_d_L0_reverse;  /* 判据A的腿长速度上限，m/s，取负值。 */
    float land_L0_margin;     /* 判据B要求腿长离拟合上限至少这么多，m。 */
    float land_d_L0_peak_min; /* 判据B要求本次腾空的伸腿速度峰值下限，m/s。 */
    float land_d_L0_drop;     /* 判据B要求速度从峰值回落的幅度，m/s。 */
    float off_F_comp_ratio;   /* 整车离地后悬空腿的下压推力，相对单腿静载的比例。 */
    /*
     * 下压推力的腿长保护余量，m。腿长进到 lqr.L0_max - 本项 以内就不再加
     * 推力，否则悬空腿会一直顶到机构限位。取自ZJU「悬空腿接近最大腿长后
     * 不再施加额外推力」。
     */
    float off_comp_L0_margin;
    /* 转向。 */
    float turn_v_diff;        /* 触发转弯半径计算的左右轮速差阈值，m/s。 */
    /* 基准是 Observer_Static_Load()（含腿轮自重），不是 0.5*body_mass*g。 */
    float turn_force_limit_ratio;
    /* 卡腿。 */
    float stuck_T_ratio;      /* 判卡腿的轮力矩阈值，相对wheel.T_limit的比例。 */
    float stuck_theta_enter;  /* 判卡腿的腿摆角阈值，rad。 */
    float stuck_theta_exit;   /* 退出卡腿计时的腿摆角阈值，rad。 */
    float stuck_time;         /* 卡腿条件连续满足时间，s。 */
    /* 下面两项的单腿静载同样是 Observer_Static_Load()，含腿轮自重。 */
    float stuck_F0_coef_ratio;/* 补偿轴向力增长速率，相对单腿静载的比例每秒。 */
    float stuck_F0_max_ratio; /* 补偿轴向力上限，相对单腿静载的比例。 */
    float stuck_L0_coef;      /* 建议收腿量随卡腿时间增长的速率，m/s。 */
    float stuck_L0_max;       /* 建议收腿量上限，m。 */
} Chassis_Observer_Config_t;

/** @brief 双腿长poly22增益拟合的范围和系数表。 */
typedef struct
{
    float L0_min; /* 当前系数实际采样腿长下限，m。 */
    float L0_max; /* 当前系数实际采样腿长上限，m。 */
    /*
     * 十维状态各自的误差限幅，进K点乘之前生效，0表示该项不限幅。
     * 只限位置类状态，速度类保持不限，与参考工程一致。
     */
    float error_limit[CHASSIS_STATE_COUNT];
    /*
     * 展平成一维是为了能直接粘贴MATLAB输出，内存布局与[4][10][6]完全一致。
     * 下标 = (输出序号 * CHASSIS_STATE_COUNT + 状态序号) * 系数个数 + 系数序号。
     */
    float coefficients[CHASSIS_OUTPUT_COUNT * CHASSIS_STATE_COUNT *
                       ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} Chassis_LQR_Config_t;

/**
 * @brief LESO扩张状态观测器的离散模型、观测器增益和接入门。
 *
 * 三块系数与lqr.coefficients出自ABK_LQR.py的同一次运行、同一个腿长网格，
 * 因此腿长采样区间复用lqr.L0_min/L0_max，不在这里再存一份。
 * 扩张系统的A_e = [Ad Bd; 0 I]、B_e = [Bd; 0]由Ad和Bd现场展开，不重复存储。
 */
typedef struct
{
    /* 扰动补偿接入比例，0关闭补偿只观测，1按估计值足额减去。 */
    float comp_scale;
    /* 各输入通道扰动估计限幅，N*m，0表示该通道不限幅。 */
    float d_limit[CHASSIS_OUTPUT_COUNT];
    /* 离散状态矩阵Ad，行序 i * CHASSIS_STATE_COUNT + j。 */
    float Ad_coefficients[CHASSIS_STATE_COUNT * CHASSIS_STATE_COUNT *
                          ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
    /* 离散输入矩阵Bd，行序 i * CHASSIS_OUTPUT_COUNT + j。 */
    float Bd_coefficients[CHASSIS_STATE_COUNT * CHASSIS_OUTPUT_COUNT *
                          ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
    /* 观测器增益L，(状态+扰动)行乘状态列，行序 i * CHASSIS_STATE_COUNT + j。 */
    float L_coefficients[(CHASSIS_STATE_COUNT + CHASSIS_OUTPUT_COUNT) *
                         CHASSIS_STATE_COUNT *
                         ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} Chassis_LESO_Config_t;

/**
 * @brief 车身高度与Roll的MPC参数（TinyMPC）。
 *
 * 复刻 Qi-Q26 的轻量化MPC：状态 x=[alpha, d_alpha, H, d_H]，输入 u=[F_l, F_r]。
 * A/B 不写成矩阵常量，而是由 model/wheel 里的物理量在 Chassis_MPC_Init() 现场推导，
 * 这样换机器人时改 model 和 wheel 就自动跟随，和本文件其余按比例定义的量一个原则。
 *
 *   竖直：M*ddH   = (F_l + F_r)*cos(alpha) - damping*dH - M*g
 *   横滚：I*ddroll = (F_l - F_r)*half_track * roll_sign
 *
 * u 保持【绝对支撑力】，重力作为仿射项进 TinyMPC 的 fdyn，所以下面的力限幅
 * 就是字面意义的绝对值，不需要相对平衡点平移。
 */
typedef struct
{
    float I_roll;        /* 横滚转动惯量，kg*m^2。当前是 M*half_track^2 的估算值。 */
    float damping;       /* 竖直阻尼，N*s/m。只影响 Ad(3,3)，取值不敏感。 */
    /*
     * 横滚通道方向，取+1或-1。整车右手系里+roll是左侧上抬，左腿加力会把左侧
     * 顶得更高，所以是 ddroll = +(F_l - F_r)*R/I，与 Qi-Q26 原文相反。
     * PC端已验证取+1时纠偏方向正确；实机若发现roll越纠越偏就改成-1。
     */
    float roll_sign;
    float Q[CHASSIS_STATE_MPC_COUNT]; /* 状态权重，顺序 alpha, d_alpha, H, d_H。 */
    float R[CHASSIS_MPC_INPUT_COUNT]; /* 输入权重，罚的是相对 M*g/2 的偏差。 */
    float rho;           /* ADMM惩罚参数。 */
    uint16_t horizon;    /* 预测步数N。15步*10ms=0.15s，够覆盖跳跃和下台阶。 */
    uint16_t max_iter;   /* ADMM迭代硬上限。宁可次优也要让耗时确定。 */
    uint16_t decimation; /* 底盘任务多少拍求解一次。1kHz任务、10拍=100Hz。 */
    float F_min;         /* 单腿支撑力下限，N。 */
    float F_max;         /* 单腿支撑力上限，N。 */
    /*
     * 单步力变化率上限，N/拍。TinyMPC的约束都是逐时刻的，跨时刻耦合装不进去；
     * 但滚动优化只下发第0步，而上一拍的u是已知常量，所以第0步的变化率就是
     * 纯逐时刻约束——真正生效的控制量严格满足，而且是在优化里满足的。
     */
    float dF_max;
} Chassis_MPC_Config_t;

/** @brief 底盘控制唯一只读配置，集中保存机械、模型和安全参数。 */
typedef struct
{
    Chassis_Leg_Config_t leg[CHASSIS_LEG_COUNT];
    Chassis_Model_Config_t model;
    Chassis_IMU_Config_t imu;
    Chassis_Wheel_Config_t wheel;
    Chassis_Kalman_Config_t speed_kalman;
    algorithm_pid_config_t leg_length_pid;
    algorithm_pid_config_t roll_pid;
    Chassis_Recovery_Config_t recovery;
    Chassis_Top_Config_t top;
    Chassis_Step_Config_t step;
    Chassis_Observer_Config_t observer;
    Chassis_LQR_Config_t lqr;
    Chassis_LESO_Config_t leso;
    Chassis_MPC_Config_t mpc;
    Chassis_Output_Config_t output;
    float phi0_offset;                /* phi0换算theta时的竖直零点，rad。 */
    float roll_target;                /* 机体横滚目标，rad。 */
    float F0_gravity_scale;           /* 重力前馈整定系数，1.0为按实测质量足额补偿。 */
    float F0_left;                    /* 左腿支撑力静态修正，N。 */
    float F0_right;                   /* 右腿支撑力静态修正，N。 */
    float default_dt;               /* 控制周期异常时采用的默认dt，s。 */
    float dt_min;                   /* 最小控制周期，s。 */
    float dt_max;                   /* 最大控制周期，s。 */
    float target[CHASSIS_STATE_COUNT]; /* 十维LQR默认目标状态。 */
    /*
     * 整车离地时的十维误差逐状态缩放，0关闭该状态反馈。取自ZJU式142，
     * 只保留腿摆角和腿摆角速度：空中没有支撑，位移、速度、航向和俯仰
     * 通道无从执行。优先级高于 top.scale——空转时既在小陀螺又腾空的话，
     * 牵引力为零，位置和航向控制毫无意义。
     */
    float off_ground_scale[CHASSIS_STATE_COUNT];
} Chassis_Config_t;

extern const Chassis_Config_t Chassis_Config;

#ifdef __cplusplus
}
#endif

#endif
