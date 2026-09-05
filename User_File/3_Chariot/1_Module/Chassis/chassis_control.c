#include "chassis_control.h"

#include "Angle.h"
#include "LQR.h"
#include "Limit.h"
#include "PID.h"
#include "chassis_config.h"
#include "chassis_mpc.h"

#include <math.h>
#include <string.h>

#define CHASSIS_RPM_TO_RADPS 0.10471975512f
#define CHASSIS_OUTPUT_FAULT_MASK                                         \
    (CHASSIS_FAULT_DISABLED | CHASSIS_FAULT_IMU |                       \
     CHASSIS_FAULT_DM_MOTOR | CHASSIS_FAULT_DM_ERROR |                  \
     CHASSIS_FAULT_DJI_MOTOR | CHASSIS_FAULT_CAN |                      \
     CHASSIS_FAULT_KINEMATICS | CHASSIS_FAULT_REMOTE)

Chassis_t Chassis;

/*
 * K矩阵每周期由 leg[].L0 重算后当周期用完，不跨周期携带信息，因此放在文件
 * 作用域而不挂进 Chassis，避免 Watch 树里多出 40 个派生量。调试器仍可观察。
 */
static float lqrK[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT];

static void Control_Reset(void);

/** @brief 让目标量每周期最多移动maximum_step，避免腿长目标阶跃。 */
static float Move_Toward(float value, float target, float maximum_step)
{
    float positive_step = fabsf(maximum_step);

    if (value < target - positive_step)
    {
        return value + positive_step;
    }
    if (value > target + positive_step)
    {
        return value - positive_step;
    }
    return target;
}

/**
 * @brief 将遥控运动目标写入正常站立的十维目标，不做斜坡。
 *
 * 位移和航向都按"有输入走速度、松杆锁位置"处理：
 * 有前进输入时位移状态在本文件后段被清零，位移项不参与，只跟速度；
 * 松杆后位移从零开始积分，把车锁在松杆位置。
 * 有偏航输入时航向目标按给定角速度积分；松杆瞬间锁存当前航向并保持，
 * 偏航角速度目标同时清零。
 */
static void Motion_Update(void)
{
    float dt = Chassis.dt;
    float model_yaw_rad = Chassis.imu.yaw_total *
                          Chassis_Config.imu.yaw_angle_scale;
    float top_phase_rad;

    if (Chassis.mode == CHASSIS_MODE_TOP)
    {
        top_phase_rad = model_yaw_rad - Chassis.top_fai;
        Chassis.top_d_s =
            Chassis.goal.d_s * cosf(top_phase_rad) +
            Chassis.goal.d_y * sinf(top_phase_rad);
        Chassis.lqr.target[CHASSIS_STATE_D_S] = Chassis.top_d_s;
        /* 小陀螺持续旋转，航向目标始终跟随实际，只靠角速度控制。 */
        Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
        /* 起转和停转都按斜率走，避免角速度目标阶跃冲击机体pitch。 */
        Chassis.lqr.target[CHASSIS_STATE_D_FAI] =
            Move_Toward(Chassis.lqr.target[CHASSIS_STATE_D_FAI],
                        Chassis.goal.d_fai,
                        Chassis_Config.top.d_fai_rate * dt);
        Chassis.yaw_stick_flag = 0U;
        Chassis.top_exit_flag = 1U;
    }
    else if (Chassis.top_exit_flag != 0U)
    {
        /*
         * 刚退出小陀螺，角速度目标还停在自转转速上。直接交回摇杆会让目标
         * 从spin_d_fai阶跃到摇杆值，LQR随即给出大反向轮力矩硬刹机体。
         * 按起转同一斜率收敛到摇杆值后再交回，收敛期间航向目标跟随实际，
         * 避免和松杆锁航向逻辑同时生效。
         */
        Chassis.top_d_s = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_D_S] = Chassis.goal.d_s;
        Chassis.lqr.target[CHASSIS_STATE_D_FAI] =
            Move_Toward(Chassis.lqr.target[CHASSIS_STATE_D_FAI],
                        Chassis.goal.d_fai,
                        Chassis_Config.top.d_fai_rate * dt);
        Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
        Chassis.yaw_stick_flag = 0U;
        /* Move_Toward到达时精确取到目标值，因此等值即表示斜坡走完。 */
        if (Chassis.lqr.target[CHASSIS_STATE_D_FAI] == Chassis.goal.d_fai)
        {
            Chassis.top_exit_flag = 0U;
        }
    }
    else
    {
        Chassis.top_d_s = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_D_S] = Chassis.goal.d_s;

        if (Chassis.goal.d_fai != 0.0f)
        {
            /* 有偏航输入：按给定角速度积分出航向目标。 */
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = Chassis.goal.d_fai;
            Chassis.lqr.target[CHASSIS_STATE_FAI] +=
                Chassis.goal.d_fai * dt;
            Chassis.yaw_stick_flag = 1U;
        }
        else
        {
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = 0.0f;
            if (Chassis.yaw_stick_flag != 0U)
            {
                /* 松杆瞬间锁存当前航向，清掉转向期间的跟踪误差后保持。 */
                Chassis.lqr.target[CHASSIS_STATE_FAI] = model_yaw_rad;
                Chassis.yaw_stick_flag = 0U;
            }
        }
    }

    /*
     * 腿长目标按速率逼近goal.L0，不直接赋值。
     * State_Enter进STANDING时已把target_L0锁到当前实际腿长，直接赋值会在
     * 下一拍抹掉这次接管：自救结束腿停在bench_L0，随即被拽到goal.L0，
     * 而收腿方向与重力同向、腿长PID只有kd这一项阻尼，于是出现伸腿后
     * 再收腿的那一下冲击。斜坡让这段落差按L0_rate走完。
     */
    Chassis.leg[CHASSIS_LEFT].target_L0 =
        Move_Toward(Chassis.leg[CHASSIS_LEFT].target_L0,
                    Chassis.goal.L0,
                    Chassis_Config.recovery.L0_rate * dt);
    Chassis.leg[CHASSIS_RIGHT].target_L0 =
        Move_Toward(Chassis.leg[CHASSIS_RIGHT].target_L0,
                    Chassis.goal.L0,
                    Chassis_Config.recovery.L0_rate * dt);
}

/** @brief 判断周期角是否存在落在指定连续区间内的等价角。 */
static uint8_t Angle_In_Range(float angle_rad,
                                        float minimum_rad,
                                        float maximum_rad)
{
    float center_rad = (minimum_rad + maximum_rad) * 0.5f;
    float equivalent_rad =
        Algorithm_AngleNearestEquivalentRad(angle_rad, center_rad);

    return ((equivalent_rad >= minimum_rad) &&
            (equivalent_rad <= maximum_rad)) ? 1U : 0U;
}

/**
 * @brief 机体是否大致朝上。判据是竖向加速度，不是姿态角。
 *
 * 取自ZJU式121。他们的原话：倒地后IMU的姿态解算可能已经跨过奇异区、
 * 四元数收敛到错误分支，而重力方向的加速度投影始终可靠。
 * 本机实测印证了这一点：机体倾过90度以后 fall_pitch 会折返，底朝天时的
 * 读数和直立分不开——原先"用 fall_pitch 替掉 imu.pitch 以躲开折返"的做法
 * 只是把折返从一个量搬到了另一个量，并没有解决问题。
 * 凡是"能不能直接站/要不要先翻身"这类判断都必须过这道门，姿态角只允许
 * 在这道门放行之后再用（ZJU也是这个顺序：Swing和DrawBack的退出才看pitch）。
 */
static uint8_t Body_Upward(void)
{
    return ((Chassis.imu.accel_raw[2] *
             Chassis_Config.imu.fall_accel_z_scale) >
            (Chassis_Config.recovery.turnover_az_ratio *
             Chassis_Config.model.gravity)) ? 1U : 0U;
}

/**
 * @brief 汇总只影响最终电机输出许可的故障位。
 *
 * 这些故障不会阻断VMC、PID或LQR中间量计算；真正的几何和控制计算
 * 故障由对应控制流程另外追加到Chassis.fault。
 */
static uint32_t Output_Fault_Get(void)
{
    uint32_t fault = CHASSIS_FAULT_NONE;
    uint32_t index;

    if (Chassis.enable_flag == 0U)
    {
        fault |= CHASSIS_FAULT_DISABLED;
    }
    if ((Chassis.imu.init_flag == 0U) ||
        (Chassis.imu.attitude_flag == 0U) ||
        (Chassis.imu.error_code != 0U))
    {
        fault |= CHASSIS_FAULT_IMU;
    }
    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        if (Chassis.dm_motor[index].online_flag == 0U)
        {
            fault |= CHASSIS_FAULT_DM_MOTOR;
        }
        /*
         * DM反馈状态0为失能、1为使能，8~E为超压、欠压、过流、过温、通讯丢失和过载。
         * 电机报错后会自行退出使能但仍在发反馈帧，只判online会漏掉这类静默失效。
         */
        if (Chassis.dm_motor[index].err_state >= 8U)
        {
            fault |= CHASSIS_FAULT_DM_ERROR;
        }
    }
    /* 轮通道关闭时不驱动轮电机，其在线状态不参与封锁，避免只调髋关节时
     * 轮电调未上电连带封锁关节力矩。 */
    if (Chassis_Config.output.wheel_flag != 0U)
    {
        for (index = 0U; index < APP_WHEEL_COUNT; index++)
        {
            if (Chassis.wheel_motor[index].online_flag == 0U)
            {
                fault |= CHASSIS_FAULT_DJI_MOTOR;
                break;
            }
        }
    }
    if (Chassis.can_error_count > APP_CAN_TX_ERROR_MAX)
    {
        fault |= CHASSIS_FAULT_CAN;
    }
    if ((Chassis.remote_online_flag == 0U) || (Chassis.remote_stop_flag != 0U))
    {
        fault |= CHASSIS_FAULT_REMOTE;
    }

    return fault;
}

/**
 * @brief 清空本轮全部控制请求量与最终电机命令，并置回发送前安全门。
 *
 * 每条控制链在入口调用一次，保证任何提前返回都不会遗留上一周期的命令。
 */
static void Output_Clear(void)
{
    memset(Chassis.output.T_joint, 0, sizeof(Chassis.output.T_joint));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
    memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    memset(Chassis.output.I_wheel_req,
           0,
           sizeof(Chassis.output.I_wheel_req));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    memset(Chassis.output.T_wheel, 0, sizeof(Chassis.output.T_wheel));
    Chassis.output.safe_flag = 1U;
}

/** @brief 清空四个关节串级PID状态、目标和调试请求量。 */
static void Joint_Control_Reset(void)
{
    uint32_t index;

    for (index = 0U; index < APP_DM_COUNT; index++)
    {
        Algorithm_PID_Init(&Chassis.joint_angle_pid[index]);
        Algorithm_PID_Init(&Chassis.joint_speed_pid[index]);
    }
    memset(Chassis.output.target_angle,
           0,
           sizeof(Chassis.output.target_angle));
    memset(Chassis.output.target_speed,
           0,
           sizeof(Chassis.output.target_speed));
    memset(Chassis.output.T_joint_req,
           0,
           sizeof(Chassis.output.T_joint_req));
}

/**
 * @brief 切换内部控制状态并初始化该状态所需目标和控制器。
 *
 * 相同状态不重复进入，避免每周期清空计时器、PID和目标斜坡。
 */
static void State_Enter(Chassis_State_t state)
{
    uint32_t side;

    if (Chassis.state == state)
    {
        return;
    }

    Chassis.state = state;
    Chassis.state_time = 0.0f;
    Chassis.stable_time = 0.0f;
    Output_Clear();
    Joint_Control_Reset();

    if ((state == CHASSIS_STANDING) || (state == CHASSIS_BENCH) ||
        (state == CHASSIS_STEP))
    {
        Control_Reset();
        memcpy(Chassis.lqr.target,
               Chassis_Config.target,
               sizeof(Chassis.lqr.target));
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_FAI] =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    }

    if (state == CHASSIS_STANDING)
    {
        Algorithm_PID_Init(&Chassis.leg_length_pid);
        Algorithm_PID_Init(&Chassis.roll_pid);
        Chassis_Leso_Init(&Chassis.leso);
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            /* 五连杆正解有效时从当前腿长接管，否则回到配置目标腿长。 */
            Chassis.leg[side].target_L0 =
                ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
                 (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ?
                    Chassis.leg[side].L0 :
                    Chassis_Config.leg[side].target_L0;
        }
    }
    else if ((state == CHASSIS_FALLEN) ||
             (state == CHASSIS_FALLING_TO_STAND))
    {
        /*
         * 自救的腿长和腿角目标都从当前实际姿态起步，之后逐周期按速率推进。
         * 不锁存的话目标会相对实际角常值超前，关节PID全程满输出，动作很猛。
         * 正解无效时实际姿态是0，只能回到配置目标，否则逆解直接失败。
         */
        uint8_t leg_valid =
            ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
             (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;

        /*
         * 倒地方向和扫掠方向只属于转腿阶段，进FALLEN时按本轮姿态重新锁存，
         * 0表示还没锁。收腿阶段不消费它们，也就不清。
         */
        if (state == CHASSIS_FALLEN)
        {
            /*
             * ZJU式121：按竖向加速度决定进哪个子阶段，机体已经朝上就跳过
             * 翻身直接摆腿。用加速度不用姿态角，理由和翻身退出判据一样：
             * 倒地后四元数可能收敛到错误分支，重力投影始终可靠。
             */
            Chassis.recovery_phase = (Body_Upward() != 0U) ?
                CHASSIS_RECOVERY_SWING : CHASSIS_RECOVERY_TURNOVER;
            Chassis.recovery_phase_hold = 0.0f;
            Chassis.recovery_theta_ref = 0.0f;
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.recovery_direction[side] = 0.0f;
                Chassis.recovery_stuck_time[side] = 0.0f;
            }
        }

        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_phi0 =
                (leg_valid != 0U) ? Chassis.leg[side].phi0_total
                                  : Chassis_Config.recovery.bench_phi0;
            Chassis.leg[side].target_L0 =
                (leg_valid != 0U) ? Chassis.leg[side].L0
                                  : Chassis_Config.leg[side].target_L0;
        }
    }
    else if (state == CHASSIS_STEP)
    {
        Chassis.step_phase = CHASSIS_STEP_PREPARE;
        Chassis.step_fai =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
        memset(Chassis.contact_time,
               0,
               sizeof(Chassis.contact_time));
        memset(Chassis.step_contact_flag,
               0,
               sizeof(Chassis.step_contact_flag));
        memset(Chassis.step_contact_latch_flag,
               0,
               sizeof(Chassis.step_contact_latch_flag));
        Chassis.step_posture_flag = 0U;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            /*
             * 和进STANDING一样从当前实际腿长接管，再由Chassis_Step()按
             * step.L0_rate斜坡逼近approach_L0。这里直接赋approach_L0
             * 会让那条斜坡变成空操作：切入瞬间腿长目标阶跃(中档0.14到
             * 0.30差0.16m)，kp=800把腿长PID一拍打到outputLimit=100N，
             * 叠加重力前馈后F0超过整车重量，腿被弹出去直接失衡倒地。
             */
            Chassis.leg[side].target_L0 =
                ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
                 (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ?
                    Chassis.leg[side].L0 :
                    Chassis_Config.step.approach_L0;
        }
    }
}

/** @brief 重新初始化速度Kalman、前进速度、加速度和位移状态。 */
static void Control_Reset(void)
{
    Algorithm_Kalman_Init(&Chassis.speed_kalman, 2U, 2U);
    memcpy(Chassis.speed_kalman.covariance,
           Chassis_Config.speed_kalman.initial_covariance,
           sizeof(Chassis_Config.speed_kalman.initial_covariance));
    memcpy(Chassis.speed_kalman.processNoise,
           Chassis_Config.speed_kalman.process_noise,
           sizeof(Chassis_Config.speed_kalman.process_noise));
    memcpy(Chassis.speed_kalman.measurementNoise,
           Chassis_Config.speed_kalman.measurement_noise,
           sizeof(Chassis_Config.speed_kalman.measurement_noise));

    Chassis.speed_kalman.stateTransition[0] = 1.0f;
    Chassis.speed_kalman.stateTransition[1] = APP_CTRL_DT_S;
    Chassis.speed_kalman.stateTransition[2] = 0.0f;
    Chassis.speed_kalman.stateTransition[3] = 1.0f;
    Chassis.speed_kalman.measurementMatrix[0] = 1.0f;
    Chassis.speed_kalman.measurementMatrix[1] = 0.0f;
    Chassis.speed_kalman.measurementMatrix[2] = 0.0f;
    Chassis.speed_kalman.measurementMatrix[3] = 1.0f;

    Chassis.body.s = 0.0f;
    Chassis.body.d_s_raw = 0.0f;
    Chassis.body.d_s = 0.0f;
    Chassis.body.dd_s = 0.0f;
    Chassis.body.dd_s_fused = 0.0f;
    Chassis_Slip_Init(&Chassis.slip);
    Chassis_Ground_Init(&Chassis.ground);
    Chassis_Turn_Init(&Chassis.turn);
    Chassis_Stuck_Init(&Chassis.stuck);
}

/**
 * @brief 清空实际发送量和请求量，用于主动零力或计算失败状态。
 */
void Chassis_Zero_Output(void)
{
    Output_Clear();
    Control_Reset();
}

/** @brief 初始化唯一底盘状态、目标、PID和速度融合器。 */
void Chassis_Init(void)
{
    uint32_t index;

    memset(&Chassis, 0, sizeof(Chassis));
    Chassis.mode = CHASSIS_MODE_ZERO_FORCE;
    Chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
    Chassis.state = CHASSIS_ZERO_FORCE;
    Chassis.output.safe_flag = 1U;
    Chassis.dt = APP_CTRL_DT_S;
    Chassis.goal.d_s = 0.0f;
    Chassis.goal.d_y = 0.0f;
    Chassis.goal.d_fai = 0.0f;
    Chassis.goal.L0 = APP_RC_LEG_M;
    memcpy(Chassis.lqr.target,
           Chassis_Config.target,
           sizeof(Chassis.lqr.target));

    Algorithm_PID_Init(&Chassis.leg_length_pid);
    Algorithm_PID_Init(&Chassis.roll_pid);
    Chassis_Leso_Init(&Chassis.leso);
    /* MPC求解器在这里一次性分配好工作矩阵，控制环里不再分配。 */
    Chassis_MPC_Init();
    Joint_Control_Reset();
    for (index = 0U; index < CHASSIS_LEG_COUNT; index++)
    {
        Chassis.leg[index].target_L0 =
            Chassis_Config.leg[index].target_L0;
        Chassis.leg[index].target_phi0 =
            Chassis_Config.recovery.bench_phi0;
    }
    Control_Reset();
}

/**
 * @brief 使用任务层保存的四个DM反馈更新左右腿五连杆状态。
 *
 * 本函数不检查电机online标志，离线调试时仍使用最后一次反馈计算。
 * 左右腿分别提交本轮已经定义的中间量，valid只描述数学完整性。
 */
void Chassis_Leg_Update(void)
{
    uint32_t side;

    /* 每条腿独立提交，便于Watch区分是哪一侧的数学计算不完整。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float last_phi0 = Chassis.leg[side].phi0_total;
        uint8_t phi1_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        /* 配置索引直接用作 dm_motor 下标，越界时不得继续访问数组。 */
        if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT))
        {
            Chassis.leg[CHASSIS_LEFT].valid_flag = 0U;
            Chassis.leg[CHASSIS_RIGHT].valid_flag = 0U;
            return;
        }

        VMC_State_Calc(&Chassis_Config.leg[side],
                       Chassis.dm_motor[phi1_index].position_rad,
                       Chassis.dm_motor[phi4_index].position_rad,
                       Chassis.dm_motor[phi1_index].speed_radps,
                       Chassis.dm_motor[phi4_index].speed_radps,
                       &Chassis.leg[side]);
        if (Chassis.leg[side].L0 > 0.0f)
        {
            /* phi0主值跨过+-pi时选择距离上一有效角最近的等价角。 */
            Chassis.leg[side].phi0_total =
                Algorithm_AngleNearestEquivalentRad(
                    Chassis.leg[side].phi0,
                    last_phi0);
        }
        else
        {
            /* 当前phi0无定义时保留上次连续角，避免恢复后丢失圈数。 */
            Chassis.leg[side].phi0_total = last_phi0;
        }
    }
}

/**
 * @brief 把外部mode转换成内部state，并维护输出故障和站立保护。
 *
 * 该函数只选择本周期应执行的控制流程，不计算VMC、PID或LQR。设备
 * 和数学故障只封锁最终输出；姿态越界仍会切入零力状态。
 */
void Chassis_State_Update(void)
{
    float theta_b;
    uint8_t leg_valid_flag;
    uint8_t attitude_ready;
    uint32_t output_fault;
    uint32_t active_fault;
    uint32_t last_output_fault;

    /*
     * 0. 重力矢量俯仰角。必须放在任何 return 之前每周期更新：模式边沿那一拍
     * 就要用它判姿态，只在 FALLEN 里更新会读到陈旧值，低通也得一直是热的。
     * 角度差先归一化再滤波，机体接近 ±pi 时不会穿过断点被滤向反方向。
     */
    Chassis.fall_pitch = Algorithm_AngleNormalizeRad(
        Chassis.fall_pitch +
        (Chassis_Config.recovery.fall_pitch_filter *
         Algorithm_AngleNormalizeRad(
             atan2f(Chassis_Config.imu.fall_accel_x_scale *
                        Chassis.imu.accel_raw[0],
                    Chassis_Config.imu.fall_accel_z_scale *
                        Chassis.imu.accel_raw[2]) -
             Chassis.fall_pitch)));

    /* 1. 外部主动零力具有最高优先级，不进入任何闭环控制。 */
    if (Chassis.mode == CHASSIS_MODE_ZERO_FORCE)
    {
        State_Enter(CHASSIS_ZERO_FORCE);
        Chassis.last_mode = CHASSIS_MODE_ZERO_FORCE;
        Chassis.fault = CHASSIS_FAULT_NONE;
        return;
    }

    /* 2. 更新输出故障，并在故障全部清除的边沿重置动态控制状态。 */
    theta_b = Chassis.imu.pitch *
                     Chassis_Config.imu.pitch_angle_scale;
    leg_valid_flag =
        ((Chassis.leg[CHASSIS_LEFT].valid_flag != 0U) &&
         (Chassis.leg[CHASSIS_RIGHT].valid_flag != 0U)) ? 1U : 0U;
    output_fault = Output_Fault_Get();
    active_fault = output_fault;
    if (leg_valid_flag == 0U)
    {
        active_fault |= CHASSIS_FAULT_KINEMATICS;
    }
    last_output_fault =
        Chassis.fault & CHASSIS_OUTPUT_FAULT_MASK;
    if ((last_output_fault != CHASSIS_FAULT_NONE) &&
        (active_fault == CHASSIS_FAULT_NONE))
    {
        /*
         * 输出重新放行前丢弃故障期间由陈旧反馈积累的动态状态，首个
         * 完整控制周期从当前姿态重新建立速度、积分和 yaw 平衡点。
         */
        Algorithm_PID_Init(&Chassis.leg_length_pid);
        Algorithm_PID_Init(&Chassis.roll_pid);
        Chassis_Leso_Init(&Chassis.leso);
        Joint_Control_Reset();
        Control_Reset();
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.lqr.target[CHASSIS_STATE_FAI] =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    }

    /* 3. 仅在外部模式变化时选择入口状态，避免每周期重复初始化。 */
    if (Chassis.mode != Chassis.last_mode)
    {
        /*
         * 能否直接进入站立：左右腿正解有效、机体倾角较小且两条虚拟腿都
         * 位于准备角区间；正解无效时也放行，中间量继续计算，最终输出
         * 由故障门封锁。TOP和FOLLOW共用同一个姿态门。
         * ⚠ 这里必须先过 Body_Upward()。fall_pitch 同样会折返——实测底朝天
         * 时它的读数和直立分不开——只查它的话底朝天会被判成"可以站"，
         * 直接跳过自救去跑LQR。姿态角只在 az 确认机体朝上之后才可信。
         */
        attitude_ready =
            ((leg_valid_flag == 0U) ||
             ((Body_Upward() != 0U) &&
              (fabsf(Chassis.fall_pitch) <=
               Chassis_Config.recovery.direct_pitch) &&
              (Angle_In_Range(
                   Chassis.leg[CHASSIS_LEFT].phi0_total,
                   Chassis_Config.recovery.phi0_min,
                   Chassis_Config.recovery.phi0_max) != 0U) &&
              (Angle_In_Range(
                   Chassis.leg[CHASSIS_RIGHT].phi0_total,
                   Chassis_Config.recovery.phi0_min,
                   Chassis_Config.recovery.phi0_max) != 0U))) ? 1U : 0U;

        Chassis.fault = active_fault;
        switch (Chassis.mode)
        {
        case CHASSIS_MODE_TOP:
            Chassis.top_fai =
                Chassis.imu.yaw_total *
                Chassis_Config.imu.yaw_angle_scale;
            Chassis.lqr.target[CHASSIS_STATE_FAI] =
                Chassis.top_fai;
            if (Chassis.state != CHASSIS_STANDING)
            {
                if (attitude_ready != 0U)
                {
                    State_Enter(CHASSIS_STANDING);
                }
                else
                {
                    /* 小陀螺不自动自起，姿态不满足先拨回左中站起来。 */
                    Chassis.fault =
                        output_fault | CHASSIS_FAULT_CONTROL;
                    State_Enter(CHASSIS_ZERO_FORCE);
                }
            }
            break;

        case CHASSIS_MODE_FOLLOW:
            if (Chassis.state != CHASSIS_STANDING)
            {
                if (attitude_ready != 0U)
                {
                    State_Enter(CHASSIS_STANDING);
                }
                else
                {
                    /*
                     * 左中合并了倒地自起：姿态不满足时不再直接零力矩报
                     * 故障，走一遍FALLEN->FALLING_TO_STAND自救链，稳定
                     * 后自动转入STANDING，全程mode保持FOLLOW，不需要
                     * 用户二次操作。
                     * 注意这个判断在mode边沿块内，只在拨杆切进左中那一拍
                     * 做一次；站立途中摔倒不会再次触发，见第4步的说明。
                     */
                    State_Enter(CHASSIS_FALLEN);
                }
            }
            break;

        case CHASSIS_MODE_SELF_SAVE:
            State_Enter(CHASSIS_FALLEN);
            break;

        case CHASSIS_MODE_BENCH:
            State_Enter(CHASSIS_BENCH);
            break;

        case CHASSIS_MODE_STEP:
            State_Enter(CHASSIS_STEP);
            break;

        case CHASSIS_MODE_ZERO_FORCE:
        default:
            State_Enter(CHASSIS_ZERO_FORCE);
            break;
        }
        Chassis.last_mode = Chassis.mode;
    }

    /*
     * 4. 数学有效时才用当前腿角执行站立姿态保护。
     * 站立途中摔倒一律落到零力矩，不自动转FALLEN重爬：自动自起只在第3步
     * 的mode边沿判一次，摔倒后要重新自起得把拨杆拨走再拨回左中。这是有意
     * 保留的行为，调试时不希望车在没人操作的情况下自己动起来。
     */
    if ((Chassis.state == CHASSIS_STANDING) &&
        (leg_valid_flag != 0U) &&
        ((fabsf(theta_b) >
          Chassis_Config.recovery.pitch_limit) ||
          (Angle_In_Range(
               Chassis.leg[CHASSIS_LEFT].phi0_total,
               Chassis_Config.recovery.stand_phi0_min,
               Chassis_Config.recovery.stand_phi0_max) == 0U) ||
          (Angle_In_Range(
               Chassis.leg[CHASSIS_RIGHT].phi0_total,
               Chassis_Config.recovery.stand_phi0_min,
               Chassis_Config.recovery.stand_phi0_max) == 0U)))
    {
        Chassis.fault = output_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
    }

    /*
     * 5. 台阶动作腿摆角本来就大，用放宽后的限值单独做姿态保护。
     * 参考HERO_LEG磕台阶模式抬高倒地判断阈值的做法，避免越台阶瞬间误判，
     * 同时不让台阶状态完全失去翻倒保护。
     */
    if ((Chassis.state == CHASSIS_STEP) &&
        (leg_valid_flag != 0U) &&
        ((fabsf(theta_b) > Chassis_Config.step.pitch_limit) ||
         (Angle_In_Range(
              Chassis.leg[CHASSIS_LEFT].phi0_total,
              Chassis_Config.step.phi0_min,
              Chassis_Config.step.phi0_max) == 0U) ||
         (Angle_In_Range(
              Chassis.leg[CHASSIS_RIGHT].phi0_total,
              Chassis_Config.step.phi0_min,
              Chassis_Config.step.phi0_max) == 0U)))
    {
        Chassis.fault = output_fault | CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
    }

    /* 活动状态保留设备和数学故障，供末端输出门和Watch共同使用。 */
    if (Chassis.state != CHASSIS_ZERO_FORCE)
    {
        Chassis.fault = active_fault;
    }
}

/**
 * @brief 板凳和恢复模式的关节角度-速度串级控制。
 *
 * 逆运动学给出主动关节角目标，角度PID生成速度目标，速度PID生成
 * 力矩请求。输出故障只阻止请求量复制到最终关节力矩数组。
 *
 * 故障统一记录在 Chassis.fault；关节映射或逆解无定义时自行切入零力状态，
 * 后续轮输出阶段会因为 Chassis.fault 非零而自然封锁。
 */
static void Joint_Control(void)
{
    VMC_Joint_Target_t joint_target[CHASSIS_LEG_COUNT];
    float geometric_torque_nm;
    float target_speed_radps;
    float feedback_angle_rad;
    float feedback_speed_radps;
    float dt = Chassis.dt;
    uint32_t side;
    uint32_t joint;

    /* 1. 输出故障只参与末端许可，当前串级控制仍继续计算请求量。 */
    Chassis.fault = Output_Fault_Get();
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
    }

    /* 2. 目标腿长和连续phi0经逆运动学转换为四个关节目标角。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        uint8_t phi1_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        /* 配置索引直接用作关节数组下标，越界时不得继续访问数组。 */
        if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT) ||
            (VMC_Inverse_Calc(&Chassis_Config.leg[side],
                              &Chassis.leg[side],
                              Chassis.leg[side].target_L0,
                              Chassis.leg[side].target_phi0,
                              &joint_target[side]) == 0U))
        {
            Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Chassis.output.target_angle[phi1_index] = joint_target[side].phi1;
        Chassis.output.target_angle[phi4_index] = joint_target[side].phi4;
    }

    /* 3. 关节角度环输出目标速度，速度环输出有方向的力矩请求。 */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        for (joint = 0U; joint < CHASSIS_JOINT_COUNT; joint++)
        {
            uint8_t motor_index =
                Chassis_Config.leg[side].joint[joint].motor_index;
            float joint_scale =
                Chassis_Config.leg[side].joint[joint].scale *
                Chassis_Config.leg[side].joint[joint].ratio;

            if (joint == CHASSIS_JOINT_PHI1)
            {
                feedback_angle_rad = Chassis.leg[side].phi1;
            }
            else
            {
                feedback_angle_rad = Chassis.leg[side].phi4;
            }
            feedback_speed_radps =
                joint_scale * Chassis.dm_motor[motor_index].speed_radps;
            target_speed_radps = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &Chassis_Config.recovery.joint_angle_pid,
                &Chassis.joint_angle_pid[motor_index],
                Chassis.output.target_angle[motor_index],
                feedback_angle_rad,
                feedback_speed_radps,
                dt,
                &target_speed_radps);
            Chassis.output.target_speed[motor_index] =
                target_speed_radps;

            geometric_torque_nm = 0.0f;
            Algorithm_PID_UpdateByFeedbackRate(
                &Chassis_Config.recovery.joint_speed_pid,
                &Chassis.joint_speed_pid[motor_index],
                target_speed_radps,
                feedback_speed_radps,
                0.0f,
                dt,
                &geometric_torque_nm);
            /* 请求量不限幅，限幅统一在最终命令处；PID输出限幅已界定量级。 */
            Chassis.output.T_joint_req[motor_index] =
                geometric_torque_nm * joint_scale;
        }

        /*
         * 叠加腿自重的重力前馈：位置串级只需处理误差，不必独自扛住整条腿，
         * 因此增益可以调小、动作变柔和。参考HERO_LEG自救时在F0上加补偿的做法，
         * 但那台车有气弹簧且前馈输入是实测支撑力，这里只按腿自重建模。
         * 重力沿伸腿方向拉F0、theta为正时给出负摆矩，前馈取相反号抵消。
         */
        if (Chassis_Config.recovery.gravity_ff_scale != 0.0f)
        {
            const Chassis_Model_Config_t *model = &Chassis_Config.model;
            float leg_weight_n = model->leg_mass * model->gravity *
                                 Chassis_Config.recovery.gravity_ff_scale;
            float cm_arm_m = Chassis_Config.recovery.leg_cm_ratio *
                             Chassis.leg[side].L0;
            VMC_Torque_t gravity_torque;

            if (VMC_Torque_Calc(&Chassis_Config.leg[side],
                                &Chassis.leg[side],
                                -leg_weight_n * cosf(Chassis.leg[side].theta),
                                leg_weight_n * cm_arm_m *
                                    sinf(Chassis.leg[side].theta),
                                &gravity_torque) != 0U)
            {
                uint8_t phi1_motor =
                    Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1]
                        .motor_index;
                uint8_t phi4_motor =
                    Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4]
                        .motor_index;

                Chassis.output.T_joint_req[phi1_motor] += gravity_torque.T1;
                Chassis.output.T_joint_req[phi4_motor] += gravity_torque.T4;
            }
        }
    }

    /* 4. 仅在全部输出条件满足时，把调试请求限幅后复制到最终命令。 */
    if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.joint_flag != 0U) &&
        (Chassis_Config.output.joint_T_limit > 0.0f))
    {
        for (side = 0U; side < APP_DM_COUNT; side++)
        {
            Chassis.output.T_joint[side] =
                Algorithm_LimitSymmetric(Chassis.output.T_joint_req[side],
                                Chassis_Config.output.joint_T_limit);
        }
        Chassis.output.safe_flag = 0U;
    }
}

/**
 * @brief 翻身阶段：长腿双腿同步扫掠，把机体翻到腿朝下。
 *
 * 对应ZJU §13.6 的 TurnOver。两条腿必须同步同向扫掠，不同步机体翻不过来，
 * 所以两腿共用一个方向、任一腿卡死就整体反向。腿长先伸到 turnover_L0：
 * 长腿力臂大、转动惯量大，机体才撬得动——原先这里的目标和板凳姿态是
 * 同一个数，腿长全程不动，底朝天就只能靠甩，甩不过来。
 */
static void Recovery_Turnover(float dt)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    float direction;
    uint8_t stuck_flag = 0U;
    uint32_t side;

    /*
     * 扫掠方向是纯机构常量，不看姿态角——ZJU的原则："方向为旋转后恰腿摆
     * 在后面对应的方向"，跟机器人当前朝哪边倒无关。
     * 也不能看 fall_pitch：实测它过90度会折返，底朝天读0，和直立分不开，
     * 拿它选方向等于拿噪声选方向。整个翻身阶段只信竖向加速度。
     * 只锁存一次是为了防抖，两腿共用同一个值——不同步则机体翻不过来。
     */
    if (Chassis.recovery_direction[CHASSIS_LEFT] == 0.0f)
    {
        direction = recovery->turnover_dir_sign;
        Chassis.recovery_direction[CHASSIS_LEFT] = direction;
        Chassis.recovery_direction[CHASSIS_RIGHT] = direction;
    }
    direction = Chassis.recovery_direction[CHASSIS_LEFT];

    /*
     * 卡死计时按腿独立——合并成一个的话单腿卡住永远判不出来，另一条腿
     * 还在转。但反向必须整体反：只反一条腿，两腿就不同步了。
     */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (fabsf(Chassis.leg[side].d_phi0) < recovery->stuck_d_phi0)
        {
            Chassis.recovery_stuck_time[side] += dt;
        }
        else
        {
            Chassis.recovery_stuck_time[side] = 0.0f;
        }
        /* stuck_time <= 0 视为禁用，否则 0>=0 恒成立会每毫秒翻一次向。 */
        if ((recovery->stuck_time > 0.0f) &&
            (Chassis.recovery_stuck_time[side] >= recovery->stuck_time))
        {
            stuck_flag = 1U;
        }
    }
    if (stuck_flag != 0U)
    {
        direction = -direction;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.recovery_direction[side] = direction;
            Chassis.recovery_stuck_time[side] = 0.0f;
            /* 目标重锁到实际角，不重锁的话解卡瞬间目标已跑远，腿会甩出去。 */
            Chassis.leg[side].target_phi0 = Chassis.leg[side].phi0_total;
        }
    }

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 =
            Move_Toward(Chassis.leg[side].target_L0,
                        recovery->turnover_L0,
                        recovery->turnover_L0_rate * dt);
        Chassis.leg[side].target_phi0 +=
            direction * recovery->turnover_rate * dt;
        /* 腿被卡住时目标不能无限跑远，否则解卡瞬间会甩。 */
        Chassis.leg[side].target_phi0 = Algorithm_LimitRange(
            Chassis.leg[side].target_phi0,
            Chassis.leg[side].phi0_total - recovery->rotate_lead_max,
            Chassis.leg[side].phi0_total + recovery->rotate_lead_max);
    }

    /*
     * 退出判据用竖向加速度而不是 fall_pitch。ZJU的理由：倒地后四元数可能
     * 收敛到错误分支，而重力方向的加速度投影始终可靠。对我们还多一条好处：
     * az 是标量阈值，没有 fall_pitch 在 ±pi 附近符号跳变的问题。
     * ZJU式122的带惩罚计数：满足加dt，不满足直接减半。翻身过程中机体会晃，
     * 硬清零计数永远攒不起来；减半既容忍偶发抖动，又拒绝断续满足。
     */
    if (Body_Upward() != 0U)
    {
        Chassis.recovery_phase_hold += dt;
    }
    else
    {
        Chassis.recovery_phase_hold *= 0.5f;
    }

    if (Chassis.recovery_phase_hold >= recovery->turnover_hold)
    {
        /* 转入摆腿。方向和参考角两个阶段的选法不同，都要重锁。 */
        Chassis.recovery_phase = CHASSIS_RECOVERY_SWING;
        Chassis.recovery_phase_hold = 0.0f;
        Chassis.recovery_theta_ref = 0.0f;
        Chassis.stable_time = 0.0f;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.recovery_direction[side] = 0.0f;
            Chassis.recovery_stuck_time[side] = 0.0f;
        }
    }
}

/**
 * @brief 摆腿阶段：保持长腿，两腿独立摆到起立预备角。
 *
 * 对应ZJU §13.6 的 Swing。机体此时已经翻正，两腿各自就近摆进窗口即可，
 * 所以扫掠方向和卡死反向都按腿独立处理。
 */
static void Recovery_Swing(float dt)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    float theta[CHASSIS_LEG_COUNT];
    float rotate_rate[CHASSIS_LEG_COUNT];
    uint8_t theta_flag[CHASSIS_LEG_COUNT];
    float theta_ref;
    float theta_sign;
    uint32_t side;

    /*
     * 机体被甩翻回去就退回翻身阶段重来。摆腿时两条腿正在扫，机体有可能
     * 被自己的角动量带过头翻扣过去，这时继续摆腿是没有意义的。
     * 判据只看 az：fall_pitch 折返，倒扣时读数和直立分不开。
     * ZJU的Recovery是单向状态机、没有这条回退——他们的翻身速率高、一次
     * 到位；本机扫掠速率还在保守档，甩过头是现实存在的情况。
     * 计数用和翻身退出同一套带惩罚逻辑，两个方向共用一个累加器，
     * 天然防住在临界姿态上来回切。
     */
    if (Body_Upward() == 0U)
    {
        Chassis.recovery_phase_hold += dt;
    }
    else
    {
        Chassis.recovery_phase_hold *= 0.5f;
    }
    if (Chassis.recovery_phase_hold >= recovery->turnover_hold)
    {
        Chassis.recovery_phase = CHASSIS_RECOVERY_TURNOVER;
        Chassis.recovery_phase_hold = 0.0f;
        Chassis.recovery_theta_ref = 0.0f;
        Chassis.stable_time = 0.0f;
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.recovery_direction[side] = 0.0f;
            Chassis.recovery_stuck_time[side] = 0.0f;
        }
        return;
    }

    /*
     * 转腿目标落在机体倾倒的同一侧，从当前姿态转过去最近。
     * 参考ZJU：它的摆腿目标是单边写死的，因为翻身阶段已经把机体翻到已知
     * 一侧；我们的翻身方向由 turnover_dir_sign 决定、不保证是那一侧，
     * 所以目标仍然要跟着倒地方向对称取符号，否则往一个方向倒时要多转约
     * 两倍窗口中心角——这就是"舍近求远"。
     * 只在本阶段第一拍锁存：机体转过竖直位时 fall_pitch 会过零，每拍重算
     * 会让参考角来回翻符号、腿原地抖。
     */
    if (Chassis.recovery_theta_ref == 0.0f)
    {
        Chassis.recovery_theta_ref =
            ((Chassis.fall_pitch < 0.0f) ? -1.0f : 1.0f) *
            (recovery->theta_min + recovery->theta_max) * 0.5f;
    }
    theta_ref = Chassis.recovery_theta_ref;
    theta_sign = (theta_ref >= 0.0f) ? 1.0f : -1.0f;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        /*
         * 这里必须用 fall_pitch 而不是 theta_b：theta 是虚拟腿相对大地竖直方向
         * 的角，机体倒过90度以后 EKF pitch 会折返，theta 会跟着算成另一个值。
         */
        theta[side] = Algorithm_AngleNearestEquivalentRad(
            Chassis.leg[side].phi0_total -
                Chassis_Config.phi0_offset + Chassis.fall_pitch,
            theta_ref);
        /*
         * 窗口跟着 theta_ref 的符号走。这里不能写成 fabsf(theta)：
         * 腿摆到反侧同样能凑出合格的绝对值，但那一侧顶不起机体。
         */
        theta_flag[side] =
            (((theta[side] * theta_sign) >= recovery->theta_min) &&
             ((theta[side] * theta_sign) <= recovery->theta_max)) ?
                1U : 0U;
        rotate_rate[side] = recovery->rotate_rate;

        /* 扫掠方向按就近原则，每条腿各锁一次。 */
        if (Chassis.recovery_direction[side] == 0.0f)
        {
            Chassis.recovery_direction[side] =
                (theta[side] < theta_ref) ? 1.0f : -1.0f;
        }

        /* 摆腿阶段两腿本来就不要求同步，卡死可以只反这一条。 */
        if ((fabsf(Chassis.leg[side].d_phi0) < recovery->stuck_d_phi0) &&
            (theta_flag[side] == 0U))
        {
            Chassis.recovery_stuck_time[side] += dt;
        }
        else
        {
            Chassis.recovery_stuck_time[side] = 0.0f;
        }
        if ((recovery->stuck_time > 0.0f) &&
            (Chassis.recovery_stuck_time[side] >= recovery->stuck_time))
        {
            Chassis.recovery_direction[side] =
                -Chassis.recovery_direction[side];
            Chassis.recovery_stuck_time[side] = 0.0f;
            Chassis.leg[side].target_phi0 = Chassis.leg[side].phi0_total;
        }
    }

    /*
     * 机体仍明显倾斜且双腿进度不一致时，让距离参考角更远的一侧使用更大的
     * 追赶速率，避免一条腿提前停住。
     */
    if ((fabsf(theta[CHASSIS_LEFT] - theta[CHASSIS_RIGHT]) >
          recovery->theta_diff) &&
        (fabsf(Chassis.fall_pitch) > recovery->ready_pitch))
    {
        if (fabsf(theta[CHASSIS_LEFT] - theta_ref) >
            fabsf(theta[CHASSIS_RIGHT] - theta_ref))
        {
            rotate_rate[CHASSIS_LEFT] = recovery->lag_rate;
        }
        else
        {
            rotate_rate[CHASSIS_RIGHT] = recovery->lag_rate;
        }
    }

    /*
     * 腿角到位且机体已接近可准备姿态时目标停在原处；否则按速率继续推进。
     * pitch门槛防止严重倾斜时过早停止转腿。目标只按速率走、不跟随实际角，
     * 动作因此可控。
     */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        /* 摆腿全程保持长腿，ZJU的Swing同样是伸到lmax再摆。 */
        Chassis.leg[side].target_L0 =
            Move_Toward(Chassis.leg[side].target_L0,
                        recovery->extend_L0,
                        recovery->turnover_L0_rate * dt);
        if ((theta_flag[side] == 0U) ||
            (fabsf(Chassis.fall_pitch) > recovery->direct_pitch))
        {
            Chassis.leg[side].target_phi0 +=
                Chassis.recovery_direction[side] * rotate_rate[side] * dt;
        }
        /* 腿被卡住时目标不能无限跑远，否则解卡瞬间会甩。 */
        Chassis.leg[side].target_phi0 = Algorithm_LimitRange(
            Chassis.leg[side].target_phi0,
            Chassis.leg[side].phi0_total - recovery->rotate_lead_max,
            Chassis.leg[side].phi0_total + recovery->rotate_lead_max);
    }

    /*
     * 双腿到位且机体倾角足够小并持续稳定后才进入收腿站起阶段。
     * Body_Upward() 不能省：摆腿过程中腿在扫，机体有可能被自己甩过头翻扣
     * 过去，而 fall_pitch 折返后 170 度也只读出 0.17、照样满足 ready_pitch，
     * 于是带着倒扣的姿态判"到位"交给收腿站起。az 是这里唯一可信的量。
     */
    if ((theta_flag[CHASSIS_LEFT] != 0U) &&
        (theta_flag[CHASSIS_RIGHT] != 0U) &&
        (Body_Upward() != 0U) &&
        (fabsf(Chassis.fall_pitch) <= recovery->ready_pitch))
    {
        Chassis.stable_time += dt;
    }
    else
    {
        Chassis.stable_time = 0.0f;
    }

    if (Chassis.stable_time >= recovery->stable_time)
    {
        State_Enter(CHASSIS_FALLING_TO_STAND);
    }
}

/**
 * @brief 收腿站起阶段：摆角转到竖直向下、腿长收到最短，轮子把机体撑起。
 *
 * 对应ZJU §13.6 的 DrawBack。到位判据同时看 pitch、roll 和两腿等长：
 * 只查 pitch 的话侧躺时腿长腿角照样能到位，会直接交给站立控制，而那个
 * 姿态已经在LQR线性化域外。这里的 pitch 用 theta_b：机体已经接近直立，
 * EKF pitch 可靠，而且要和站立控制消费的是同一份。
 */
static void Recovery_Drawback(float dt, float theta_b)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    uint8_t prepare_flag;
    uint32_t side;

    prepare_flag =
        ((fabsf(theta_b) <= recovery->ready_pitch) &&
         (fabsf(Chassis.imu.roll *
                Chassis_Config.imu.roll_angle_scale) <=
          recovery->ready_roll) &&
         (fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                Chassis.leg[CHASSIS_RIGHT].L0) <=
          recovery->L0_diff_tol)) ? 1U : 0U;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        /* 腿长和腿角都按速率靠近板凳姿态，不是阶跃。 */
        Chassis.leg[side].target_L0 =
            Move_Toward(Chassis.leg[side].target_L0,
                        recovery->bench_L0,
                        recovery->drawback_L0_rate * dt);
        Chassis.leg[side].target_phi0 =
            Move_Toward(Chassis.leg[side].target_phi0,
                        Algorithm_AngleNearestEquivalentRad(
                            recovery->bench_phi0,
                            Chassis.leg[side].target_phi0),
                        recovery->drawback_phi0_rate * dt);
        Chassis.leg[side].target_phi0 = Algorithm_LimitRange(
            Chassis.leg[side].target_phi0,
            Chassis.leg[side].phi0_total - recovery->rotate_lead_max,
            Chassis.leg[side].phi0_total + recovery->rotate_lead_max);
        if ((fabsf(Chassis.leg[side].L0 - recovery->bench_L0) >
             recovery->L0_tol) ||
            (fabsf(Algorithm_AngleNormalizeRad(
                 recovery->bench_phi0 - Chassis.leg[side].phi0_total)) >
             recovery->angle_tol))
        {
            prepare_flag = 0U;
        }
    }

    if (prepare_flag != 0U)
    {
        Chassis.stable_time += dt;
    }
    else
    {
        Chassis.stable_time = 0.0f;
    }

    if (Chassis.stable_time >= recovery->stable_time)
    {
        State_Enter(CHASSIS_STANDING);
    }
}

/**
 * @brief 执行 TurnOver / Swing / DrawBack 三阶段重新站立状态机。
 *
 * 阶段划分取自ZJU §13.6，裁掉了我们没有的云台阶段 YawFront。TurnOver 和
 * Swing 都在 CHASSIS_FALLEN 下跑，子阶段见 Chassis.recovery_phase；
 * DrawBack 就是 CHASSIS_FALLING_TO_STAND。顶层状态枚举保持两个不变，
 * task 层的分发和观测器的自救判据都不受影响。
 */
void Chassis_Recovery(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    float theta_b;
    float dt = Chassis.dt;

    Output_Clear();

    if ((Chassis.state != CHASSIS_FALLEN) &&
        (Chassis.state != CHASSIS_FALLING_TO_STAND))
    {
        Chassis_Zero_Output();
        return;
    }
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        /*
         * 当前腿姿态不足以推进恢复阶段时冻结阶段跳转，但仍运行已有目标下
         * 的关节串级控制，保留可定义请求量供Watch观察。
         * 计时必须继续：几何长期无解会让恢复永不超时，模式和状态一起锁死。
         */
        float stuck_timeout = (Chassis.state == CHASSIS_FALLEN) ?
                                  recovery->fallen_timeout :
                                  recovery->prepare_timeout;

        Chassis.state_time += dt;
        if (Chassis.state_time >= stuck_timeout)
        {
            Chassis_Zero_Output();
            Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Joint_Control();
        return;
    }

    theta_b = Chassis.imu.pitch * Chassis_Config.imu.pitch_angle_scale;

    if (Chassis.state == CHASSIS_FALLEN)
    {
        Chassis.state_time += dt;

        /*
         * 姿态已经满足准备条件时两段都跳过，直接收腿站起。
         * ZJU把 TurnOver 和 Swing 都标成"可跳过"，就是这条。
         * Body_Upward() 不能省：fall_pitch 过90度会折返，底朝天时读数和
         * 直立分不开，只查它的话会带着机体倒扣的姿态直接进收腿站起。
         */
        if ((Body_Upward() != 0U) &&
            (fabsf(Chassis.fall_pitch) <= recovery->direct_pitch) &&
            (Angle_In_Range(
                 Chassis.leg[CHASSIS_LEFT].phi0_total,
                 recovery->phi0_min,
                 recovery->phi0_max) != 0U) &&
            (Angle_In_Range(
                 Chassis.leg[CHASSIS_RIGHT].phi0_total,
                 recovery->phi0_min,
                 recovery->phi0_max) != 0U))
        {
            State_Enter(CHASSIS_FALLING_TO_STAND);
        }
        else if (Chassis.recovery_phase == CHASSIS_RECOVERY_TURNOVER)
        {
            Recovery_Turnover(dt);
        }
        else
        {
            Recovery_Swing(dt);
        }

        /* 上面可能已经切走；超时判在阶段推进之后，同一拍成功优先于超时。 */
        if (Chassis.state == CHASSIS_FALLEN)
        {
            if (Chassis.state_time >= recovery->fallen_timeout)
            {
                Chassis_Zero_Output();
                Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
                State_Enter(CHASSIS_ZERO_FORCE);
                return;
            }
            Joint_Control();
        }
        return;
    }

    Chassis.state_time += dt;
    Recovery_Drawback(dt, theta_b);
    if (Chassis.state == CHASSIS_FALLING_TO_STAND)
    {
        if (Chassis.state_time >= recovery->prepare_timeout)
        {
            Chassis_Zero_Output();
            Chassis.fault = CHASSIS_FAULT_RECOVERY_TIMEOUT;
            State_Enter(CHASSIS_ZERO_FORCE);
            return;
        }
        Joint_Control();
    }
}

/**
 * @brief 保持板凳腿长/腿角，再复用主控制环计算轮LQR和关节位置环。
 *
 * 进入板凳时锁存当前实际姿态作为起点，此后按遥控模块给出的调节速率
 * 逐周期积分，腿长限制在机构可达区间内。
 */
void Chassis_Bench(void)
{
    const Chassis_Recovery_Config_t *recovery = &Chassis_Config.recovery;
    uint32_t side;

    if (Chassis.state_time == 0.0f)
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_L0 = Chassis.leg[side].L0;
            Chassis.leg[side].target_phi0 = Chassis.leg[side].phi0_total;
        }
    }
    Chassis.state_time += Chassis.dt;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 +=
            Chassis.goal.bench_d_L0[side] * Chassis.dt;
        Chassis.leg[side].target_phi0 +=
            Chassis.goal.bench_d_phi0[side] * Chassis.dt;
        Chassis.leg[side].target_L0 =
            fminf(fmaxf(Chassis.leg[side].target_L0,
                        recovery->bench_L0_min),
                  recovery->bench_L0_max);
    }

    Chassis_Control();
}

/** @brief 按当前模式缩放十维误差并完成四路LQR统一点乘。 */
static void LQR_Calc(void)
{
    float *T[CHASSIS_OUTPUT_COUNT] = {
        &Chassis.output.T_wheel[CHASSIS_LEFT],
        &Chassis.output.T_wheel[CHASSIS_RIGHT],
        &Chassis.leg[CHASSIS_LEFT].Tp,
        &Chassis.leg[CHASSIS_RIGHT].Tp,
    };
    uint32_t output;
    uint32_t state;

    /*
     * 整车离地优先于小陀螺：既在小陀螺又腾空时牵引力为零，位置和航向
     * 控制毫无意义，只剩腿摆通道可控。掩码取自ZJU式142。
     */
    if ((Chassis_Config.output.off_ground_act_flag != 0U) &&
        (Chassis.ground.all_off_flag != 0U))
    {
        memcpy(Chassis.lqr.scale,
               Chassis_Config.off_ground_scale,
               sizeof(Chassis.lqr.scale));
    }
    else if ((Chassis.mode == CHASSIS_MODE_TOP) &&
             (Chassis.state == CHASSIS_STANDING))
    {
        memcpy(Chassis.lqr.scale,
               Chassis_Config.top.scale,
               sizeof(Chassis.lqr.scale));
    }
    else
    {
        for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
        {
            Chassis.lqr.scale[state] = 1.0f;
        }
    }

    // Chassis.lqr.scale[CHASSIS_STATE_FAI] = 0.0;
    // Chassis.lqr.scale[CHASSIS_STATE_D_FAI] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_S] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_S] = 1.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_THETA_L] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_THETA_L] = 0.0f;

    // Chassis.lqr.scale[CHASSIS_STATE_THETA_B] = 0.0f;
    // Chassis.lqr.scale[CHASSIS_STATE_D_THETA_B] = 0.0f;
    /*
     * 误差先按状态限幅再进K点乘，防止位移积累或姿态瞬时越界时单一状态项
     * 主导四路输出。限幅只作用在误差输入端，不改K、不改状态顺序。
     */
    for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
    {
        float error = Chassis.lqr.target[state] - Chassis.lqr.x[state];
        float limit = Chassis_Config.lqr.error_limit[state];

        Chassis.lqr.error[state] =
            (limit > 0.0f) ? Algorithm_LimitSymmetric(error, limit) : error;
    }

    for (output = 0U; output < CHASSIS_OUTPUT_COUNT; output++)
    {
        float control_output = 0.0f;

        for (state = 0U; state < CHASSIS_STATE_COUNT; state++)
        {
            control_output +=
                lqrK[output][state] *
                Chassis.lqr.error[state] *
                Chassis.lqr.scale[state];
        }
        *T[output] = control_output;
    }
}

/**
 * @brief 完成速度融合、十维状态、支撑力、LQR和VMC整条控制链。
 *
 * 函数开始先清最终命令；输出故障不阻断中间量计算，只在末端阶段
 * 阻止请求量进入最终T_joint和I_wheel。
 */
void Chassis_Control(void)
{
    float measurement[ALGORITHM_KALMAN_MAX_MEASUREMENT_COUNT] = {0.0f};
    uint8_t off_ground_act_flag;
    float dt;
    float theta_b;
    float d_theta_b;
    uint32_t side;

    Output_Clear();
    Chassis.lqr.limit_flag = 0U;

    /* 1. 输出故障只封锁最终命令，中间控制量继续使用最新反馈计算。 */
    Chassis.fault = Output_Fault_Get();
    if ((Chassis.leg[CHASSIS_LEFT].valid_flag == 0U) ||
        (Chassis.leg[CHASSIS_RIGHT].valid_flag == 0U))
    {
        Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
    }

    /* 配置轴号随后直接用作 imu.gyro/body_accel 下标，越界不得继续。 */
    if ((Chassis_Config.imu.pitch_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.roll_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.yaw_rate_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.lateral_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.vertical_accel_axis >= APP_IMU_AXIS_COUNT) ||
        (Chassis_Config.imu.forward_accel_axis >= APP_IMU_AXIS_COUNT))
    {
        Chassis.fault |= CHASSIS_FAULT_CONTROL;
        State_Enter(CHASSIS_ZERO_FORCE);
        return;
    }

    dt = Chassis.dt;
    if (Chassis.state == CHASSIS_STANDING)
    {
        Motion_Update();
    }

    /* 轮速、IMU 和腿部状态组成速度融合的两个测量量。 */
    theta_b = Chassis.imu.pitch *
                     Chassis_Config.imu.pitch_angle_scale;
    d_theta_b =
        Chassis.imu.gyro[Chassis_Config.imu.pitch_rate_axis] *
        Chassis_Config.imu.pitch_rate_scale;

    /*
     * phi0和pitch均绕车体+Y增大。虚拟腿相对地面的绝对角为
     * theta = phi0 - pi/2 + pitch；多圈phi0只用于保持几何连续。
     */
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const float wheel_scale =
            (side == CHASSIS_LEFT) ?
                Chassis_Config.wheel.left_scale :
                Chassis_Config.wheel.right_scale;
        const Chassis_State_Index_t theta_index =
            (side == CHASSIS_LEFT) ?
                CHASSIS_STATE_THETA_L :
                CHASSIS_STATE_THETA_R;

        /* speed_rpm是电机转子转速，先除gear_ratio换算到轮轴角速度。 */
        Chassis.body.wheel_speed[side] =
            (float)Chassis.wheel_motor[side].speed_rpm *
            CHASSIS_RPM_TO_RADPS * wheel_scale /
            Chassis_Config.wheel.gear_ratio;
        Chassis.leg[side].theta = Algorithm_AngleNearestEquivalentRad(
            Chassis.leg[side].phi0_total -
                Chassis_Config.phi0_offset + theta_b,
            Chassis.lqr.target[theta_index]);
        Chassis.leg[side].d_theta =
            Chassis.leg[side].d_phi0 + d_theta_b;
        /*
         * ESC反馈是转子相对定子的速度。定子随虚拟腿转动，因此轮缘
         * 绝对速度还需加入R*d_theta，再叠加轮轴的腿部平动速度。
         */
        Chassis.body.side_speed[side] =
            Chassis_Config.wheel.R *
                (Chassis.body.wheel_speed[side] +
                 Chassis.leg[side].d_theta) +
            Chassis.leg[side].L0 * Chassis.leg[side].d_theta *
                cosf(Chassis.leg[side].theta) +
            Chassis.leg[side].d_L0 * sinf(Chassis.leg[side].theta);
    }
    Chassis.body.d_s_raw =
        0.5f * (Chassis.body.side_speed[CHASSIS_LEFT] +
                Chassis.body.side_speed[CHASSIS_RIGHT]);
    Chassis.body.dd_s =
        Chassis.imu.body_accel[Chassis_Config.imu.forward_accel_axis] *
        Chassis_Config.imu.forward_accel_scale;

    if (Chassis_Config.speed_kalman.enable_flag != 0U)
    {
        Chassis.speed_kalman.stateTransition[0] = 1.0f;
        Chassis.speed_kalman.stateTransition[1] = dt;
        Chassis.speed_kalman.stateTransition[2] = 0.0f;
        Chassis.speed_kalman.stateTransition[3] = 1.0f;
        measurement[0] = Chassis.body.d_s_raw;
        measurement[1] = Chassis.body.dd_s;
        Algorithm_Kalman_Update(&Chassis.speed_kalman, measurement);
        Chassis.body.d_s = Chassis.speed_kalman.state[0];
        Chassis.body.dd_s_fused = Chassis.speed_kalman.state[1];
    }
    else
    {
        Chassis.body.d_s = Chassis.body.d_s_raw;
        Chassis.body.dd_s_fused = Chassis.body.dd_s;
        Chassis.speed_kalman.state[0] = Chassis.body.d_s;
        Chassis.speed_kalman.state[1] = Chassis.body.dd_s_fused;
    }

    if ((fabsf(Chassis.lqr.target[CHASSIS_STATE_D_S]) <= 1.0e-4f) &&
        (Chassis_Config.speed_kalman.position_d_s_limit > 0.0f) &&
        (fabsf(Chassis.body.d_s) <=
         Chassis_Config.speed_kalman.position_d_s_limit))
    {
        Chassis.body.s += Chassis.body.d_s * dt;
    }
    else
    {
        Chassis.body.s = 0.0f;
    }

    Chassis.body.fai =
        Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
    Chassis.body.d_fai =
        Chassis.imu.gyro[Chassis_Config.imu.yaw_rate_axis] *
        Chassis_Config.imu.yaw_rate_scale;

    /* 十维数组只表达模型接口，物理状态在 body 和 leg 中各有唯一所有者。 */
    Chassis.lqr.x[CHASSIS_STATE_S] = Chassis.body.s;
    Chassis.lqr.x[CHASSIS_STATE_D_S] = Chassis.body.d_s;
    Chassis.lqr.x[CHASSIS_STATE_FAI] = Chassis.body.fai;
    Chassis.lqr.x[CHASSIS_STATE_D_FAI] = Chassis.body.d_fai;
    Chassis.lqr.x[CHASSIS_STATE_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].theta;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_L] =
        Chassis.leg[CHASSIS_LEFT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].theta;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_R] =
        Chassis.leg[CHASSIS_RIGHT].d_theta;
    Chassis.lqr.x[CHASSIS_STATE_THETA_B] = theta_b;
    Chassis.lqr.x[CHASSIS_STATE_D_THETA_B] = d_theta_b;
    /*
     * 四套只读观测，各自先算观测量再出判定。
     * 顺序有依赖：转向的转弯半径要先知道是否打滑，卡腿要先知道是否整车离地。
     */
    Chassis_Slip_Update(&Chassis_Config, &Chassis);
    Chassis_Slip_Calc(&Chassis_Config, &Chassis);
    Chassis_Ground_Update(&Chassis_Config, &Chassis);
    Chassis_Ground_Calc(&Chassis_Config, &Chassis);
    Chassis_Turn_Update(&Chassis_Config, &Chassis);
    Chassis_Turn_Calc(&Chassis_Config, &Chassis);
    Chassis_Stuck_Update(&Chassis_Config, &Chassis);
    Chassis_Stuck_Calc(&Chassis_Config, &Chassis);

    /*
     * 整车离地动作的边沿处理。三项动作(悬空腿下压、悬空轮清零、LQR掩码)统一
     * 挂在整车级all_off_flag上：单腿过坎或压弹丸时短暂卸载是常态，逐腿直接
     * 动控制会在正常行驶中乱切。ZJU同样是先把整车切进Flight模式再逐腿细分。
     */
    off_ground_act_flag =
        ((Chassis_Config.output.off_ground_act_flag != 0U) &&
         (Chassis.ground.all_off_flag != 0U)) ? 1U : 0U;
    if (Chassis.ground.all_off_flag != Chassis.last_all_off_flag)
    {
        if (Chassis.ground.all_off_flag != 0U)
        {
            /* 起飞：锁存腾空前的腿长指令。 */
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.off_ground_L0_latch[side] =
                    Chassis.leg[side].target_L0;
            }
        }
        else if (Chassis_Config.output.off_ground_act_flag != 0U)
        {
            /*
             * 落地交接。腾空期间腿被推出去、腿长和横滚PID在没有地面反力的
             * 情况下积分卷了一路、航向和位移参考也漂了，都要还原。
             * 刻意不调Control_Reset()：它会连带重置四套观测器，而地面观测
             * 刚刚才判出落地，重置它自相矛盾。
             */
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                Chassis.leg[side].target_L0 =
                    Chassis.off_ground_L0_latch[side];
            }
            Algorithm_PID_Init(&Chassis.leg_length_pid);
            Algorithm_PID_Init(&Chassis.roll_pid);
            Chassis.body.s = 0.0f;
            Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
            Chassis.lqr.target[CHASSIS_STATE_FAI] =
                Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
        }
        Chassis.last_all_off_flag = Chassis.ground.all_off_flag;
    }

    /* 3. 小板凳由关节位置环保持腿姿态，不再叠加腿长和横滚支撑力。 */
    if (Chassis.state == CHASSIS_BENCH)
    {
        Chassis.leg[CHASSIS_LEFT].F0 = 0.0f;
        Chassis.leg[CHASSIS_RIGHT].F0 = 0.0f;
    }
    else
    {
        float gravity_force[CHASSIS_LEG_COUNT];
        float roll =
            Chassis.imu.roll * Chassis_Config.imu.roll_angle_scale;
        float d_roll =
            Chassis.imu.gyro[Chassis_Config.imu.roll_rate_axis] *
            Chassis_Config.imu.roll_rate_scale;
        /* 共模：左右平均腿长，代表车身高度。 */
        float H = 0.5f * (Chassis.leg[CHASSIS_LEFT].L0 +
                          Chassis.leg[CHASSIS_RIGHT].L0);
        float H_target = 0.5f * (Chassis.leg[CHASSIS_LEFT].target_L0 +
                                 Chassis.leg[CHASSIS_RIGHT].target_L0);
        float d_H = 0.5f * (Chassis.leg[CHASSIS_LEFT].d_L0 +
                            Chassis.leg[CHASSIS_RIGHT].d_L0);
        float height_force = 0.0f;
        float roll_force = 0.0f;

        /*
         * 支撑力按共模和差模两路生成，两路物理上正交，互不干扰。
         *
         *   共模 = 车身高度：用左右平均腿长，输出等量加到两条腿；
         *   差模 = 车身横滚：用IMU横滚角，输出反号加到两条腿。
         *
         * 早先是每条腿各跑一个腿长PID再叠加横滚差动，两者在差模上直接对抗：
         * 横滚控制靠制造左右腿长差来纠正车身，而两条腿共享同一个target_L0，
         * 腿长PID一见到腿长差就反向拉回，横滚能力被按增益比例吃掉，且与增益
         * 大小无关。改成共模后腿长PID不再看见腿长差，两个环各管一个自由度。
         */

        /*共模：车身高度PID*/
        Algorithm_PID_UpdateByFeedbackRate(&Chassis_Config.leg_length_pid,
                                           &Chassis.leg_length_pid,
                                           H_target,
                                           H,
                                           d_H,
                                           dt,
                                           &height_force);
        /*差模：横滚PID*/
        Algorithm_PID_UpdateByFeedbackRate(&Chassis_Config.roll_pid,
                                           &Chassis.roll_pid,
                                           Chassis_Config.roll_target,
                                           roll,
                                           d_roll,
                                           dt,
                                           &roll_force);
        /*
         * 重力前馈：整车重力在虚拟腿轴向的投影，单腿承担一半。
         * 腿摆得越靠前或靠后，轴向需要承担的分量越小，因此乘cos(theta)。
         * F0符号约定为正向撑地，gravity_force取负值后由下方减号还原成正贡献。
         */
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            gravity_force[side] =
                -0.5f * Chassis_Model_Mass(&Chassis_Config.model) *
                Chassis_Config.model.gravity *
                Chassis_Config.F0_gravity_scale *
                cosf(Chassis.leg[side].theta);
        }
        /*
         * MPC按固定拍数分频求解，不用累加实际dt——模型里的Ts是按decimation
         * 写死的，dt抖动会让离散模型失配。
         *
         * ⚠ 求解必须挂在 mpc_flag 下面。原先写成"开关关着也照算，方便在Watch
         * 里对照两路输出"，结果 Debug(-O0) 下 Eigen 模板完全没优化，单次求解
         * 撑爆了 1kHz 的控制周期：底盘任务被拖住 -> Chassis_Command_Send() 停发
         * CAN -> DM电机在MIT模式下保持最后一条力矩 -> 腿持续出力且遥控拨回中位
         * 也没人处理。实机表现就是自起正常、一进STANDING立刻疯车且断不了电。
         * 想对照两路输出，等 Release 上把 Chassis_MPC.cycles_max 量清楚再开。
         */
        if (Chassis_Config.output.mpc_flag != 0U)
        {
            Chassis.mpc_tick++;
            if (Chassis.mpc_tick >= Chassis_Config.mpc.decimation)
            {
                float mpc_x0[CHASSIS_STATE_MPC_COUNT];

                Chassis.mpc_tick = 0U;
                mpc_x0[0] = roll;
                mpc_x0[1] = d_roll;
                mpc_x0[2] = H;
                mpc_x0[3] = d_H;
                Chassis_MPC_Solve(mpc_x0, H_target);
            }
        }

        /* F0_left/F0_right 的正负号沿用原写法，两者当前均为0。 */
        if (Chassis_Config.output.mpc_flag != 0U)
        {
            /*
             * MPC直接给绝对支撑力：重力已经作为仿射项在模型里，不再叠加
             * 重力前馈，也不再走共模腿长PID和差模横滚PID那两路。
             */
            Chassis.leg[CHASSIS_LEFT].F0 = Chassis_MPC.F[0];
            Chassis.leg[CHASSIS_RIGHT].F0 = Chassis_MPC.F[1];
        }
        else
        {
            Chassis.leg[CHASSIS_LEFT].F0 =
                height_force + roll_force -
                gravity_force[CHASSIS_LEFT] +
                Chassis_Config.F0_left;
            Chassis.leg[CHASSIS_RIGHT].F0 =
                height_force - roll_force -
                gravity_force[CHASSIS_RIGHT] -
                Chassis_Config.F0_right;
        }

        /*
         * 悬空腿主动伸腿找地。F0大于零即为伸腿撑地，直接叠加。
         * 腿长快到拟合上限就停止加推力，否则悬空腿会一直顶到机构限位
         * ——ZJU「悬空腿接近最大腿长后不再施加额外推力」。
         */
        if (off_ground_act_flag != 0U)
        {
            for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
            {
                if (Chassis.leg[side].L0 <
                    (Chassis_Config.lqr.L0_max -
                     Chassis_Config.observer.off_comp_L0_margin))
                {
                    Chassis.leg[side].F0 += Chassis.ground.fn_comp[side];
                }
            }
        }
    }

    /* 4. 始终根据左右实时腿长拟合K，固定目标腿长不伪造K输入。 */
    Algorithm_LQR_FitLqrKPoly22(
        Chassis_Config.lqr.coefficients,
        CHASSIS_OUTPUT_COUNT,
        CHASSIS_STATE_COUNT,
        Chassis.leg[CHASSIS_LEFT].L0,
        Chassis.leg[CHASSIS_RIGHT].L0,
        Chassis_Config.lqr.L0_min,
        Chassis_Config.lqr.L0_max,
        &lqrK[0][0],
        &Chassis.leg[CHASSIS_LEFT].K_L0_fit,
        &Chassis.leg[CHASSIS_RIGHT].K_L0_fit,
        &Chassis.lqr.limit_flag);

    /* 5. TOP只屏蔽位移和航向位置反馈，K矩阵和状态顺序保持不变。 */
    LQR_Calc();

    /*
     * LESO估出的总扰动与输入同通道同单位，直接从四路广义输出里减掉。
     * 上一周期的估计供本周期消费，这一拍延迟是因果实现躲不掉的。
     * 放在这里保证轮和关节各自唯一的限幅点仍在补偿之后生效。
     */
    if (Chassis.leso.gate_flag != 0U)
    {
        Chassis.output.T_wheel[CHASSIS_LEFT] -= Chassis.leso.d_comp[0];
        Chassis.output.T_wheel[CHASSIS_RIGHT] -= Chassis.leso.d_comp[1];
        Chassis.leg[CHASSIS_LEFT].Tp -= Chassis.leso.d_comp[2];
        Chassis.leg[CHASSIS_RIGHT].Tp -= Chassis.leso.d_comp[3];
    }

    // Chassis.output.T_wheel[CHASSIS_LEFT] = 0.15f;
    // Chassis.output.T_wheel[CHASSIS_RIGHT] = 0.15f;

    if (Chassis.state == CHASSIS_BENCH)
    {
        Chassis.leg[CHASSIS_LEFT].Tp = 0.0f;
        Chassis.leg[CHASSIS_RIGHT].Tp = 0.0f;
    }
    /*
     * 悬空轮不驱动：空转没有意义，落地瞬间轮速与地面不匹配会直接打滑。
     * 触地那条腿的轮子照常由LQR驱动，逐腿判断。放在限幅之前，
     * 保证轮通道唯一的限幅点仍然生效。
     */
    if (off_ground_act_flag != 0U)
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            if (Chassis.ground.off_ground_flag[side] != 0U)
            {
                Chassis.output.T_wheel[side] = 0.0f;
            }
        }
    }
    /* 轮通道只在力矩侧限幅一次，电调命令由换算得到，不再二次限幅。 */
    if ((Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f))
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            float wheel_scale =
                (side == CHASSIS_LEFT) ?
                    Chassis_Config.wheel.left_scale :
                    Chassis_Config.wheel.right_scale;

            Chassis.output.I_wheel_req[side] = (int16_t)(
                Algorithm_LimitSymmetric(Chassis.output.T_wheel[side],
                                Chassis_Config.wheel.T_limit) *
                Chassis_Config.wheel.T_to_I * wheel_scale);
        }
    }

    /* 6. 板凳使用关节串级，其他状态由VMC生成四路关节请求。 */
    if (Chassis.state == CHASSIS_BENCH)
    {
        Joint_Control();
    }
    else
    {
        VMC_Torque_t torque[CHASSIS_LEG_COUNT] = {0};

        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            uint8_t phi1_index =
                Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
            uint8_t phi4_index =
                Chassis_Config.leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

            /* 配置索引直接用作关节数组下标，越界时不得继续访问数组。 */
            if ((phi1_index >= APP_DM_COUNT) || (phi4_index >= APP_DM_COUNT))
            {
                Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
                State_Enter(CHASSIS_ZERO_FORCE);
                return;
            }
            if (VMC_Torque_Calc(&Chassis_Config.leg[side],
                                &Chassis.leg[side],
                                Chassis.leg[side].F0,
                                Chassis.leg[side].Tp,
                                &torque[side]) == 0U)
            {
                Chassis.fault |= CHASSIS_FAULT_KINEMATICS;
            }
            Chassis.output.T_joint_req[phi1_index] = torque[side].T1;
            Chassis.output.T_joint_req[phi4_index] = torque[side].T4;
        }

        /* 7. VMC请求经总开关、分路开关和限幅后进入最终关节命令。 */
        if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
            (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
            (Chassis_Config.output.joint_flag != 0U) &&
            (Chassis_Config.output.joint_T_limit > 0.0f))
        {
            for (side = 0U; side < APP_DM_COUNT; side++)
            {
                Chassis.output.T_joint[side] =
                    Algorithm_LimitSymmetric(Chassis.output.T_joint_req[side],
                                    Chassis_Config.output.joint_T_limit);
            }
            Chassis.output.safe_flag = 0U;
        }
    }

    /* 8. 轮请求在全部输出条件满足时进入最终CAN电流数组。 */
    if ((Chassis.fault == CHASSIS_FAULT_NONE) &&
        (APP_CHASSIS_OUTPUT_ENABLE != 0U) &&
        (Chassis_Config.output.wheel_flag != 0U) &&
        (Chassis_Config.wheel.T_limit > 0.0f) &&
        (Chassis_Config.wheel.T_to_I != 0.0f))
    {
        memcpy(Chassis.output.I_wheel,
               Chassis.output.I_wheel_req,
               sizeof(Chassis.output.I_wheel));
        Chassis.output.safe_flag = 0U;
    }

    /*
     * 9. 四路输出定稿后才推进扩张状态观测：此时lqr.x是本周期测量、
     *    output里是本周期实际施加的命令，两者同拍，无需引入输入滞后。
     */
    Chassis_Leso_Update(&Chassis_Config, &Chassis);
    Chassis_Leso_Calc(&Chassis_Config, &Chassis);
}

/**
 * @brief 切换爬台阶阶段。
 *
 * 进入两段动作时把腿角目标从当前实际角接管，否则位置串级第一拍就带着
 * 常值超前，关节PID会满输出。从RECOVER转回PREPARE等于跨完一级台阶，
 * 参照ZJU的Return做一次完整交接：两段动作期间不跑Chassis_Control()，
 * 速度卡尔曼和body.s停在上台阶之前的值，不复位就开下一级会突然纠偏。
 */
static void Step_Phase_Enter(Chassis_Step_Phase_t phase)
{
    uint32_t side;

    if ((phase == CHASSIS_STEP_PREPARE) &&
        (Chassis.step_phase == CHASSIS_STEP_RECOVER))
    {
        Control_Reset();
        memcpy(Chassis.lqr.target,
               Chassis_Config.target,
               sizeof(Chassis.lqr.target));
        Chassis.lqr.target[CHASSIS_STATE_S] = 0.0f;
        Chassis.step_fai =
            Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
        Chassis.lqr.target[CHASSIS_STATE_FAI] = Chassis.step_fai;
        memset(Chassis.contact_time, 0, sizeof(Chassis.contact_time));
        memset(Chassis.step_contact_flag,
               0,
               sizeof(Chassis.step_contact_flag));
        memset(Chassis.step_contact_latch_flag,
               0,
               sizeof(Chassis.step_contact_latch_flag));
        Chassis.step_posture_flag = 0U;
    }

    Chassis.step_phase = phase;
    if ((phase == CHASSIS_STEP_MOTION1) ||
        (phase == CHASSIS_STEP_MOTION2))
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            Chassis.leg[side].target_phi0 = Chassis.leg[side].phi0_total;
        }
    }
}

/**
 * @brief 更新左右轮碰撞候选，两路判据并联，任一成立即计入消抖。
 *
 * 第一路是原有的轮力矩判据：请求、反馈和腿角同时超限。
 * 第二路取自ZJU台阶检测，是整车级的主判据与辅判据相与——ZJU指出轮力矩这类
 * 信号与颠簸、急减速、踩弹丸难以区分，而"指令速度不为零却跟不上"只有真被
 * 挡住才会出现。两路的结论分别留在step_contact_flag和step_posture_flag里，
 * Watch中可以直接看出是哪一路把车切进MOTION1的。
 */
static void Step_Contact_Update(float dt)
{
    const Chassis_Step_Config_t *config = &Chassis_Config.step;
    float theta_b = Chassis.lqr.x[CHASSIS_STATE_THETA_B];
    float d_theta_b = Chassis.lqr.x[CHASSIS_STATE_D_THETA_B];
    float d_s_ref = Chassis.lqr.target[CHASSIS_STATE_D_S];
    float d_s = Chassis.lqr.x[CHASSIS_STATE_D_S];
    uint8_t main_flag;
    uint8_t sub_flag;
    uint32_t side;

    /* 主判据P：机体已经被顶得低头，且至少一条腿被迫后摆。 */
    main_flag =
        ((fabsf(theta_b) > config->contact_pitch) &&
         ((fabsf(Chassis.leg[CHASSIS_LEFT].theta) > config->contact_theta) ||
          (fabsf(Chassis.leg[CHASSIS_RIGHT].theta) >
           config->contact_theta))) ? 1U : 0U;
    /* 辅判据S：四条里任一成立。速度跟踪误差那条是最能区分台阶的。 */
    sub_flag =
        ((fabsf(d_theta_b) > config->contact_d_pitch) ||
         ((fabsf(d_s_ref - d_s) > config->contact_d_s_err) &&
          (fabsf(d_s_ref) > config->contact_d_s_min)) ||
         (fabsf(theta_b) > config->contact_pitch_hard) ||
         (fabsf(Chassis.leg[CHASSIS_LEFT].theta) >
          config->contact_theta_hard) ||
         (fabsf(Chassis.leg[CHASSIS_RIGHT].theta) >
          config->contact_theta_hard)) ? 1U : 0U;
    Chassis.step_posture_flag =
        ((main_flag != 0U) && (sub_flag != 0U)) ? 1U : 0U;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float request_torque_nm =
            fabsf(Chassis.output.T_wheel[side]);
        float feedback_torque_nm = 0.0f;

        if (fabsf(Chassis_Config.wheel.T_to_I) > 1.0e-6f)
        {
            feedback_torque_nm =
                fabsf((float)Chassis.wheel_motor[side].current /
                      Chassis_Config.wheel.T_to_I);
        }
        Chassis.step_contact_flag[side] =
            ((request_torque_nm >= config->contact_T_req) &&
             (feedback_torque_nm >= config->contact_T_fb) &&
             (fabsf(Chassis.leg[side].theta) >=
              config->contact_theta)) ? 1U : 0U;

        if (Chassis.step_contact_latch_flag[side] != 0U)
        {
            continue;
        }
        /* 姿态路是整车级结论，成立时两条腿一起进消抖。 */
        if ((Chassis.step_contact_flag[side] != 0U) ||
            (Chassis.step_posture_flag != 0U))
        {
            Chassis.contact_time[side] += dt;
            if (Chassis.contact_time[side] >=
                config->contact_time)
            {
                Chassis.step_contact_latch_flag[side] = 1U;
            }
        }
        else
        {
            Chassis.contact_time[side] = 0.0f;
        }
    }
}

/**
 * @brief 执行五阶段辅助爬台阶。
 *
 * PREPARE/APPROACH/RECOVER 复用整条LQR+VMC控制链；MOTION1/MOTION2 取自ZJU的
 * Onestep，走关节位置串级(Joint_Control)，目标角和目标腿长各自限速推进，
 * 期间不经过LQR。轮电流在任一腿判定撞上台阶之后就全程清零。
 *
 * 接触锁存后只清零I_wheel，LQR轮力矩和I_wheel_req继续更新，
 * 便于在Watch中判断碰撞条件和后续控制请求。
 */
void Chassis_Step(void)
{
    const Chassis_Step_Config_t *config = &Chassis_Config.step;
    float dt = Chassis.dt;
    float target_length_m;
    float target_speed_mps = 0.0f;
    float target_angle_rad = 0.0f;
    float theta_b;
    uint8_t contact_flag;
    uint32_t side;

    if (Chassis.state != CHASSIS_STEP)
    {
        Chassis_Zero_Output();
        return;
    }

    theta_b = Chassis.imu.pitch * Chassis_Config.imu.pitch_angle_scale;

    /*
     * MOTION1/MOTION2：两段位置型动作。目标腿角和目标腿长各自按限速推进，
     * 由关节串级跟踪，不进LQR，因此轮力矩自然全程为零。
     * MOTION1把腿后摆到swing_phi0并伸到mid_L0，让前轮搭上台阶沿；
     * MOTION2把腿收到最短并转到大地竖直向下，把机体拉上台阶。
     */
    if ((Chassis.step_phase == CHASSIS_STEP_MOTION1) ||
        (Chassis.step_phase == CHASSIS_STEP_MOTION2))
    {
        float target_phi0_rad;
        float phi0_rate_radps;
        uint8_t reach_flag = 1U;

        if (Chassis.step_phase == CHASSIS_STEP_MOTION1)
        {
            target_length_m = config->mid_L0;
            target_phi0_rad = Chassis_Config.phi0_offset + config->swing_phi0;
            phi0_rate_radps = config->swing_phi0_rate;
        }
        else
        {
            target_length_m = config->retract_L0;
            /* 减去机体俯仰即大地竖直向下，等价ZJU的 pi/2 - theta_b。 */
            target_phi0_rad = Chassis_Config.phi0_offset - theta_b;
            phi0_rate_radps = config->home_phi0_rate;
        }

        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            /* 纯五连杆正解的腿角，相对车体，不含pitch，即ZJU的"机构角"。 */
            float phi0_rel_rad = Chassis.leg[side].phi0_total -
                                 Chassis_Config.phi0_offset;
            float angle_error_rad;
            float angle_tol_rad;

            Chassis.leg[side].target_L0 =
                Move_Toward(Chassis.leg[side].target_L0,
                            target_length_m,
                            config->climb_L0_rate * dt);
            Chassis.leg[side].target_phi0 =
                Move_Toward(Chassis.leg[side].target_phi0,
                            Algorithm_AngleNearestEquivalentRad(
                                target_phi0_rad,
                                Chassis.leg[side].target_phi0),
                            phi0_rate_radps * dt);

            /*
             * 到位判据必须和该段目标同口径，否则判的不是"跟没跟上"。
             * MOTION1目标是车体系机构角，判据就用机构角，不掺theta_b；
             * ZJU原文这里比的是含pitch的theta_l，展开等于
             * |腿跟踪误差 + theta_b|，机体俯仰大时会永远退不出——他们有
             * 3秒模式超时兜底，本工程没有，所以改成同口径。
             * MOTION2目标是大地竖直，判据就用含pitch的theta，同样是同口径。
             */
            if (Chassis.step_phase == CHASSIS_STEP_MOTION1)
            {
                angle_error_rad = phi0_rel_rad - config->swing_phi0;
                angle_tol_rad = config->swing_phi0_tol;
            }
            else
            {
                angle_error_rad = phi0_rel_rad + theta_b;
                angle_tol_rad = config->home_theta_tol;
            }
            if ((fabsf(Chassis.leg[side].L0 - target_length_m) >
                 config->climb_L0_tol) ||
                (fabsf(angle_error_rad) > angle_tol_rad))
            {
                reach_flag = 0U;
            }
        }

        Joint_Control();
        memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
        if (Chassis.state != CHASSIS_STEP)
        {
            return;
        }
        if (reach_flag != 0U)
        {
            Step_Phase_Enter(
                (Chassis.step_phase == CHASSIS_STEP_MOTION1) ?
                    CHASSIS_STEP_MOTION2 :
                    CHASSIS_STEP_RECOVER);
        }
        return;
    }

    target_length_m = config->approach_L0;
    if (Chassis.step_phase == CHASSIS_STEP_APPROACH)
    {
        target_speed_mps = Algorithm_LimitSymmetric(
            Chassis.goal.d_s,
            config->approach_d_s);
        if (target_speed_mps < 0.0f)
        {
            target_speed_mps = 0.0f;
        }
    }
    else if (Chassis.step_phase == CHASSIS_STEP_RECOVER)
    {
        target_length_m = Chassis.goal.L0;
        target_angle_rad = config->recover_theta;
    }

    Chassis.lqr.target[CHASSIS_STATE_D_S] =
        Move_Toward(Chassis.lqr.target[CHASSIS_STATE_D_S],
                           target_speed_mps,
                           APP_RC_VEL_RATE * dt);
    /*
     * 准备和接近阶段要能转向对准台阶：右摇杆的角速度积分进step_fai，
     * 松杆瞬间把step_fai锁到当前航向，和站立模式同一套手感。
     * 磕上台阶后(RECOVER)航向锁死不再接受摇杆：那一段轮电流已被清零，
     * 转向本来就无从执行，继续积分只会让目标漂走，等轮子恢复输出时
     * 突然甩一下。MOTION1/MOTION2 走位置串级，根本不到这里。
     */
    if ((Chassis.step_phase == CHASSIS_STEP_PREPARE) ||
        (Chassis.step_phase == CHASSIS_STEP_APPROACH))
    {
        if (Chassis.goal.d_fai != 0.0f)
        {
            Chassis.step_fai += Chassis.goal.d_fai * dt;
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = Chassis.goal.d_fai;
            Chassis.yaw_stick_flag = 1U;
        }
        else
        {
            Chassis.lqr.target[CHASSIS_STATE_D_FAI] = 0.0f;
            if (Chassis.yaw_stick_flag != 0U)
            {
                Chassis.step_fai =
                    Chassis.imu.yaw_total *
                    Chassis_Config.imu.yaw_angle_scale;
                Chassis.yaw_stick_flag = 0U;
            }
        }
    }
    else
    {
        Chassis.lqr.target[CHASSIS_STATE_D_FAI] = 0.0f;
    }
    Chassis.lqr.target[CHASSIS_STATE_FAI] = Chassis.step_fai;
    Chassis.lqr.target[CHASSIS_STATE_THETA_L] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_THETA_R] = target_angle_rad;
    Chassis.lqr.target[CHASSIS_STATE_D_THETA_L] = 0.0f;
    Chassis.lqr.target[CHASSIS_STATE_D_THETA_R] = 0.0f;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        Chassis.leg[side].target_L0 =
            Move_Toward(
                Chassis.leg[side].target_L0,
                target_length_m,
                config->L0_rate * dt);
    }

    Chassis_Control();
    if (Chassis.state != CHASSIS_STEP)
    {
        return;
    }

    switch (Chassis.step_phase)
    {
    case CHASSIS_STEP_PREPARE:
        if ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                   config->approach_L0) <=
             config->L0_tol) &&
            (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                   config->approach_L0) <=
             config->L0_tol))
        {
            Step_Phase_Enter(CHASSIS_STEP_APPROACH);
        }
        break;

    case CHASSIS_STEP_APPROACH:
        Step_Contact_Update(dt);
        if ((Chassis.step_contact_latch_flag[CHASSIS_LEFT] != 0U) &&
            (Chassis.step_contact_latch_flag[CHASSIS_RIGHT] != 0U))
        {
            Step_Phase_Enter(CHASSIS_STEP_MOTION1);
        }
        break;

    case CHASSIS_STEP_RECOVER:
        if ((fabsf(Chassis.leg[CHASSIS_LEFT].L0 -
                   target_length_m) <=
             config->L0_tol) &&
            (fabsf(Chassis.leg[CHASSIS_RIGHT].L0 -
                   target_length_m) <=
             config->L0_tol) &&
            (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_L] -
                   config->recover_theta) <=
             config->angle_tol) &&
            (fabsf(Chassis.lqr.x[CHASSIS_STATE_THETA_R] -
                   config->recover_theta) <=
             config->angle_tol))
        {
            /*
             * 一次台阶跨越完成，回PREPARE等待下一级；只有外部mode变化
             * （拨杆离开左上）才会真正退出STEP状态。回PREPARE时
             * Step_Phase_Enter会做一次完整交接复位。
             */
            Step_Phase_Enter(CHASSIS_STEP_PREPARE);
        }
        break;

    default:
        break;
    }

    /*
     * 任一腿判定撞上台阶之后就不再驱动轮子，不等两腿都锁存、也不等阶段跳转。
     * 否则先锁存那一侧的轮子会继续顶着已经被挡住的台阶。
     */
    contact_flag =
        ((Chassis.step_contact_latch_flag[CHASSIS_LEFT] != 0U) ||
         (Chassis.step_contact_latch_flag[CHASSIS_RIGHT] != 0U) ||
         (Chassis.step_posture_flag != 0U)) ? 1U : 0U;
    if ((Chassis.state == CHASSIS_STEP) &&
        ((contact_flag != 0U) ||
         (Chassis.step_phase == CHASSIS_STEP_RECOVER)))
    {
        memset(Chassis.output.I_wheel, 0, sizeof(Chassis.output.I_wheel));
    }
}
