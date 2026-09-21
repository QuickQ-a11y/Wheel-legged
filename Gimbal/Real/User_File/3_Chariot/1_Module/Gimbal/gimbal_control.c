#include "gimbal_control.h"

#include "Angle.h"
#include "device_board.h"
#include "Limit.h"
#include "task_can.h"
#include "task_remote.h"

#include <math.h>
#include <string.h>

Gimbal_t Gimbal;

/* 板间下发分频计数，跨周期状态。 */
static uint8_t boardSendCount;

/**
 * @brief 从电机反馈折算出该轴的关节量。
 *
 * 位置、速度和力矩统一消费 motor_scale*motor_ratio；设备层不维护安装方向。
 * 关节量本身只用于机械限位、YAW 投影角和 Watch 诊断，不进控制环，
 * 但同一个 motor_scale 也定义了力矩下发的方向，见 Gimbal_Command_Send。
 */
static void Joint_Update(Gimbal_Axis_t *axis,
                         const Gimbal_Axis_Config_t *config,
                         motor_dm_index_t index,
                         uint32_t nowTick)
{
    motor_dm_state_t motor;
    float gain = config->motor_scale * config->motor_ratio;

    Motor_DM_GetState(index, &motor);

    axis->angle_joint = (motor.positionRad - config->motor_zero) * gain;
    axis->d_angle_joint = motor.velocityRadps * gain;
    axis->T_fb = motor.torqueNm * gain;
    axis->online = Motor_DM_IsOnline(index, nowTick);
}

/**
 * @brief 串级 PID：角度环给目标角速度，速度环给力矩请求。
 *
 * 角度环把该轴角速度当阻尼项（公共 PID 内部 derivative = -feedbackRate），
 * 所以不需要对角度做数值差分；速度环没有角加速度测量，feedbackRate 传 0。
 * 公共 PID 在 dt <= 0 时不写出参，两个中间量必须先清零。
 */
static void Axis_Control(Gimbal_Axis_t *axis,
                         const Gimbal_Axis_Config_t *config,
                         float dt)
{
    float speed_target = 0.0f;
    float torque = 0.0f;

    Algorithm_PID_UpdateByFeedbackRate(&config->angle_pid,
                                       &axis->angle_pid,
                                       axis->target,
                                       axis->angle,
                                       axis->d_angle,
                                       dt,
                                       &speed_target);
    axis->d_angle_target = speed_target;

    Algorithm_PID_UpdateByFeedbackRate(&config->speed_pid,
                                       &axis->speed_pid,
                                       speed_target,
                                       axis->d_angle,
                                       0.0f,
                                       dt,
                                       &torque);
    axis->T_req = torque;
}

/**
 * @brief 卸力时让目标跟随反馈并清积分。
 *
 * 不做这一步，重新使能的瞬间会拿着陈旧目标和积分踹一脚。
 */
static void Axis_Follow(Gimbal_Axis_t *axis)
{
    axis->target = axis->angle;
    axis->d_angle_target = 0.0f;
    axis->T_req = 0.0f;
    Algorithm_PID_Init(&axis->angle_pid);
    Algorithm_PID_Init(&axis->speed_pid);
}

/**
 * @brief 按杆量积分角度目标，并限住目标相对反馈的超前量。
 *
 * 杆量是角速度语义：满杆一直转，松杆锁在当前角度。YAW 因此可以无限转，
 * 不受杆量行程限制。
 *
 * follow_limit 是必须的：云台被卡住或电机失能时反馈不动，目标会一直积分跑远，
 * 松开后猛地追赶。限住超前量等价于对目标积分做抗饱和。
 */
static void Target_Integrate(Gimbal_Axis_t *axis,
                             const Gimbal_Axis_Config_t *config,
                             float stick,
                             float dt)
{
    axis->target += stick * config->stick_rate * dt;
    axis->target = Algorithm_LimitRange(axis->target,
                                        axis->angle - config->follow_limit,
                                        axis->angle + config->follow_limit);
}

/**
 * @brief 按编码器关节角把角度目标削到机械限位内。
 *
 * IMU 角度会零漂，直接夹 IMU 目标会让限位跟着漂，所以用
 * "编码器关节角 + 当前跟踪误差" 预测该目标最终对应的机械角，超限就把目标削到
 * 预测机械角刚好等于限位。
 *
 * 不需要额外判断"目标是否在往限位方向压"：往里打时预测值自然落回范围内，
 * 根本不会触发削减，所以不存在越界后退不出来的问题。反过来加上那个判断，
 * 反而会在关节已经越界、目标又不够往里时放行"命令关节停在限位外"。
 */
static void Pitch_Joint_Limit(Gimbal_Axis_t *axis,
                              const Gimbal_Axis_Config_t *config)
{
    float predict;

    if (config->joint_min >= config->joint_max)
    {
        return;
    }

    predict = axis->angle_joint + (axis->target - axis->angle);

    if (predict > config->joint_max)
    {
        axis->target = axis->angle + (config->joint_max - axis->angle_joint);
    }
    else if (predict < config->joint_min)
    {
        axis->target = axis->angle + (config->joint_min - axis->angle_joint);
    }
}

void Gimbal_Init(void)
{
    memset(&Gimbal, 0, sizeof(Gimbal));

    Gimbal.state = GIMBAL_ZERO_FORCE;
    Gimbal.fault = GIMBAL_FAULT_NONE;
    Gimbal.dt = Gimbal_Config.default_dt;
    /* 上电默认封锁输出，由每周期的安全门重新判定。 */
    Gimbal.safe_flag = 1U;

    Algorithm_PID_Init(&Gimbal.yaw.angle_pid);
    Algorithm_PID_Init(&Gimbal.yaw.speed_pid);
    Algorithm_PID_Init(&Gimbal.pitch.angle_pid);
    Algorithm_PID_Init(&Gimbal.pitch.speed_pid);

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
    /*
     * DM 必须进入协议使能状态才会持续上报反馈。这里上电就使能并永久保持，
     * 不跟着 safe_flag 走——否则会死锁：要使能才有反馈，要有反馈才能清
     * GIMBAL_FAULT_DM，要清 fault 才能解 safe_flag，要解 safe_flag 才使能。
     * 协议使能不等同于开放动力输出：零力矩由 Motor_DM_SetSafe 和输出安全门
     * 双重保证，命令值本身在 Gimbal_Command_Send 里已经被清零。
     */
    Motor_DM_SetEnable(1U);
}

void Gimbal_Feedback_Update(void)
{
    uint32_t nowTick = HAL_GetTick();
    uint32_t index;

    /* 把遥控任务的发布快照取到全局 Remote，之后本周期的消费都读它。 */
    Remote_Task_Update();
    IMU_Task_GetState(&Gimbal.imu);

    Joint_Update(&Gimbal.yaw, &Gimbal_Config.yaw, MOTOR_DM_YAW, nowTick);
    Joint_Update(&Gimbal.pitch, &Gimbal_Config.pitch, MOTOR_DM_PITCH, nowTick);

    /* YAW 用连续角，目标和反馈在同一个连续坐标里，误差直接相减不必绕环。 */
    Gimbal.yaw.angle = Gimbal.imu.yawTotalRad * Gimbal_Config.yaw.imu_scale;
    Gimbal.pitch.angle = Gimbal.imu.pitchRad * Gimbal_Config.pitch.imu_scale;

    /*
     * IMU 装在云台头部会跟着 PITCH 一起俯仰，陀螺测到的是已经倾斜了的机体角速度，
     * 必须投影回竖直 YAW 轴才是真实的 YAW 角速度。投影角用编码器机械角而不是
     * imu.pitchRad —— 需要的是 IMU 相对 YAW 轴转了多少。PITCH 为零时退化为绕 Z 分量。
     */
    Gimbal.yaw.d_angle =
        ((cosf(Gimbal.pitch.angle_joint) *
          Gimbal.imu.filteredGyroRadps[Gimbal_Config.yaw.gyro_index]) +
         (sinf(Gimbal.pitch.angle_joint) * Gimbal.imu.filteredGyroRadps[0])) *
        Gimbal_Config.yaw.imu_scale;
    Gimbal.pitch.d_angle =
        Gimbal.imu.filteredGyroRadps[Gimbal_Config.pitch.gyro_index] *
        Gimbal_Config.pitch.imu_scale;

    Gimbal.fault = GIMBAL_FAULT_NONE;

    if ((Gimbal.imu.isInitialized == 0U) || (Gimbal.imu.isAttitudeReady == 0U))
    {
        Gimbal.fault |= GIMBAL_FAULT_IMU;
    }
    if ((Gimbal.yaw.online == 0U) || (Gimbal.pitch.online == 0U))
    {
        Gimbal.fault |= GIMBAL_FAULT_DM;
    }
    /*
     * 只有真要驱动发射机构时才把它的在线状态计入故障。发射机构尚未装机时
     * 这三台恒离线，若无条件计入，fault 永远非零、safe_flag 永远为 1，
     * 云台自己也别想输出——一个未装的部件不该拦住已装部件。
     */
    if (Gimbal_Config.output.dji_flag != 0U)
    {
        for (index = 0U; index < APP_DJI_COUNT; index++)
        {
            if (Motor_DJI_IsOnline((motor_dji_index_t)index, nowTick) == 0U)
            {
                Gimbal.fault |= GIMBAL_FAULT_DJI;
            }
        }
    }
    if (CAN_Task_GetTxErrorCount() > APP_CAN_TX_ERROR_MAX)
    {
        Gimbal.fault |= GIMBAL_FAULT_CAN;
    }
    if (Remote.online == 0U)
    {
        Gimbal.fault |= GIMBAL_FAULT_REMOTE;
    }
}

/**
 * @brief 右拨杆直接决定外层状态：下=卸力，中和上都是遥控控制。
 *
 * 左拨杆云台侧只接收不消费，后续由板间通信原样下发给底盘。
 */
void Gimbal_State_Update(void)
{
    Gimbal_State_t next = Gimbal.state;

    if (Remote.rightSwitch == REMOTE_SWITCH_DOWN)
    {
        next = GIMBAL_ZERO_FORCE;
    }
    else if ((Remote.rightSwitch == REMOTE_SWITCH_MID) ||
             (Remote.rightSwitch == REMOTE_SWITCH_UP))
    {
        next = GIMBAL_CONTROL;
    }

    /*
     * 进入控制态时清积分。目标不需要在这里初始化：卸力期间 Axis_Follow 每周期
     * 都把目标压成当前角度，接管瞬间目标已经等于反馈，不会跳变。
     */
    if ((next == GIMBAL_CONTROL) && (Gimbal.state != GIMBAL_CONTROL))
    {
        /* PITCH 有地平面这个绝对基准，使能后自动回水平；YAW 没有基准，停原地。 */
        Gimbal.pitch_home_flag = 1U;
        Algorithm_PID_Init(&Gimbal.yaw.angle_pid);
        Algorithm_PID_Init(&Gimbal.yaw.speed_pid);
        Algorithm_PID_Init(&Gimbal.pitch.angle_pid);
        Algorithm_PID_Init(&Gimbal.pitch.speed_pid);
    }

    Gimbal.state = next;
}

void Gimbal_Control(void)
{
    if (Gimbal.state != GIMBAL_CONTROL)
    {
        Axis_Follow(&Gimbal.yaw);
        Axis_Follow(&Gimbal.pitch);
        Gimbal.pitch_home_flag = 0U;
        memset(Gimbal.I_dji_req, 0, sizeof(Gimbal.I_dji_req));
        return;
    }

    /* 右摇杆左右给 YAW、上下给 PITCH；杆量是角速度，积分成角度目标。 */
    Target_Integrate(&Gimbal.yaw, &Gimbal_Config.yaw,
                     Remote.rightStick.x, Gimbal.dt);
    if (Gimbal.pitch_home_flag != 0U)
    {
        /*
         * 回水平期间不吃摇杆，按速率把目标推向 0（IMU 口径的水平）。
         * 用限幅步长而不是直接赋 0：一步到位会让角度环立刻吐出满限幅的角速度目标。
         * Algorithm_LimitSymmetric 在剩余量小于步长时返回剩余量本身，所以最后一步
         * 精确落到 0，等零判据可靠。到位即交回摇杆，剩下的收敛由控制环完成。
         */
        Gimbal.pitch.target +=
            Algorithm_LimitSymmetric(-Gimbal.pitch.target,
                                     Gimbal_Config.pitch_home_rate * Gimbal.dt);
        if (Gimbal.pitch.target == 0.0f)
        {
            Gimbal.pitch_home_flag = 0U;
        }
    }
    else
    {
        Target_Integrate(&Gimbal.pitch, &Gimbal_Config.pitch,
                         Remote.rightStick.y, Gimbal.dt);
    }

    /* YAW 无限位；PITCH 先夹角度范围，再按关节角做机械限位（安全项放最后）。 */
    Gimbal.pitch.target = Algorithm_LimitRange(Gimbal.pitch.target,
                                               Gimbal_Config.pitch.angle_min,
                                               Gimbal_Config.pitch.angle_max);
    Pitch_Joint_Limit(&Gimbal.pitch, &Gimbal_Config.pitch);

    Axis_Control(&Gimbal.yaw, &Gimbal_Config.yaw, Gimbal.dt);
    Axis_Control(&Gimbal.pitch, &Gimbal_Config.pitch, Gimbal.dt);

    memset(Gimbal.I_dji_req, 0, sizeof(Gimbal.I_dji_req));
}

void Gimbal_Command_Send(void)
{
    motor_dm_command_t command;

    Gimbal.safe_flag = 1U;
    if ((APP_GIMBAL_OUTPUT_ENABLE != 0U) &&
        (Gimbal.fault == GIMBAL_FAULT_NONE) &&
        (Gimbal.state == GIMBAL_CONTROL))
    {
        Gimbal.safe_flag = 0U;
    }

    if ((Gimbal.safe_flag == 0U) && (Gimbal_Config.output.dm_flag != 0U))
    {
        Gimbal.yaw.T = Algorithm_LimitSymmetric(Gimbal.yaw.T_req,
                                                Gimbal_Config.yaw.T_limit);
        Gimbal.pitch.T = Algorithm_LimitSymmetric(Gimbal.pitch.T_req,
                                                  Gimbal_Config.pitch.T_limit);
    }
    else
    {
        Gimbal.yaw.T = 0.0f;
        Gimbal.pitch.T = 0.0f;
    }

    if ((Gimbal.safe_flag == 0U) && (Gimbal_Config.output.dji_flag != 0U))
    {
        memcpy(Gimbal.I_dji, Gimbal.I_dji_req, sizeof(Gimbal.I_dji));
    }
    else
    {
        memset(Gimbal.I_dji, 0, sizeof(Gimbal.I_dji));
    }

    /*
     * 力矩按本轴正方向折回电机正方向，口径与 Joint_Update 的位置速度完全同源，
     * 和底盘 chassis_control.c 的 joint_scale 是同一条规则。
     *
     * 力矩方向不是独立自由度：电机正力矩必然让编码器读数正向增大，而 motor_scale
     * 已经定义了"编码器正向 -> 轴正向"，所以力矩方向被它唯一确定。这里原先另设了
     * 一个 T_scale，既多余又给了填成互相矛盾的机会。
     * 含 ratio 是功率守恒的结果：w_joint = w_motor*scale*ratio 推出
     * T_motor = T_joint*scale*ratio；当前直驱 ratio = 1，只剩符号起作用。
     */
    command.torqueNm = Gimbal.yaw.T *
                       (Gimbal_Config.yaw.motor_scale * Gimbal_Config.yaw.motor_ratio);
    Motor_DM_SetCommand(MOTOR_DM_YAW, &command);
    command.torqueNm = Gimbal.pitch.T *
                       (Gimbal_Config.pitch.motor_scale * Gimbal_Config.pitch.motor_ratio);
    Motor_DM_SetCommand(MOTOR_DM_PITCH, &command);

    Motor_DM_SetSafe(Gimbal.safe_flag);
    Motor_DM_UpdateTxFrames();

    /*
     * 发射机构未装机时不要往 FDCAN2 发帧：总线上没有任何节点应答 ACK，
     * 发送错误计数器会一路爬到 BusOff，驱动随即 Stop/Start 恢复再 BusOff，
     * 空转在错误恢复里。装机后把 output.dji_flag 打开即可。
     */
    if (Gimbal_Config.output.dji_flag != 0U)
    {
        CAN_Task_SetDjiCurrent(Gimbal.I_dji);
    }

    /*
     * 板间下发分频到 200 Hz：DR16 本身只有约 72 Hz 更新率，1 kHz 转发纯属浪费总线。
     * yaw_rel 用编码器口径的机体系关节角，底盘要的是"云台相对车体转了多少"，
     * 不是 IMU 的世界系朝向。
     */
    if (Gimbal_Config.output.board_flag != 0U)
    {
        boardSendCount++;
        if (boardSendCount >= APP_BOARD_SEND_DIV)
        {
            boardSendCount = 0U;
            Board_UpdateTxFrames(&Remote,
                                 Algorithm_AngleNormalizeRad(Gimbal.yaw.angle_joint));
        }
    }

    CAN_Task_RequestTx();
}
