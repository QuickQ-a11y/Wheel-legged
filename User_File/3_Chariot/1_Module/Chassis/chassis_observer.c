#include "chassis_control.h"

#include "Limit.h"

#include <math.h>
#include <string.h>

#define CHASSIS_OBS_EPS 1.0e-6f

/**
 * @brief 一阶低通，时间常数或周期非正时直接透传当前值。
 */
static float Observer_Filter(float value,
                             float previous,
                             float time_constant_s,
                             float dt)
{
    float ratio;

    if ((time_constant_s <= 0.0f) || (dt <= 0.0f))
    {
        return value;
    }
    ratio = dt / (time_constant_s + dt);

    return previous + ratio * (value - previous);
}

/**
 * @brief 单腿静载，力类观测阈值统一以它为基准按比例定义。
 */
static float Observer_Static_Load(const Chassis_Model_Config_t *config)
{
    return 0.5f * Chassis_Model_Mass(config) * config->gravity;
}

/**
 * @brief 当前处于倒地或台阶动作，离地与卡腿判定在这些状态下不成立。
 */
static uint8_t Observer_In_Action(const Chassis_t *chassis)
{
    return ((chassis->state == CHASSIS_FALLEN) ||
            (chassis->state == CHASSIS_FALLING_TO_STAND) ||
            (chassis->state == CHASSIS_STEP)) ? 1U : 0U;
}

void Chassis_Slip_Init(Chassis_Slip_t *slip)
{
    memset(slip, 0, sizeof(*slip));
}

/**
 * @brief 由轮速反推偏航并给出单侧速度残差，末尾保存本轮差分基准。
 *
 * 首轮没有上一周期速度，两路增量残差直接给零，从第二轮开始才有意义。
 */
void Chassis_Slip_Update(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *slip_config = &config->observer;
    Chassis_Slip_t *slip = &chassis->slip;
    float track_m = 2.0f * config->wheel.half_track;
    uint32_t side;

    if (track_m > CHASSIS_OBS_EPS)
    {
        slip->d_fai_wheel =
            (chassis->body.side_speed[CHASSIS_RIGHT] -
             chassis->body.side_speed[CHASSIS_LEFT]) / track_m;
    }
    else
    {
        slip->d_fai_wheel = 0.0f;
    }
    slip->yaw_res = slip->d_fai_wheel - chassis->body.d_fai;
    slip->yaw_res_lpf = Observer_Filter(slip->yaw_res,
                                        slip->yaw_res_lpf,
                                        slip_config->residual_filter_s,
                                        chassis->dt);
    slip->dv_acc = fabsf(chassis->body.dd_s * chassis->dt);

    slip->v_expect[CHASSIS_LEFT] =
        chassis->body.d_s - config->wheel.half_track * chassis->body.d_fai;
    slip->v_expect[CHASSIS_RIGHT] =
        chassis->body.d_s + config->wheel.half_track * chassis->body.d_fai;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float wheel_delta;

        /* 轮缘线速度只含轮子自转，不含腿摆和平动，用于轮打滑判定。 */
        slip->v_wheel[side] =
            chassis->body.wheel_speed[side] * config->wheel.R;
        slip->v_res[side] =
            chassis->body.side_speed[side] - slip->v_expect[side];
        slip->v_res_lpf[side] = Observer_Filter(slip->v_res[side],
                                                slip->v_res_lpf[side],
                                                slip_config->residual_filter_s,
                                                chassis->dt);
        if (slip->init_flag == 0U)
        {
            slip->dv_res[side] = 0.0f;
            wheel_delta = 0.0f;
        }
        else
        {
            slip->dv_res[side] =
                (chassis->body.side_speed[side] - slip->last_v[side]) -
                chassis->body.dd_s * chassis->dt;
            wheel_delta =
                fabsf(slip->v_wheel[side] - slip->last_v_wheel[side]);
        }
        slip->dv_wheel[side] = Observer_Filter(wheel_delta,
                                               slip->dv_wheel[side],
                                               slip_config->residual_filter_s,
                                               chassis->dt);

        slip->last_v[side] = chassis->body.side_speed[side];
        slip->last_v_wheel[side] = slip->v_wheel[side];
    }
    slip->init_flag = 1U;
}

/**
 * @brief 闸门成立后由残差判定打滑，退出由起始轮速锁存决定。
 *
 * 闸门要求偏航残差或任一侧轮速与整车速度差超阈值，且当前不是小陀螺；
 * 小陀螺本身就靠左右轮速差旋转，不屏蔽会持续误判。
 */
void Chassis_Slip_Calc(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *slip_config = &config->observer;
    Chassis_Slip_t *slip = &chassis->slip;
    uint32_t side;

    slip->gate_flag =
        (((fabsf(slip->yaw_res_lpf) >= slip_config->slip_gate_yaw) ||
          (fabsf(slip->v_wheel[CHASSIS_LEFT]) - fabsf(chassis->body.d_s) >=
           slip_config->slip_gate_v) ||
          (fabsf(slip->v_wheel[CHASSIS_RIGHT]) - fabsf(chassis->body.d_s) >=
           slip_config->slip_gate_v)) &&
         (chassis->mode != CHASSIS_MODE_TOP)) ? 1U : 0U;

    if (slip->gate_flag == 0U)
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            slip->candidate_flag[side] = 0U;
            slip->slip_flag[side] = 0U;
            slip->enter_time[side] = 0.0f;
        }
        return;
    }

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        slip->candidate_flag[side] =
            ((fabsf(slip->v_res_lpf[side]) >= slip_config->slip_v_enter) &&
             ((fabsf(slip->yaw_res_lpf) >= slip_config->slip_yaw_enter) ||
              (fabsf(slip->dv_res[side]) >= slip_config->slip_dv_enter) ||
              (slip->dv_wheel[side] - slip->dv_acc >=
               slip_config->slip_dv_enter))) ? 1U : 0U;

        if (slip->slip_flag[side] == 0U)
        {
            if (slip->candidate_flag[side] != 0U)
            {
                slip->enter_time[side] += chassis->dt;
                if (slip->enter_time[side] >= slip_config->slip_enter_s)
                {
                    /* 锁存打滑起始轮速，轮速回落到该值以下才算重新抓地。 */
                    slip->slip_flag[side] = 1U;
                    slip->v_latch[side] = fabsf(slip->v_wheel[side]);
                    slip->enter_time[side] = 0.0f;
                }
            }
            else
            {
                slip->enter_time[side] = 0.0f;
            }
        }
        else
        {
            slip->enter_time[side] = 0.0f;
            if (fabsf(slip->v_wheel[side]) <= slip->v_latch[side])
            {
                slip->slip_flag[side] = 0U;
            }
        }
    }
}

void Chassis_Ground_Init(Chassis_Ground_t *ground)
{
    memset(ground, 0, sizeof(*ground));
}

/**
 * @brief 由关节反馈力矩反解虚拟腿广义力，扣除腿部惯性后得到单腿支撑力。
 *
 * 力矩反解无解或腿长退化时本轮支撑力无效，末尾保存腿速度差分基准。
 */
void Chassis_Ground_Update(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *ground_config = &config->observer;
    Chassis_Ground_t *ground = &chassis->ground;
    uint32_t side;

    ground->Fn_static = Observer_Static_Load(&config->model);
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        const Chassis_Leg_t *leg = &chassis->leg[side];
        float dd_L0_raw = 0.0f;
        float dd_theta_raw = 0.0f;
        uint8_t phi1_index =
            config->leg[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            config->leg[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        /* 首轮用当前反馈建立基准，使二阶导从零起步而不是跳变。 */
        if (ground->init_flag == 0U)
        {
            ground->last_d_L0[side] = leg->d_L0;
            ground->last_d_theta[side] = leg->d_theta;
        }
        if (chassis->dt > CHASSIS_OBS_EPS)
        {
            dd_L0_raw =
                (leg->d_L0 - ground->last_d_L0[side]) / chassis->dt;
            dd_theta_raw =
                (leg->d_theta - ground->last_d_theta[side]) / chassis->dt;
        }
        ground->dd_L0[side] = Observer_Filter(dd_L0_raw,
                                              ground->dd_L0[side],
                                              ground_config->residual_filter_s,
                                              chassis->dt);
        ground->dd_theta[side] = Observer_Filter(dd_theta_raw,
                                                 ground->dd_theta[side],
                                                 ground_config->residual_filter_s,
                                                 chassis->dt);

        ground->valid_flag[side] =
            (VMC_Force_Calc(&config->leg[side],
                            leg,
                            chassis->dm_motor[phi1_index].torque_nm,
                            chassis->dm_motor[phi4_index].torque_nm,
                            &ground->force[side]) != 0U) &&
            (fabsf(leg->L0) > CHASSIS_OBS_EPS);
        if (ground->valid_flag[side] != 0U)
        {
            /* 轮轴竖直加速度由极坐标腿的向心、科氏和角加速度项展开。 */
            float leg_accel =
                chassis->imu.body_accel[config->imu.vertical_accel_axis] -
                ground->dd_L0[side] * cosf(leg->theta) +
                2.0f * leg->d_L0 * leg->d_theta * sinf(leg->theta) +
                leg->L0 * ground->dd_theta[side] * sinf(leg->theta) +
                leg->L0 * leg->d_theta * leg->d_theta * cosf(leg->theta);
            float Fn =
                -(ground->force[side].F0 * cosf(leg->theta) +
                  ground->force[side].Tp * sinf(leg->theta) / leg->L0) +
                config->model.leg_mass *
                    (config->model.gravity + leg_accel);

            ground->Fn_raw[side] = Fn;
            if (ground->Fn_init_flag[side] == 0U)
            {
                ground->Fn[side] = Fn;
                ground->Fn_init_flag[side] = 1U;
            }
            else
            {
                ground->Fn[side] =
                    Observer_Filter(Fn,
                                    ground->Fn[side],
                                    ground_config->normal_force_filter_s,
                                    chassis->dt);
            }
            if (ground->Fn_static > CHASSIS_OBS_EPS)
            {
                ground->Fn_ratio[side] = ground->Fn[side] / ground->Fn_static;
            }
            else
            {
                ground->Fn_ratio[side] = 0.0f;
            }
        }

        ground->last_d_L0[side] = leg->d_L0;
        ground->last_d_theta[side] = leg->d_theta;
    }
    ground->init_flag = 1U;
}

/**
 * @brief 支撑力低于离地比例判定离地，回到落地比例判定落地，带双时间迟滞。
 *
 * 本轮支撑力无效时清空该侧判定；倒地和台阶动作期间腿本来就会脱离地面，
 * 整车离地结论在这些状态下不成立。
 */
void Chassis_Ground_Calc(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *ground_config = &config->observer;
    Chassis_Ground_t *ground = &chassis->ground;
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (ground->valid_flag[side] == 0U)
        {
            ground->off_candidate_flag[side] = 0U;
            ground->off_ground_flag[side] = 0U;
            ground->off_time[side] = 0.0f;
            ground->land_time[side] = 0.0f;
            continue;
        }

        ground->off_candidate_flag[side] =
            (ground->Fn_ratio[side] <= ground_config->off_force_ratio) ? 1U : 0U;
        if (ground->off_ground_flag[side] == 0U)
        {
            ground->land_time[side] = 0.0f;
            if (ground->off_candidate_flag[side] != 0U)
            {
                ground->off_time[side] += chassis->dt;
                if (ground->off_time[side] >= ground_config->off_hold_s)
                {
                    ground->off_ground_flag[side] = 1U;
                    ground->off_time[side] = 0.0f;
                }
            }
            else
            {
                ground->off_time[side] = 0.0f;
            }
        }
        else
        {
            ground->off_time[side] = 0.0f;
            if (ground->Fn_ratio[side] >= ground_config->land_force_ratio)
            {
                ground->land_time[side] += chassis->dt;
                if (ground->land_time[side] >= ground_config->land_hold_s)
                {
                    ground->off_ground_flag[side] = 0U;
                    ground->land_time[side] = 0.0f;
                }
            }
            else
            {
                ground->land_time[side] = 0.0f;
            }
        }
    }

    ground->all_off_flag =
        ((ground->off_ground_flag[CHASSIS_LEFT] != 0U) &&
         (ground->off_ground_flag[CHASSIS_RIGHT] != 0U) &&
         (Observer_In_Action(chassis) == 0U)) ? 1U : 0U;
    ground->fn_comp =
        (ground->all_off_flag != 0U) ?
            (ground->Fn_static * ground_config->off_F_comp_ratio) : 0.0f;
}

void Chassis_Turn_Init(Chassis_Turn_t *turn)
{
    memset(turn, 0, sizeof(*turn));
}

/**
 * @brief 取IMU横向加速度和运动学向心加速度两路，滤波后给出残差。
 *
 * 残差反映侧滑或标定误差；质心高度按当前平均腿长加机体质心偏置估计。
 */
void Chassis_Turn_Update(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *turn_config = &config->observer;
    Chassis_Turn_t *turn = &chassis->turn;

    turn->a_y_imu = chassis->imu.body_accel[config->imu.lateral_accel_axis];
    turn->a_y_kin = chassis->body.d_s * chassis->body.d_fai;
    turn->a_y_imu_lpf = Observer_Filter(turn->a_y_imu,
                                        turn->a_y_imu_lpf,
                                        turn_config->turn_filter_s,
                                        chassis->dt);
    turn->a_y_kin_lpf = Observer_Filter(turn->a_y_kin,
                                        turn->a_y_kin_lpf,
                                        turn_config->turn_filter_s,
                                        chassis->dt);
    turn->a_y_res = turn->a_y_imu_lpf - turn->a_y_kin_lpf;
    turn->h_cg = 0.5f * (chassis->leg[CHASSIS_LEFT].L0 +
                         chassis->leg[CHASSIS_RIGHT].L0) +
                 config->model.cg_to_hip;
}

/**
 * @brief 由左右轮速差反推转弯半径和前向加速度离心修正，再估算支撑力差。
 *
 * 转弯半径取二轮差速模型；任一侧打滑时轮速不再代表车体运动，直接给零。
 * 离心修正量本轮只作为观测值输出，不参与速度Kalman。
 */
void Chassis_Turn_Calc(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *turn_config = &config->observer;
    Chassis_Turn_t *turn = &chassis->turn;
    const Chassis_Slip_t *slip = &chassis->slip;
    float total_mass_kg = Chassis_Model_Mass(&config->model);
    float force_limit = Observer_Static_Load(&config->model) *
                        turn_config->turn_force_limit_ratio;
    float track_m = 2.0f * config->wheel.half_track;
    float v_sum = slip->v_wheel[CHASSIS_LEFT] + slip->v_wheel[CHASSIS_RIGHT];
    float v_diff = slip->v_wheel[CHASSIS_LEFT] - slip->v_wheel[CHASSIS_RIGHT];

    if ((slip->slip_flag[CHASSIS_LEFT] == 0U) &&
        (slip->slip_flag[CHASSIS_RIGHT] == 0U) &&
        (fabsf(v_diff) >= turn_config->turn_v_diff) &&
        (fabsf(v_diff) > CHASSIS_OBS_EPS))
    {
        turn->R_turn = fabsf(track_m * v_sum / (2.0f * v_diff));
    }
    else
    {
        turn->R_turn = 0.0f;
    }
    turn->dd_s_turn =
        chassis->body.d_fai * chassis->body.d_fai * turn->R_turn;
    turn->dd_s_fix = chassis->body.dd_s + turn->dd_s_turn;

    if (fabsf(track_m) > CHASSIS_OBS_EPS)
    {
        turn->dF_imu =
            total_mass_kg * turn->h_cg * turn->a_y_imu_lpf / track_m;
        turn->dF_kin =
            total_mass_kg * turn->h_cg * turn->a_y_kin_lpf / track_m;
    }
    else
    {
        turn->dF_imu = 0.0f;
        turn->dF_kin = 0.0f;
    }
    turn->dF_imu_lim = Algorithm_LimitSymmetric(turn->dF_imu, force_limit);
    turn->dF_kin_lim = Algorithm_LimitSymmetric(turn->dF_kin, force_limit);
}

void Chassis_Stuck_Init(Chassis_Stuck_t *stuck)
{
    memset(stuck, 0, sizeof(*stuck));
}

/**
 * @brief 取本轮轮力矩请求和腿摆角绝对值作为卡腿判定输入。
 */
void Chassis_Stuck_Update(const Chassis_Config_t *config, Chassis_t *chassis)
{
    Chassis_Stuck_t *stuck = &chassis->stuck;
    uint32_t side;

    (void)config;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        stuck->T_wheel[side] = fabsf(chassis->output.T_wheel[side]);
        stuck->theta[side] = fabsf(chassis->leg[side].theta);
    }
}

/**
 * @brief 轮力矩和腿摆角同时超阈值并持续后判定卡腿，给出补偿建议值。
 *
 * 补偿量随卡腿时间线性增长并封顶，本轮只作为Watch观测值，不接入F0和目标腿长。
 * 倒地、台阶和整车离地期间腿摆角本来就大，这些状态下不参与判定。
 */
void Chassis_Stuck_Calc(const Chassis_Config_t *config, Chassis_t *chassis)
{
    const Chassis_Observer_Config_t *stuck_config = &config->observer;
    Chassis_Stuck_t *stuck = &chassis->stuck;
    /* 力和力矩阈值按比例定义，换机器人时随model和wheel自动缩放。 */
    float static_load = Observer_Static_Load(&config->model);
    float torque_enter = config->wheel.T_limit * stuck_config->stuck_T_ratio;
    uint8_t blocked_flag =
        ((Observer_In_Action(chassis) != 0U) ||
         (chassis->ground.all_off_flag != 0U)) ? 1U : 0U;
    uint32_t side;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        if (blocked_flag != 0U)
        {
            stuck->stuck_time[side] = 0.0f;
        }
        else if ((stuck->T_wheel[side] >= torque_enter) &&
                 (stuck->theta[side] >= stuck_config->stuck_theta_enter))
        {
            stuck->stuck_time[side] += chassis->dt;
        }
        else if (stuck->theta[side] <= stuck_config->stuck_theta_exit)
        {
            stuck->stuck_time[side] = 0.0f;
        }

        if (stuck->stuck_time[side] >= stuck_config->stuck_time)
        {
            stuck->stuck_flag[side] = 1U;
            stuck->comp_F0[side] = Algorithm_LimitRange(
                stuck->stuck_time[side] * stuck_config->stuck_F0_coef_ratio *
                    static_load,
                0.0f,
                stuck_config->stuck_F0_max_ratio * static_load);
            stuck->comp_L0[side] = Algorithm_LimitRange(
                stuck->stuck_time[side] * stuck_config->stuck_L0_coef,
                0.0f,
                stuck_config->stuck_L0_max);
        }
        else
        {
            stuck->stuck_flag[side] = 0U;
            stuck->comp_F0[side] = 0.0f;
            stuck->comp_L0[side] = 0.0f;
        }
    }
}
