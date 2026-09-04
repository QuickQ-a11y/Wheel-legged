/**
 * @file chassis_mpc.cpp
 * @brief 车身高度与Roll的MPC实现。Eigen 和 TinyMPC 只出现在本文件。
 *
 * 复刻 Qi-Q26（山东理工）的轻量化MPC，求解器换成 TinyMPC：
 * 他们用 condensed QP（整个时域压成一个U，Hessian随N平方增长）跑在香橙派上，
 * TinyMPC 保留状态做Riccati递推，复杂度随N线性，才装得进MCU。
 */

#include "chassis_mpc.h"

#include "chassis_config.h"

#include <tinympc/tiny_api.hpp>

#include "stm32h7xx.h"   /* DWT 周期计数器 */

Chassis_MPC_t Chassis_MPC;

static TinySolver *mpcSolver;
static tinyMatrix mpcUmin;   /* nu x (N-1)，第0列每拍按变化率约束刷新 */
static tinyMatrix mpcUmax;
static tinyMatrix mpcXmin;
static tinyMatrix mpcXmax;
static tinyMatrix mpcXref;
static tinyMatrix mpcUref;

/** @brief 使能DWT周期计数器，用来量单次求解耗时。 */
static void Mpc_Cycle_Counter_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Chassis_MPC_Init(void)
{
    const Chassis_MPC_Config_t *cfg = &Chassis_Config.mpc;
    const int nx = (int)CHASSIS_STATE_MPC_COUNT;
    const int nu = (int)CHASSIS_MPC_INPUT_COUNT;
    const int N  = (int)cfg->horizon;

    const tinytype M  = Chassis_Config.model.body_mass;
    const tinytype g  = Chassis_Config.model.gravity;
    const tinytype Rh = Chassis_Config.wheel.half_track;
    const tinytype Ts = (tinytype)Chassis_Config.mpc.decimation * APP_CTRL_DT_S;
    const tinytype F_eq = 0.5f * M * g;

    /*
     * 连续模型雅可比线性化后再前向欧拉离散。
     *   ddroll = roll_sign * (F_l - F_r) * half_track / I_roll
     *   ddH    = (F_l + F_r)/M - damping/M * dH - g
     * 重力是常量项，进 fdyn，这样 u 可以保持绝对力、力限幅不用平移。
     */
    tinyMatrix Ad(nx, nx);
    Ad << 1, Ts, 0,  0,
          0,  1, 0,  0,
          0,  0, 1, Ts,
          0,  0, 0, 1.0f - (cfg->damping / M) * Ts;

    const tinytype b_roll = cfg->roll_sign * Rh / cfg->I_roll * Ts;
    const tinytype b_h    = Ts / M;
    tinyMatrix Bd(nx, nu);
    Bd << 0,       0,
          b_roll, -b_roll,
          0,       0,
          b_h,     b_h;

    tinyVector fdyn(nx);
    fdyn << 0, 0, 0, -g * Ts;

    tinyVector Q(nx);
    tinyVector Rw(nu);
    for (int i = 0; i < nx; i++) { Q(i)  = cfg->Q[i]; }
    for (int j = 0; j < nu; j++) { Rw(j) = cfg->R[j]; }

    if (tiny_setup(&mpcSolver, Ad, Bd, fdyn, Q.asDiagonal(), Rw.asDiagonal(),
                   cfg->rho, nx, nu, N, /*verbose=*/0) != 0)
    {
        Chassis_MPC.ready_flag = 0U;
        return;
    }

    tiny_set_default_settings(mpcSolver->settings);
    mpcSolver->settings->max_iter = (int)cfg->max_iter;
    /*
     * 只留输入箱式约束。锥、线性、时变线性一律关闭：那些分支里的
     * project_soc/project_hyperplane 是按值返回的，会在控制环里分配临时量。
     * 状态不设限也省掉一半投影，本模型的状态本来就没有物理上限。
     */
    mpcSolver->settings->en_input_bound = 1;
    mpcSolver->settings->en_state_bound = 0;
    mpcSolver->settings->en_input_soc = 0;
    mpcSolver->settings->en_state_soc = 0;
    mpcSolver->settings->en_input_linear = 0;
    mpcSolver->settings->en_state_linear = 0;
    mpcSolver->settings->en_tv_input_linear = 0;
    mpcSolver->settings->en_tv_state_linear = 0;

    /* 工作矩阵一次分配好，Solve 里只改值不改尺寸，避免任何重分配。 */
    mpcXmin = tinyMatrix::Constant(nx, N, -1.0e6f);
    mpcXmax = tinyMatrix::Constant(nx, N,  1.0e6f);
    mpcUmin = tinyMatrix::Constant(nu, N - 1, cfg->F_min);
    mpcUmax = tinyMatrix::Constant(nu, N - 1, cfg->F_max);
    mpcXref = tinyMatrix::Zero(nx, N);
    mpcUref = tinyMatrix::Constant(nu, N - 1, F_eq);

    Chassis_MPC.F[0] = F_eq;
    Chassis_MPC.F[1] = F_eq;
    Chassis_MPC.cycles_max = 0U;

    Mpc_Cycle_Counter_Enable();
    Chassis_MPC.ready_flag = 1U;
}

void Chassis_MPC_Solve(const float x0[4], float H_ref)
{
    const Chassis_MPC_Config_t *cfg = &Chassis_Config.mpc;
    const int nu = (int)CHASSIS_MPC_INPUT_COUNT;
    uint32_t t0;
    uint32_t t1;

    if (Chassis_MPC.ready_flag == 0U)
    {
        return;
    }

    for (int i = 0; i < (int)CHASSIS_STATE_MPC_COUNT; i++)
    {
        Chassis_MPC.x[i] = x0[i];
    }
    Chassis_MPC.H_ref = H_ref;

    tinyVector x0v(CHASSIS_STATE_MPC_COUNT);
    x0v << x0[0], x0[1], x0[2], x0[3];
    tiny_set_x0(mpcSolver, x0v);

    /* 参考轨迹：roll和两个速度都要0，高度跟H_ref。 */
    mpcXref.setZero();
    mpcXref.row(2).setConstant(H_ref);
    tiny_set_x_ref(mpcSolver, mpcXref);
    tiny_set_u_ref(mpcSolver, mpcUref);

    /*
     * 只对第0步叠加变化率约束。滚动优化只下发第0步，而上一拍的u是已知常量，
     * 所以这一条是纯逐时刻约束，装得进TinyMPC，且MPC在优化时就知道自己被限了
     * ——比事后砍一刀的外部限速强。后面14步只受力限幅，反正不下发。
     */
    for (int j = 0; j < nu; j++)
    {
        const tinytype lo = Chassis_MPC.F[j] - cfg->dF_max;
        const tinytype hi = Chassis_MPC.F[j] + cfg->dF_max;

        mpcUmin(j, 0) = (lo > cfg->F_min) ? lo : cfg->F_min;
        mpcUmax(j, 0) = (hi < cfg->F_max) ? hi : cfg->F_max;
    }
    tiny_set_bound_constraints(mpcSolver, mpcXmin, mpcXmax, mpcUmin, mpcUmax);

    t0 = DWT->CYCCNT;
    (void)tiny_solve(mpcSolver);
    t1 = DWT->CYCCNT;

    Chassis_MPC.cycles = t1 - t0;
    if (Chassis_MPC.cycles > Chassis_MPC.cycles_max)
    {
        Chassis_MPC.cycles_max = Chassis_MPC.cycles;
    }
    Chassis_MPC.iter = (uint32_t)mpcSolver->solution->iter;
    Chassis_MPC.solved = (uint32_t)mpcSolver->solution->solved;
    Chassis_MPC.F[0] = mpcSolver->solution->u(0, 0);
    Chassis_MPC.F[1] = mpcSolver->solution->u(1, 0);
}
