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
/*
 * 控制环发布、MPC任务消费的求解输入。裸全局不加锁：都是32位对齐的float，
 * 单字读写在Cortex-M上是原子的，最坏只会读到跨一拍的一组值，物理量1ms内的
 * 变化远小于测量噪声。
 */
static float mpcInputX[CHASSIS_STATE_MPC_COUNT];
static float mpcInputHref;

/**
 * @brief 使能DWT周期计数器，用来量单次求解耗时。
 *
 * LAR 那一行不能省：Cortex-M7 的 DWT 带软件锁（CMSIS 的 DWT_Type 里有
 * LAR/LSR 就是证据），锁着的时候对 CTRL 和 CYCCNT 的写入会被【静默忽略】
 * ——不报错、不置位，CYCCNT 恒为 0，于是 cycles = t1 - t0 恒为 0，
 * Chassis_MPC.cycles_max 永远停在 0。M4 没有这把锁，从 F4 工程搬代码过来
 * 最容易漏的就是它。0xC5ACCE55 是 ARM 规定的解锁魔数。
 * 顺序也有讲究：先开 TRCENA 总开关，再解锁，最后才配置计数器。
 */
static void Mpc_Cycle_Counter_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
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
    /*
     * 单腿平衡支撑力。乘 F0_gravity_scale 是为了把【未建模的恒定力】算进来：
     * 静力学说髋部只需 0.5*M*g，但实测稳态命令比它高一截——髋部实际分担了一部分
     * 腿自重（模型把腿轮自重全算在地面上），再加五连杆轴承和同步带的静摩擦。
     * PID 路径本来就用同一个系数补这一份，MPC 以前不读它，两条路径口径不一致。
     */
    const tinytype F_eq = 0.5f * M * g * Chassis_Config.F0_gravity_scale;

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
    /*
     * ⚠ 常量项必须与 F_eq 同源，写成 -(2*F_eq/M)*Ts 而不是 -g*Ts。
     * 否则代价函数的 Uref 停在 F_eq、而模型的平衡点停在 0.5*M*g，两者互相拉扯，
     * 结果就是【又一个稳态误差】，只是方向相反。同源之后平衡点严格落在 F = F_eq，
     * 标称无稳态误差。输入增益 Bd 仍用真实质量 M，那是动力学，不该跟着缩放。
     */
    fdyn << 0, 0, 0, -(2.0f * F_eq / M) * Ts;

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

    /*
     * 约束和参考量只在这里灌一次。这几个 tiny_set_* 都是【值传递】动态Eigen矩阵
     * （见 tiny_api.hpp），一次调用要堆分配+拷贝两遍，放在求解环里纯属浪费：
     * 状态限幅和输入参考自始至终不变，腿角参考只有第2行随 H_ref 变，
     * 输入限幅只有第0列随变化率约束变。Solve() 里改成直写 mpcSolver->work，
     * 省掉每拍 7 次矩阵拷贝。work 是 TinyMPC 的公开结构体，不算改它的代码。
     */
    tiny_set_bound_constraints(mpcSolver,
                               tinyMatrix::Constant(nx, N, -1.0e6f),
                               tinyMatrix::Constant(nx, N,  1.0e6f),
                               tinyMatrix::Constant(nu, N - 1, cfg->F_min),
                               tinyMatrix::Constant(nu, N - 1, cfg->F_max));
    tiny_set_x_ref(mpcSolver, tinyMatrix::Zero(nx, N));
    tiny_set_u_ref(mpcSolver, tinyMatrix::Constant(nu, N - 1, F_eq));

    Chassis_MPC.F[0] = F_eq;
    Chassis_MPC.F[1] = F_eq;
    Chassis_MPC.cycles_max = 0U;
    Chassis_MPC.solve_us_max = 0U;
    Chassis_MPC.solve_count = 0U;
    Chassis_MPC.age = 0U;
    /* 控制环还没发布过输入时，先喂一组静止平衡姿态，别让首次求解吃全零。 */
    mpcInputX[0] = 0.0f;
    mpcInputX[1] = 0.0f;
    mpcInputX[2] = Chassis_Config.leg[CHASSIS_LEFT].target_L0;
    mpcInputX[3] = 0.0f;
    mpcInputHref = Chassis_Config.leg[CHASSIS_LEFT].target_L0;

    Mpc_Cycle_Counter_Enable();
    Chassis_MPC.ready_flag = 1U;
}

void Chassis_MPC_SetInput(const float x0[4], float H_ref)
{
    for (int i = 0; i < (int)CHASSIS_STATE_MPC_COUNT; i++)
    {
        mpcInputX[i] = x0[i];
    }
    mpcInputHref = H_ref;
}

void Chassis_MPC_Solve(void)
{
    const Chassis_MPC_Config_t *cfg = &Chassis_Config.mpc;
    const int nu = (int)CHASSIS_MPC_INPUT_COUNT;
    float x0[CHASSIS_STATE_MPC_COUNT];
    float H_ref;
    uint32_t t0;
    uint32_t t1;

    if (Chassis_MPC.ready_flag == 0U)
    {
        return;
    }

    /* 先把本次要用的输入取到局部，后面整个求解期间不再看全局，免得中途被改。 */
    for (int i = 0; i < (int)CHASSIS_STATE_MPC_COUNT; i++)
    {
        x0[i] = mpcInputX[i];
        Chassis_MPC.x[i] = x0[i];
    }
    H_ref = mpcInputHref;
    Chassis_MPC.H_ref = H_ref;

    /*
     * 直写 workspace，不走 tiny_set_* ——那几个是值传递动态矩阵，每调一次就
     * 堆分配加拷贝。这里改的都是原地赋值，零分配。尺寸在 Init 时已定，
     * 不会变，所以 setter 里那些 check_dimension 也没有意义。
     */
    mpcSolver->work->x(0, 0) = x0[0];
    mpcSolver->work->x(1, 0) = x0[1];
    mpcSolver->work->x(2, 0) = x0[2];
    mpcSolver->work->x(3, 0) = x0[3];

    /* 参考轨迹只有高度这一行随目标变，其余行在 Init 里已经是0且不再改动。 */
    mpcSolver->work->Xref.row(2).setConstant(H_ref);

    /*
     * 只对第0步叠加变化率约束。滚动优化只下发第0步，而上一拍的u是已知常量，
     * 所以这一条是纯逐时刻约束，装得进TinyMPC，且MPC在优化时就知道自己被限了
     * ——比事后砍一刀的外部限速强。后面14步只受力限幅，反正不下发。
     */
    for (int j = 0; j < nu; j++)
    {
        const tinytype lo = Chassis_MPC.F[j] - cfg->dF_max;
        const tinytype hi = Chassis_MPC.F[j] + cfg->dF_max;

        mpcSolver->work->u_min(j, 0) = (lo > cfg->F_min) ? lo : cfg->F_min;
        mpcSolver->work->u_max(j, 0) = (hi < cfg->F_max) ? hi : cfg->F_max;
    }

    t0 = DWT->CYCCNT;
    (void)tiny_solve(mpcSolver);
    t1 = DWT->CYCCNT;

    Chassis_MPC.cycles = t1 - t0;
    if (Chassis_MPC.cycles > Chassis_MPC.cycles_max)
    {
        Chassis_MPC.cycles_max = Chassis_MPC.cycles;
    }
    /* 用运行时主频换算，免得把 480MHz 写死（历史交接文档写成 550 过）。 */
    Chassis_MPC.solve_us = Chassis_MPC.cycles / (SystemCoreClock / 1000000U);
    if (Chassis_MPC.solve_us > Chassis_MPC.solve_us_max)
    {
        Chassis_MPC.solve_us_max = Chassis_MPC.solve_us;
    }
    Chassis_MPC.iter = (uint32_t)mpcSolver->solution->iter;
    Chassis_MPC.solved = (uint32_t)mpcSolver->solution->solved;
    Chassis_MPC.F[0] = mpcSolver->solution->u(0, 0);
    Chassis_MPC.F[1] = mpcSolver->solution->u(1, 0);
    Chassis_MPC.solve_count++;
    Chassis_MPC.age = 0U;
}
