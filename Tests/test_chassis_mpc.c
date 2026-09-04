/*
 * 车身高度与Roll的MPC。把PC端验证过的场景固化成断言，防止改模型、改权重或
 * 升级TinyMPC时悄悄退化。
 *
 * 求解器本身是C++，本文件仍是C——正好也验证了 extern "C" 壳是通的。
 */
#include "chassis_mpc.h"
#include "chassis_config.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TOL 0.05f

static void solve_until_settled(float a, float da, float H, float dH, float Href)
{
    uint32_t k;

    /* 变化率约束每拍只放行 dF_max，要跑够拍数才能到稳态。 */
    for (k = 0U; k < 200U; k++)
    {
        float x0[4];

        x0[0] = a; x0[1] = da; x0[2] = H; x0[3] = dH;
        Chassis_MPC_Solve(x0, Href);
    }
}

/* 静止站立、高度已到位时，两腿应各承担一半整车重量。 */
static void test_mpc_steady_state(void)
{
    const float F_eq = 0.5f * Chassis_Config.model.body_mass *
                       Chassis_Config.model.gravity;

    solve_until_settled(0.0f, 0.0f, 0.20f, 0.0f, 0.20f);
    assert(fabsf(Chassis_MPC.F[0] - F_eq) < TOL);
    assert(fabsf(Chassis_MPC.F[1] - F_eq) < TOL);
}

/*
 * Roll 偏差要靠左右【差动】纠正。整车右手系里+roll是左侧上抬，
 * 所以左腿力必须【减小】让左侧落下——照抄 Qi-Q26 的符号会反过来。
 */
static void test_mpc_roll_differential(void)
{
    float F_left_pos;
    float F_right_pos;

    solve_until_settled(0.10f, 0.0f, 0.20f, 0.0f, 0.20f);
    F_left_pos = Chassis_MPC.F[0];
    F_right_pos = Chassis_MPC.F[1];
    assert(F_left_pos < F_right_pos);

    /* 镜像姿态必须镜像响应。 */
    solve_until_settled(-0.10f, 0.0f, 0.20f, 0.0f, 0.20f);
    assert(Chassis_MPC.F[0] > Chassis_MPC.F[1]);
    assert(fabsf((Chassis_MPC.F[0] - F_right_pos)) < TOL);
    assert(fabsf((Chassis_MPC.F[1] - F_left_pos)) < TOL);
}

/* 高度偏差是【共模】：车身偏矮时两腿要同向增大支撑力。 */
static void test_mpc_height_common_mode(void)
{
    float F_low_left;

    solve_until_settled(0.0f, 0.0f, 0.15f, 0.0f, 0.20f);
    F_low_left = Chassis_MPC.F[0];
    assert(fabsf(Chassis_MPC.F[0] - Chassis_MPC.F[1]) < TOL);   /* 同向等量 */

    solve_until_settled(0.0f, 0.0f, 0.20f, 0.0f, 0.20f);
    assert(F_low_left > Chassis_MPC.F[0]);                      /* 偏矮时更大 */
}

/* 力限幅是硬约束，任何状态下都不许越界。 */
static void test_mpc_force_bounds(void)
{
    const Chassis_MPC_Config_t *mpc = &Chassis_Config.mpc;
    uint32_t k;

    for (k = 0U; k < 400U; k++)
    {
        float x0[4];
        float sweep = (float)k * 0.01f - 2.0f;

        x0[0] = sweep; x0[1] = -sweep; x0[2] = 0.05f + sweep; x0[3] = sweep;
        Chassis_MPC_Solve(x0, 0.30f);
        assert(Chassis_MPC.F[0] >= mpc->F_min - TOL);
        assert(Chassis_MPC.F[0] <= mpc->F_max + TOL);
        assert(Chassis_MPC.F[1] >= mpc->F_min - TOL);
        assert(Chassis_MPC.F[1] <= mpc->F_max + TOL);
    }
}

/*
 * 变化率约束。TinyMPC 的约束都是逐时刻的，跨时刻耦合装不进去；靠的是
 * "滚动优化只下发第0步、上一拍的u又是已知常量"，把它变成纯逐时刻约束。
 */
static void test_mpc_rate_limit(void)
{
    const float dF = Chassis_Config.mpc.dF_max;
    float prev[2];
    uint32_t k;

    /* 先稳在平衡点，再突然要求一个够不着的高度，逼它顶到变化率上限。 */
    solve_until_settled(0.0f, 0.0f, 0.20f, 0.0f, 0.20f);
    prev[0] = Chassis_MPC.F[0];
    prev[1] = Chassis_MPC.F[1];

    for (k = 0U; k < 20U; k++)
    {
        float x0[4] = {0.0f, 0.0f, 0.05f, 0.0f};

        Chassis_MPC_Solve(x0, 0.30f);
        assert(fabsf(Chassis_MPC.F[0] - prev[0]) <= dF + TOL);
        assert(fabsf(Chassis_MPC.F[1] - prev[1]) <= dF + TOL);
        prev[0] = Chassis_MPC.F[0];
        prev[1] = Chassis_MPC.F[1];
    }
}

/* 迭代次数必须有确定上界，否则控制周期不可预测。 */
static void test_mpc_iteration_bounded(void)
{
    uint32_t k;
    uint32_t worst = 0U;

    for (k = 0U; k < 200U; k++)
    {
        float x0[4];
        float s = ((k % 2U) == 0U) ? 0.30f : -0.30f;

        x0[0] = s; x0[1] = -s; x0[2] = 0.05f + 0.2f * (float)(k % 3U); x0[3] = s;
        Chassis_MPC_Solve(x0, 0.30f);
        if (Chassis_MPC.iter > worst) { worst = Chassis_MPC.iter; }
    }
    assert(worst <= (uint32_t)Chassis_Config.mpc.max_iter);
}

int main(void)
{
    /* MPC默认不接进F0，但求解器本身照常初始化。 */
    assert(Chassis_Config.output.mpc_flag == 0U);

    Chassis_MPC_Init();
    assert(Chassis_MPC.ready_flag == 1U);

    test_mpc_steady_state();
    test_mpc_roll_differential();
    test_mpc_height_common_mode();
    test_mpc_force_bounds();
    test_mpc_rate_limit();
    test_mpc_iteration_bounded();
    return 0;
}
