#!/usr/bin/env bash
#
# MPC 主机侧测试。仓库里其余单测都是手工 gcc，只有这套需要 g++ 加一堆条件，
# 每次重拼太费事，固化在这里。
#
#   ./Tests/run_mpc_host.sh            断言测试 + 堆分配计数
#   ./Tests/run_mpc_host.sh assert     只跑 test_chassis_mpc.c 的断言
#   ./Tests/run_mpc_host.sh malloc     只数 Chassis_MPC_Solve 的 malloc 次数
#   ./Tests/run_mpc_host.sh stiff      量等效高度刚度 dF/dH 和稳态力（整定用）
#   ./Tests/run_mpc_host.sh step       闭环阶跃响应：超调/峰值时间/收敛（治超调用）
#   ./Tests/run_mpc_host.sh qr         扫 R 和 Q[2]：刚度 vs 超调的取舍表（治稳态误差用）
#
# 三个坑（照抄别的单测配方会失败）：
#   1. chassis_mpc.cpp 要 include "stm32h7xx.h" 取 DWT，主机上没有，本脚本现造一个桩。
#   2. chassis_config.c 用了数组下标指定初始化，是 C99 语法，必须用 gcc 编、不能用 g++。
#   3. admm.cpp 引用 benchmark_rho_adaptation，rho_benchmark.cpp 不能漏链。
#
set -u
cd "$(dirname "$0")/.."
ROOT=$(pwd)
MODE=${1:-all}
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

TM=User_File/1_Middleware/2_Algorithm/TinyMPC
CH=User_File/3_Chariot/1_Module/Chassis
INC="-I$TMP/inc -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm -I$CH -I$TM -I$TM/Eigen"
CXX_FLAGS="-std=gnu++17 -Os -fno-exceptions -fno-rtti -Wall -Wextra -Wno-missing-braces"
CC_FLAGS="-std=gnu11 -Os -Wall -Wextra"

mkdir -p "$TMP/inc" "$TMP/obj"

# ── DWT 桩 ────────────────────────────────────────────────────────────────────
cat > "$TMP/inc/stm32h7xx.h" <<'EOF'
#ifndef HOST_STUB_STM32H7XX_H
#define HOST_STUB_STM32H7XX_H
#include <stdint.h>
typedef struct { uint32_t DEMCR; } CoreDebug_Type_Stub;
/* LAR 是 Cortex-M7 DWT 的解锁寄存器，chassis_mpc.cpp 会写它，桩里必须有。 */
typedef struct { uint32_t CTRL; uint32_t CYCCNT; uint32_t LAR; } DWT_Type_Stub;
#ifdef __cplusplus
extern "C" {
#endif
extern CoreDebug_Type_Stub host_coredebug;
extern DWT_Type_Stub host_dwt;
extern uint32_t SystemCoreClock;   /* chassis_mpc.cpp 用它把 cycles 换成微秒 */
#ifdef __cplusplus
}
#endif
#define CoreDebug (&host_coredebug)
#define DWT (&host_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 0x01000000U
#define DWT_CTRL_CYCCNTENA_Msk     0x00000001U
#endif
EOF
cat > "$TMP/dwt_stub.cpp" <<'EOF'
#include "stm32h7xx.h"
CoreDebug_Type_Stub host_coredebug;
DWT_Type_Stub host_dwt;
uint32_t SystemCoreClock = 480000000U;
EOF

# ── malloc 计数垫片。数真实 malloc，不依赖 Eigen 的 assert 管线 ────────────────
cat > "$TMP/wrap.c" <<'EOF'
#include <stddef.h>
extern void *__real_malloc(size_t);        extern void  __real_free(void *);
extern void *__real_calloc(size_t, size_t); extern void *__real_realloc(void *, size_t);
int g_count_on = 0; long g_mallocs = 0; long g_frees = 0; long g_bytes = 0;
void *__wrap_malloc(size_t n)             { if (g_count_on) { g_mallocs++; g_bytes += (long)n; } return __real_malloc(n); }
void  __wrap_free(void *p)                { if (g_count_on && p) g_frees++; __real_free(p); }
void *__wrap_calloc(size_t a, size_t b)   { if (g_count_on) { g_mallocs++; g_bytes += (long)(a*b); } return __real_calloc(a, b); }
void *__wrap_realloc(void *p, size_t n)   { if (g_count_on) { g_mallocs++; g_bytes += (long)n; } return __real_realloc(p, n); }
EOF

cat > "$TMP/malloc_main.cpp" <<'EOF'
/*
 * 数 Chassis_MPC_Solve() 单次求解的堆分配次数。
 * 求解环里的 malloc 会让耗时不确定（newlib 是自由链表遍历，还会碎片化），
 * 这正是 1 kHz 控制环最怕的东西，所以要有个数。
 */
#include <cstdio>
extern "C" {
#include "chassis_mpc.h"
#include "chassis_config.h"
extern int g_count_on; extern long g_mallocs, g_frees, g_bytes;
}
static void probe(const char *tag, float a, float da, float H, float dH, float Href)
{
    float x0[4] = { a, da, H, dH };
    g_mallocs = g_frees = g_bytes = 0; g_count_on = 1;
    Chassis_MPC_SetInput(x0, Href);
    Chassis_MPC_Solve();
    g_count_on = 0;
    std::printf("  %-20s iter=%2u  malloc=%4ld  free=%4ld  字节=%5ld  每迭代≈%.0f\n",
                tag, Chassis_MPC.iter, g_mallocs, g_frees, g_bytes,
                Chassis_MPC.iter ? (double)g_mallocs / Chassis_MPC.iter : 0.0);
}
int main(void)
{
    Chassis_MPC_Init();
    if (Chassis_MPC.ready_flag != 1U) { std::printf("tiny_setup 失败\n"); return 2; }

    float warm[4] = { 0.1f, 0.0f, 0.18f, 0.0f };
    Chassis_MPC_SetInput(warm, 0.20f);
    Chassis_MPC_Solve();            /* 热一拍，排除首次的一次性开销 */

    std::printf("单次 Chassis_MPC_Solve 的堆分配:\n");
    probe("稳态",         0.00f, 0.0f, 0.20f,  0.0f, 0.20f);
    probe("小扰动",       0.05f, 0.2f, 0.19f, -0.1f, 0.20f);
    probe("大roll+高度差", 0.30f, 1.5f, 0.12f, -0.8f, 0.30f);
    probe("极端",         0.50f, 3.0f, 0.11f, -2.0f, 0.35f);

    /* 交替激励不让它 warm-start，逼近 max_iter */
    long worst = 0; uint32_t worst_iter = 0;
    for (int k = 0; k < 400; k++)
    {
        float s = (k % 2) ? 1.0f : -1.0f;
        float x0[4] = { s * 0.6f, s * 4.0f, 0.11f + 0.2f * (float)(k % 3), -s * 2.5f };
        g_mallocs = 0; g_count_on = 1;
        Chassis_MPC_SetInput(x0, 0.35f - 0.2f * (float)(k % 2));
        Chassis_MPC_Solve();
        g_count_on = 0;
        if (g_mallocs > worst) { worst = g_mallocs; worst_iter = Chassis_MPC.iter; }
    }
    std::printf("\n  实测最坏  malloc=%ld @ iter=%u\n", worst, worst_iter);
    std::printf("  按 max_iter=%u 外推 ≈ %.0f 次 malloc/求解",
                (unsigned)Chassis_Config.mpc.max_iter,
                worst_iter ? (double)worst / worst_iter * Chassis_Config.mpc.max_iter : 0.0);
    std::printf("，100 Hz 下约 %.0f 万次/秒\n",
                worst_iter ? (double)worst / worst_iter * Chassis_Config.mpc.max_iter * 100.0 / 1e4 : 0.0);
    std::printf("\n判据：求解环内应为 0。非 0 说明 tinyMatrix 的动态尺寸存储在起作用，\n");
    std::printf("      单次求解耗时会随堆状态漂移，无法给出可信的最坏值。\n");
    return 0;
}
EOF

cat > "$TMP/stiff_main.cpp" <<'EOF'
/*
 * 量 MPC 的等效高度刚度 dF/dH 和稳态输出力，整定 Q/R/body_mass 时用。
 *
 * 为什么需要它：MPC 没有积分作用，刚度决定了"未建模的恒定力偏差会造成多大的
 * 稳态腿长误差"。历史上踩过两次——
 *   1) R=0.05 而 rho=5.0，rho 把 R 淹没 100 倍，刚度只有 73 N/m，
 *      21N 的前馈误差 => 29cm 偏差 => 腿直接顶到机构限位。
 *   2) R 调到 500 以为能治抖，刚度掉回 ~80 N/m，遥控拨轮伸腿推不动。
 * 腿长 PID 的 kp=600 N/m 是天然的对照基准。
 */
#include <cstdio>
extern "C" {
#include "chassis_mpc.h"
#include "chassis_config.h"
}
static float settle(float H, float Href)
{
    /* 变化率约束每拍只放行 dF_max，要跑够拍数才到稳态 */
    for (int k = 0; k < 600; k++) {
        float x0[4] = { 0.0f, 0.0f, H, 0.0f };
        Chassis_MPC_SetInput(x0, Href);
        Chassis_MPC_Solve();
    }
    return Chassis_MPC.F[0];
}
int main(void)
{
    Chassis_MPC_Init();
    if (Chassis_MPC.ready_flag != 1U) { std::printf("tiny_setup 失败\n"); return 2; }
    const Chassis_MPC_Config_t *c = &Chassis_Config.mpc;
    const float M = Chassis_Config.model.body_mass, g = Chassis_Config.model.gravity;

    std::printf("  body_mass=%.2f kg  F0_gravity_scale=%.2f  ->  F_eq = %.2f N"
                "（= Uref，必须等于实测稳态单腿 F0）\n",
                M, Chassis_Config.F0_gravity_scale,
                0.5f * M * g * Chassis_Config.F0_gravity_scale);
    std::printf("  Q=[%.1e %.1e %.1e %.1e]  R=%.2f  rho=%.2f  ->  R1=R+rho=%.2f\n",
                c->Q[0], c->Q[1], c->Q[2], c->Q[3], c->R[0], c->rho, c->R[0] + c->rho);
    if (c->R[0] < c->rho) {
        std::printf("  ⚠ R < rho：rho 会淹没 R，Q/R 比值失效，刚度会远低于设计值\n");
    }

    const float H0 = 0.14f;
    float a = settle(H0, H0), b = settle(H0 + 0.01f, H0);
    std::printf("\n  H=%.3f(到位) -> F=%6.2f N\n", H0, a);
    std::printf("  H=%.3f(高1cm)-> F=%6.2f N\n", H0 + 0.01f, b);
    std::printf("  等效高度刚度 dF/dH = %.0f N/m      （腿长PID 的 kp = 600 N/m）\n", (a - b) / 0.01f);
    std::printf("  该刚度下 21N 未建模偏差 -> 稳态腿长误差 %.1f cm\n", 21.0f / ((a - b) / 0.01f) * 100.0f);
    return 0;
}
EOF

cat > "$TMP/step_main.cpp" <<'EOF'
/*
 * 闭环阶跃响应。整定"伸腿收腿超调"时用这个。
 *
 * ⚠ 不要靠改 mpc.damping：那一项是【被控对象模型】里的物理阻尼，进的是
 * Ad(3,3) = 1 - damping/M*Ts，本车取值下只有 0.01 量级，改它几乎不动闭环；
 * 而且方向是反的——调大等于告诉 MPC"对象自己会衰减"，MPC 更不愿出力压速度。
 * 代价里的阻尼旋钮是 Q[3]（d_H 的权重）。
 *
 * 但标称闭环（零延迟、模型=真实）本来就几乎不超调，所以实机超调多半来自
 * 模型里【没有】的两件事，本程序把它们做成可扫的参数：
 *   argv[1] delay  —— F 的滞后拍数(1kHz)。MPC 跑在独立低优先级任务、按
 *                     decimation 自定时，控制环拿到的 F 天然是旧的
 *                     （见 Chassis_MPC.age，它目前只观测不参与控制）。
 *   argv[2] m_mul  —— 真实质量 / 模型质量。model.body_mass 填大了会让 MPC
 *                     以为机体更重、出力过量，直接表现为超调。
 *
 * ⚠ 还要看 F 有没有撞 F_min/F_max：力箱对 F_eq 是不对称的，撞了就不是 Q 的事。
 */
#include <cstdio>
#include <cstdlib>
#include <cmath>
extern "C" {
#include "chassis_mpc.h"
#include "chassis_config.h"
}

#define MAX_DELAY 64

struct Result { float overshoot_pct, t_peak, t_settle; float F_lo, F_hi; int clipped; };

static Result run_step(float H0, float dH_step, int delay, float m_mul)
{
    const Chassis_MPC_Config_t *c = &Chassis_Config.mpc;
    const float M_model = Chassis_Config.model.body_mass;
    const float M_true = M_model * m_mul;
    const float g = Chassis_Config.model.gravity;
    const float dt = APP_CTRL_DT_S;
    const int   dec = (int)c->decimation;
    const float H_ref = H0 + dH_step;
    const int   steps = (int)(3.0f / dt);
    const float tol = 0.02f * fabsf(dH_step);
    /*
     * 平衡点按【真实】质量，否则起点就不静止，量到的超调里混了初始暂态。
     * 必须带 F0_gravity_scale：被控对象要代表【实机】，而实机的恒定支撑需求
     * 就是含未建模力的那一份（髋部分担的腿自重 + 静摩擦）。少乘它，标称工况
     * 就变成了人为制造的 21% 失配，量出来的超调全是假的。
     */
    const float F_eq_true = 0.5f * M_true * g * Chassis_Config.F0_gravity_scale;

    float H = H0, dH = 0.0f;
    float pipe[MAX_DELAY + 1];
    for (int i = 0; i <= MAX_DELAY; i++) { pipe[i] = F_eq_true; }
    float Fl = F_eq_true, Fr = F_eq_true, F_hold = F_eq_true;
    float peak = 0.0f, t_peak = 0.0f, t_settle = -1.0f;
    float F_lo = F_eq_true, F_hi = F_eq_true;
    int clipped = 0, head = 0;

    for (int k = 0; k < steps; k++) {
        if (k % dec == 0) {
            float x0[4] = { 0.0f, 0.0f, H, dH };
            Chassis_MPC_SetInput(x0, H_ref);
            Chassis_MPC_Solve();
            float Fnew = Chassis_MPC.F[0];
            if (Fnew < F_lo) { F_lo = Fnew; }
            if (Fnew > F_hi) { F_hi = Fnew; }
            if ((Fnew <= c->F_min + 1.0e-3f) || (Fnew >= c->F_max - 1.0e-3f)) { clipped++; }
            F_hold = Fnew;
        }
        /*
         * ⚠ 流水线必须【每拍】都推入当前保持值，不能只在求解拍写：
         * head 每拍前进，只在求解拍写的话 9/10 的拍会读到陈旧格子，
         * 零阶保持被打断，量出来的超调全是假的（第一版就踩了这个）。
         */
        pipe[head] = F_hold;
        int tap = (head - delay + MAX_DELAY + 1) % (MAX_DELAY + 1);
        Fl = Fr = pipe[tap];
        head = (head + 1) % (MAX_DELAY + 1);

        /* 恒定项与 F_eq_true 同源：平衡时 Fl=Fr=F_eq_true 正好使 ddH=0 */
        float ddH = (Fl + Fr) / M_true - (c->damping / M_true) * dH
                    - (2.0f * F_eq_true / M_true);
        dH += ddH * dt;
        H  += dH * dt;

        float over = (dH_step > 0.0f) ? (H - H_ref) : (H_ref - H);
        if (over > peak) { peak = over; t_peak = (float)k * dt; }
        if (fabsf(H - H_ref) > tol) { t_settle = -1.0f; }
        else if (t_settle < 0.0f) { t_settle = (float)k * dt; }
    }

    Result r;
    r.overshoot_pct = 100.0f * peak / fabsf(dH_step);
    r.t_peak = t_peak; r.t_settle = t_settle;
    r.F_lo = F_lo; r.F_hi = F_hi; r.clipped = clipped;
    return r;
}

static void one_row(const char *tag, float H0, float d, int delay, float m_mul)
{
    Result r = run_step(H0, d, delay, m_mul);
    char sb[16];
    if (r.t_settle < 0.0f) { std::snprintf(sb, sizeof(sb), "未收敛"); }
    else { std::snprintf(sb, sizeof(sb), "%.3f", r.t_settle); }
    std::printf("  %-12s %6.1f   %8.3f   %9s  %5.1f~%-6.1f  %d\n",
                tag, r.overshoot_pct, r.t_peak, sb, r.F_lo, r.F_hi, r.clipped);
}

int main(int argc, char **argv)
{
    Chassis_MPC_Init();
    if (Chassis_MPC.ready_flag != 1U) { std::printf("tiny_setup 失败\n"); return 2; }
    const Chassis_MPC_Config_t *c = &Chassis_Config.mpc;
    const float M = Chassis_Config.model.body_mass, g = Chassis_Config.model.gravity;
    const int   delay = (argc > 1) ? std::atoi(argv[1]) : 0;
    const float m_mul = (argc > 2) ? (float)std::atof(argv[2]) : 1.0f;
    const float H0 = 0.15f;

    std::printf("  Q=[%.1e %.1e %.1e %.1e]  R=%.2f  rho=%.2f  R1=%.2f   delay=%d拍  真实质量/模型=%.2f\n",
                c->Q[0], c->Q[1], c->Q[2], c->Q[3], c->R[0], c->rho, c->R[0] + c->rho, delay, m_mul);
    std::printf("  F_eq=%.1f N  力箱[%.0f,%.0f] => 往下刹车 %.0f N / 往上 %.0f N%s\n",
                0.5f * M * g * Chassis_Config.F0_gravity_scale, c->F_min, c->F_max,
                0.5f * M * g * Chassis_Config.F0_gravity_scale - c->F_min,
                c->F_max - 0.5f * M * g * Chassis_Config.F0_gravity_scale,
                (0.5f * M * g * Chassis_Config.F0_gravity_scale - c->F_min) * 2.0f <
                (c->F_max - 0.5f * M * g * Chassis_Config.F0_gravity_scale) ||
                (c->F_max - 0.5f * M * g * Chassis_Config.F0_gravity_scale) * 2.0f <
                (0.5f * M * g * Chassis_Config.F0_gravity_scale - c->F_min)
                    ? "  ← ⚠ 不对称" : "");
    if (c->R[0] < c->rho) { std::printf("  ⚠ R < rho：rho 淹没 R\n"); }
    else if (c->R[0] < 3.0f * c->rho) {
        std::printf("  ⚠ R 与 rho 同量级：R1=R+rho 把有效输入权重抬了 %.1f 倍\n",
                    (c->R[0] + c->rho) / c->R[0]); }

    std::printf("\n  动作          超调%%   峰值时刻s  收敛(±2%%)s  F范围N          撞限\n");
    one_row("伸腿 +5cm", H0, +0.05f, delay, m_mul);
    one_row("收腿 -5cm", H0, -0.05f, delay, m_mul);
    one_row("伸腿+10cm", H0, +0.10f, delay, m_mul);
    one_row("收腿-10cm", H0, -0.10f, delay, m_mul);
    return 0;
}
EOF

# ── 编译 ──────────────────────────────────────────────────────────────────────
# test_chassis_mpc.c 的 main() 断言 output.mpc_flag == 0U（出厂安全默认）。
# 实机整定期间这一位会被置 1，那时直接编就会在第一个断言 abort。
# 和 test_chassis_recovery 需要 sed 掉三道输出门是同一类问题：测试写死了可调参数。
# 这里编一份把 mpc_flag 强制回 0 的副本，不动工作区的 chassis_config.c。
CFG="$TMP/chassis_config.c"
sed 's/\.mpc_flag = 1U,/.mpc_flag = 0U,/' "$CH/chassis_config.c" > "$CFG"

build_common() {
    gcc $CC_FLAGS -DEIGEN_NO_DEBUG $INC -c "$CFG" -o "$TMP/obj/cfg.o" || return 1
    for f in "$TM/tinympc/tiny_api.cpp" "$TM/tinympc/admm.cpp" "$TM/tinympc/rho_benchmark.cpp" \
             "$CH/chassis_mpc.cpp" "$TMP/dwt_stub.cpp"; do
        n=$(basename "$f" .cpp)
        # vendored TinyMPC 有一批 unused-variable，与固件 CMakeLists 一样单独放宽
        extra=""; case "$n" in tiny_api|admm|rho_benchmark) extra="-Wno-unused-variable -Wno-unused-parameter";; esac
        g++ $CXX_FLAGS $extra -DEIGEN_NO_DEBUG $INC -c "$f" -o "$TMP/obj/$n.o" || return 1
    done
}

fail=0
build_common || { echo "编译公共部分失败"; exit 1; }

if [ "$MODE" = all ] || [ "$MODE" = assert ]; then
    echo "═══════ 断言测试 (test_chassis_mpc.c) ═══════"
    gcc $CC_FLAGS -DEIGEN_NO_DEBUG $INC -c Tests/test_chassis_mpc.c -o "$TMP/obj/t.o" \
        && g++ -o "$TMP/test_mpc" "$TMP/obj"/*.o -lm
    # ⚠ 必须查编译退出码再跑，否则会跑到上一次的旧二进制、误报 PASS
    if [ $? -ne 0 ]; then echo "编译失败"; fail=1; else
        "$TMP/test_mpc" && echo "PASS" || { echo "FAIL"; fail=1; }
    fi
    rm -f "$TMP/obj/t.o"
fi

if [ "$MODE" = step ]; then
    echo "═══════ 闭环阶跃响应 ═══════"
    g++ $CXX_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/step_main.cpp" -o "$TMP/obj/step.o" \
        && g++ -o "$TMP/sp" "$TMP/obj"/*.o -lm
    if [ $? -ne 0 ]; then echo "编译失败"; fail=1; else
        echo "--- 标称：零延迟、模型=真实 ---"
        "$TMP/sp" 0 1.0 || fail=1
        echo
        echo "--- 扫 F 滞后拍数（MPC 跑独立任务，控制环拿到的 F 天然是旧的）---"
        for d in 5 10 20 30; do
            echo "  ── delay = $d 拍 (${d} ms) ──"
            "$TMP/sp" "$d" 1.0 | sed -n '/动作/,$p' | tail -4
        done
        echo
        echo "--- 扫质量失配（模型 body_mass 填大了 = MPC 出力过量）---"
        for m in 0.5 0.7 1.3; do
            echo "  ── 真实/模型 = $m ──"
            "$TMP/sp" 0 "$m" | sed -n '/动作/,$p' | tail -4
        done
    fi
fi

if [ "$MODE" = qr ]; then
    echo "═══════ Q/R 取舍表：刚度 vs 超调 ═══════"
    echo "  稳态误差 = 未建模力 / 刚度。刚度越高误差越小，但超调和撞限风险上升。"
    echo
    g++ $CXX_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/stiff_main.cpp" -o "$TMP/obj/sm.o" || exit 1
    g++ $CXX_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/step_main.cpp"  -o "$TMP/obj/step.o" || exit 1
    printf "  %-22s %10s %9s %9s %7s\n" "参数" "刚度N/m" "21N->cm" "超调%" "收敛s"
    sweep_one() {                       # $1=标签 $2=sed表达式
        sed -E "$2" "$CFG" > "$TMP/cfg_qr.c"
        gcc $CC_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/cfg_qr.c" -o "$TMP/obj/cfg.o" 2>/dev/null || {
            printf "  %-22s 编译失败\n" "$1"; return; }
        g++ -o "$TMP/k1" "$TMP/obj/sm.o" $(ls "$TMP/obj"/*.o | grep -v -e step.o -e sm.o) -lm 2>/dev/null
        g++ -o "$TMP/k2" "$TMP/obj/step.o" $(ls "$TMP/obj"/*.o | grep -v -e step.o -e sm.o) -lm 2>/dev/null
        k=$("$TMP/k1" | grep -oP '刚度 dF/dH = \K[0-9]+')
        e=$("$TMP/k1" | grep -oP '稳态腿长误差 \K[0-9.]+')
        # step 输出列：$1=伸腿 $2=+5cm $3=超调% $4=峰值时刻 $5=收敛s
        r=$("$TMP/k2" 0 1.0 | grep '伸腿 +5cm' | awk '{print $3, $5}')
        printf "  %-22s %10s %9s %9s %7s\n" "$1" "${k:-?}" "${e:-?}" $r
    }
    R_NOW=$(sed -n 's/.*\.R = { *\([0-9.e+-]*\)f.*/\1/p' "$CFG" | tail -1)
    Q2_NOW=$(sed -n 's/.*\.Q = {[^,]*,[^,]*, *\([0-9.e+-]*\)f.*/\1/p' "$CFG" | tail -1)
    sweep_one "当前 R=$R_NOW Q2=$Q2_NOW" "s/__NOPE__/x/"
    for r in 15 8 3 1; do
        sweep_one "R=$r (Q2=$Q2_NOW)" "s/\.R = \{ *[0-9.e+-]+f, *[0-9.e+-]+f *\}/.R = { ${r}.0f, ${r}.0f }/"
    done
    for q in 4.0e7 8.0e7; do
        sweep_one "Q2=$q (R=$R_NOW)" "s/(\.Q = \{[^,]*,[^,]*,)[^,]*,/\1 ${q}f,/"
    done
    gcc $CC_FLAGS -DEIGEN_NO_DEBUG $INC -c "$CFG" -o "$TMP/obj/cfg.o"
    echo
    echo "  ⚠ 改 R 时必须复查它与 rho 的相对大小：R < 3*rho 时 R1=R+rho 会把有效输入权重抬高。"
fi

if [ "$MODE" = stiff ]; then
    echo "═══════ 等效高度刚度 ═══════"
    g++ $CXX_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/stiff_main.cpp" -o "$TMP/obj/sm.o" \
        && g++ -o "$TMP/st" "$TMP/obj"/*.o -lm
    if [ $? -ne 0 ]; then echo "编译失败"; fail=1; else "$TMP/st" || fail=1; fi
fi

if [ "$MODE" = all ] || [ "$MODE" = malloc ]; then
    echo
    echo "═══════ 求解环堆分配计数 ═══════"
    gcc $CC_FLAGS -c "$TMP/wrap.c" -o "$TMP/obj/wrap.o" \
        && g++ $CXX_FLAGS -DEIGEN_NO_DEBUG $INC -c "$TMP/malloc_main.cpp" -o "$TMP/obj/mm.o" \
        && g++ -Wl,--wrap=malloc,--wrap=free,--wrap=calloc,--wrap=realloc \
               -o "$TMP/mc" "$TMP/obj"/*.o -lm
    if [ $? -ne 0 ]; then echo "编译失败"; fail=1; else "$TMP/mc" || fail=1; fi
fi

exit $fail
