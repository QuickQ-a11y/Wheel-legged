#!/usr/bin/env bash
#
# MPC 主机侧测试。仓库里其余单测都是手工 gcc，只有这套需要 g++ 加一堆条件，
# 每次重拼太费事，固化在这里。
#
#   ./Tests/run_mpc_host.sh            断言测试 + 堆分配计数
#   ./Tests/run_mpc_host.sh assert     只跑 test_chassis_mpc.c 的断言
#   ./Tests/run_mpc_host.sh malloc     只数 Chassis_MPC_Solve 的 malloc 次数
#   ./Tests/run_mpc_host.sh stiff      量等效高度刚度 dF/dH 和稳态力（整定用）
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

    std::printf("  body_mass=%.2f kg  ->  F_eq = 0.5*M*g = %.2f N（= Uref，必须等于实测单腿静载）\n",
                M, 0.5f * M * g);
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
