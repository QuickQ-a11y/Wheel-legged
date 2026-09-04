#ifndef CHASSIS_MPC_H
#define CHASSIS_MPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 车身高度与Roll的模型预测控制，对 TinyMPC 的唯一对外接口。
 *
 * Eigen 和 TinyMPC 全部关在 chassis_mpc.cpp 里，控制层看不到一行C++。
 * 模型、权重和约束都取自 Chassis_Config.mpc，见那里的注释。
 */

/** @brief MPC一次求解的结果与诊断量，Watch入口。 */
typedef struct
{
    float F[2];          /* 求出的左右腿支撑力，N。 */
    float x[4];          /* 本次喂进去的状态 alpha, d_alpha, H, d_H。 */
    float H_ref;         /* 本次的目标车身高度，m。 */
    uint32_t iter;       /* ADMM迭代次数。逼近max_iter说明没收敛，要么放宽要么调Q/R。 */
    uint32_t solved;     /* TinyMPC自报的收敛标志。 */
    uint32_t cycles;     /* 单次求解的CPU周期数，除以主频即秒。上机第一件事看它。 */
    uint32_t cycles_max; /* 上电以来的最坏值，判够不够10ms预算就看这个。 */
    uint8_t ready_flag;  /* 求解器已初始化。 */
} Chassis_MPC_t;

extern Chassis_MPC_t Chassis_MPC;

/**
 * @brief 建立求解器并预计算增益缓存。只在 Chassis_Init() 里调一次。
 *
 * TinyMPC 在这里 new 出全部工作矩阵，是唯一发生堆分配的地方；
 * 之后的 Chassis_MPC_Solve() 在控制环里零分配。
 */
void Chassis_MPC_Init(void);

/**
 * @brief 求解一次，取滚动优化的第一组控制量。
 *
 * @param x0     [alpha, d_alpha, H, d_H]。alpha用IMU实测roll，H用左右平均腿长。
 * @param H_ref  目标车身高度，m。
 *
 * 结果写进全局 Chassis_MPC，不用返回值——和工程其余流程型函数一致。
 */
void Chassis_MPC_Solve(const float x0[4], float H_ref);

#ifdef __cplusplus
}
#endif

#endif
