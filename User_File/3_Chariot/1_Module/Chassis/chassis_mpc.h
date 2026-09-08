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
    uint32_t cycles;     /* 单次求解的周期数。 */
    uint32_t cycles_max; /* 上电以来的最坏值。 */
    /*
     * 单次求解耗时，微秒。由 cycles 按运行时 SystemCoreClock 换算，Watch 直读。
     * ⚠ 这是【墙钟】时间，不是纯CPU时间：DWT在本任务被抢占期间照样计数，而
     *   MPC任务优先级最低，1kHz底盘任务和IMU任务都会插进来。判"一个求解周期
     *   内做不做得完"要的正是墙钟，所以这个口径是对的；但它不代表求解本身
     *   吃掉多少CPU。
     * ⚠ 读数期间不要暂停调试器：暂停时长会被算进来。之前几次异常大的读数
     *   就是这么来的。
     */
    uint32_t solve_us;
    uint32_t solve_us_max;
    /*
     * 求解完成次数。Watch里看它的增速就是【实际】求解频率：
     * MPC任务按100Hz自定时，但求解超过10ms时会自然降频，增速立刻反映出来。
     */
    uint32_t solve_count;
    /*
     * 最近一次求解结果的年龄，单位是底盘控制拍(1ms)。控制环每拍加一，
     * 求解完成清零。正常应在0~10之间徘徊；持续偏大说明MPC任务跟不上或被饿死。
     * ⚠ 目前只观测、不参与控制：F陈旧时控制环仍会照用。打开mpc_flag接管F0
     *   之前要先决定陈旧到什么程度该退回PID路径。
     */
    uint32_t age;
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
 * @brief 发布本拍的求解输入。由 1kHz 控制环每拍调用，只做赋值，不求解。
 *
 * @param x0     [alpha, d_alpha, H, d_H]。alpha用IMU实测roll，H用左右平均腿长。
 * @param H_ref  目标车身高度，m。
 */
void Chassis_MPC_SetInput(const float x0[4], float H_ref);

/**
 * @brief 求解一次，取滚动优化的第一组控制量。由 MPC 任务调用，不要在控制环里调。
 *
 * 吃的是 Chassis_MPC_SetInput() 最近发布的那组输入，结果写进全局 Chassis_MPC。
 *
 * ⚠ 单次求解是毫秒级，必须跑在独立的低优先级任务里。放进 1kHz 控制环会把那一拍
 * 撑爆：任务卡住 -> Chassis_Command_Send() 停发CAN -> DM在MIT模式下保持最后力矩
 * -> 遥控拨回中位也断不了电。见 commit 3caf81a。
 *
 * 输入输出都走裸全局、不加锁，和本工程其余部分一致。底盘任务优先级更高，可能在
 * 本函数写 F[0] 和 F[1] 之间抢占，读到跨求解步的一对力；偏差上界是配置的 dF_max，
 * 持续不超过一拍。双缓冲能消掉这点偏差，但那属于本工程刻意不做的封装。
 */
void Chassis_MPC_Solve(void);

#ifdef __cplusplus
}
#endif

#endif
