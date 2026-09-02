# TinyMPC 上车前的学习路线

面向：轮腿底盘（STM32H723），目标是把 TinyMPC 用在**腿子系统**（车身高度 + 横滚），
顺带解决腿长 PID 与 roll PID 打架的问题。

**本文只讲原理和该学什么，不含任何代码改动。**

---

## 〇、先回答一个问题，答不上来就先别上 MPC

**MPC 相对 LQR，唯一真正多出来的能力是「显式处理约束」和「利用未来信息」。**
其他所有说法（"更先进""更智能"）都是幻觉。

所以先问自己：**你能说出一条必须靠约束才能表达的需求吗？** 比如：

- `F0` 不能超过地面附着力，否则轮子打滑
- 关节力矩不能超过 `joint_T_limit`，且**饱和时希望优雅降级而不是硬砍**
- 腿长必须留在 `[L0_min, L0_max]` 内，且**接近边界时提前减速**而不是撞上去
- 横滚恢复时不能让某条腿卸载到离地

如果能说出来 —— MPC 值得上，因为 LQR 只能事后限幅，limiter 一夹最优性就没了。
如果说不出来 —— 你要的其实是**更好的模型**或**更好的坐标**，不是更好的求解器。

> 你目前最痛的「PID 打架」属于后者：那是坐标选错（腿长 PID 同时管住了共模和差模），
> 换坐标就能解决，MPC 只是顺手也能解决。想清楚这一点再决定投入。

---

## 〇·五、三个目标功能分别靠什么（先看这张表）

目标是复刻 Qi-Q26 那篇 PDF 的三个功能。**但它们不是同一个难度，也不都靠 MPC。**

| 功能 | 指标 | 真正靠什么 | 结论 |
|---|---|---|---|
| **③ 任意地形 Roll 自稳** | 稳态误差 ≤2° | 坐标解耦（✅已做）+ **LESO** | ⚠️ **MPC 帮不上主要忙**。稳态误差靠扰动估计消除，不靠预测 |
| **② 高速飞坡落地缓冲** | 冲击后 <1s 收敛不发散 | QR 权重 + **力上下界约束** | ✅ 可行。PDF 自己说「通过调整 QR 矩阵完成」——那 LQR 也能做；**MPC 的真增量是冲击时的力约束** |
| **① 侧身下台阶** | 悬空腿 <100ms 主动伸出，Roll 动态偏差 ≤5° | 预测 + **接触状态** | ✅ 可行，但有 PDF 未讲的坑（见下） |

**这张表的含义**：你这一个月要做的 LESO，**本身就能拿下功能③**，
并且是功能①②的底座（模型误差被实时补掉，MPC 才有意义）。
不要以为三个功能都得等 MPC 上车才有。

### 功能①的坑：单腿悬空时模型是错的

模型 `M·Ḧ = (F_Nl + F_Nr)·cos α − b·Ḣ − M·g` 假设**两条腿都踩在地上**。
单腿悬空时那条腿产生不了支撑力，B 矩阵对应列实际归零——
MPC 会以为悬空腿在出力，于是对着地腿的修正量算少了。

**你已经有解决它的信号**：`Chassis.ground.off_ground_flag[side]`（单腿离地结论，
带 `off_time`/`land_time` 滞回）。

**但在线换模型有硬限制**（已核实 TinyMPC 源码）：
`tiny_precompute_and_set_cache()` 虽是公开 API，内部要跑 Riccati 迭代 + 矩阵求逆
（`tiny_api.cpp:337-338`），**不可能每周期在 MCU 上重算**。
唯一可行做法是**离线预生成 3 份 cache**（双腿着地／左悬空／右悬空），
运行时按观测器结论切指针。cache 只含 `Kinf(2×4)`、`Pinf(4×4)`、`Quu_inv(2×2)`、
`AmBKt(4×4)`，3 份内存开销可忽略。

**第一版不做这个**——先用单模型，悬空带来的模型误差正好交给 LESO 当扰动吸收。
多 cache 留到第二个月。

---

## 一、贯穿全局的核心认知

**LQR 是 MPC 的一个特例：无限时域 + 无约束。**

反过来，**TinyMPC 内部就在用 LQR**。它缓存的 `Kinf`、`Pinf`（见 `types.hpp` 的 `TinyCache`）
就是无限时域 LQR 的增益和 Riccati 解，被当作 MPC 的**终端代价**用。

这意味着两件事：

1. 你在 `ABK_LQR.m` 上做的全部建模工作**不会浪费**，它是 MPC 的组成部分而非替代品。
2. 你要学的不是"另一套控制理论"，而是"在 LQR 外面套一层约束处理"。

学习时始终抓这条主线：**LQR → 加有限时域 → 加约束 → 变成 QP → 用 ADMM 解 → 缓存加速 = TinyMPC**。

---

## 二、你已经有的 vs 你缺的

| 能力 | 现状 |
|---|---|
| 状态空间建模 | ✅ `ABK_LQR.m` 用拉格朗日符号推导出 10 维 A/B |
| LQR 求解 | ✅ 在用，`lqr(A_ac, B_ac, Q, R)` |
| Q/R 权重整定 | ⚠️ 在用但多半是试出来的，缺系统方法 |
| **离散化** | ❌ **完全没有**。用的是 `lqr()` 不是 `dlqr()`，全文件无 `c2d` |
| 凸优化 / QP | ❌ |
| ADMM | ❌ |
| 约束型控制 | ❌ 现在靠事后 limiter |
| **模型参数可信度** | ❌ **最大短板**，见下 |

### ⚠️ 最大短板：模型参数没有一个是验证过的

- `ABK_LQR.m` 当前生效参数是**小车**的（`m_b_ac = 3.043`），固件编译的是**大车**（`body_mass = 12.0`）
- `I_z` 缺平行轴项，约 2 倍偏小
- `l_c_ac`（质心到髋轴）从未实测
- 平衡点 `theta_b` 从未实测

**MPC 对模型误差比 LQR 敏感得多。** LQR 只用当前状态算一次反馈，模型偏一点就是增益偏一点；
MPC 会基于模型**预测未来 N 步**并按预测行动，模型错了就是朝错误方向坚定地走 N 步。

**在参数对齐之前上 MPC，效果一定比现在差。** 这是我最强的一条建议。

---

## 二·五、四周日程（一个月 = 学习 + LESO 落地）

**MPC 上车放到第二个月。**这一个月的交付物是「LESO 在实机上跑起来、Pitch 波动明显下降」，
外加「MPC 理论学完 + 主机上量到真实耗时，据此决定上不上车」。

| 周 | 干什么 | 主要材料 |
|---|---|---|
| **1** | 补基础 + LESO 理论 | DR_CAN 3.5 离散化、B站 状态观测器、**Han 2009**、**Gao 2003**、**arXiv 2104.01943** |
| **2** | **MATLAB 里算出 `A_e/B_e/L_e`** | `Others/Qi-Q26-Leg_Robot/LQR_calc.m` 模板 |
| **3** | **LESO 写进固件 + 架空调 `ω_o`** | ——落地周，不学新东西 |
| **4** | MPC 理论 + 主机验证 | DR_CAN MPC 系列 + TinyMPC 论文；主机跑通 TinyMPC 量耗时 |

第 1 周细分：

- **Day 1**：离散化（DR_CAN 3.5，10 分钟）+ 状态观测器与极点配置（B 站，半天）
- **Day 2~3**：**Han 2009《From PID to ADRC》**（7 页）—— ADRC 之父韩京清亲笔，
  读它比看几小时视频快；天大那个视频当补充
- **Day 4**：**Gao 2003 带宽参数化**（6 页）—— 学会只调一个 `ω_o` 而不是逐个配极点。
  经验值：**观测器带宽取控制器带宽的 5~10 倍**
- **Day 5**：**arXiv 2104.01943《Minimum-Footprint Discrete-Time ADRC》**
  —— 直接讲嵌入式上怎么实现离散 LESO，正是你要干的事

### 移到第二个月的

- ADMM 深入（第 4 周只需要读懂 TinyMPC 在干什么，不必吃透收敛性）
- MPC 上车（双速率任务、实机调 QR）
- 接触状态多 cache 切换（功能①的完整版）
- 力变化率约束（需要状态增广或 TinyMPC 的通用线性约束）

### 明确砍掉的（以及为什么）

| 砍掉 | 理由 |
|---|---|
| 凸优化系统课（Boyd 书前 5 章 / EE364A） | 读 ADMM 综述前 3 章**不需要**完整凸优化背景 |
| MPC 稳定性证明、递归可行性 | 工程上先不需要。知道"终端代价取 LQR 的 `Pinf`"就够了 |
| Rawlings & Mayne 全书 | 700 页参考书，一个月里读它是浪费 |
| **系统辨识专题** | **做了 LESO 之后紧迫性大幅下降**——LESO 存在的意义就是把模型误差当扰动估掉。<br>只保留两件各半天的实测，和第 2 周并行：① `I_z` 补平行轴项 ② 悬挂法实测 `cg_to_hip` |
| 非线性 MPC / CasADi / SQP | 参考工程实测单周期 20~100 ms，单片机根本不可能。直接排除 |

---

## 三、学习清单（按优先级，附搜索关键词）

### 第 0 层：补上离散化（**半天**，必须最先做）

**为什么**：MPC 天生是离散的（预测第 1 步、第 2 步……），而你的 A/B 是连续的。这是硬前提。

**中文视频（首选）**
- **DR_CAN《Advanced 控制理论》[3.5 连续系统离散化](https://space.bilibili.com/230105574/channel/seriesdetail?sid=1569601)**
  —— 10 分钟，正好补你缺的这一课，整个合集也覆盖状态空间、能控能观、李雅普诺夫

**搜索关键词**
- 连续系统离散化、零阶保持 ZOH、`c2d`
- 矩阵指数 `expm`、采样周期选择

**顺带补牢 LQR**：你在用 LQR 但没推过 Riccati。看完下面两集，再回头看 TinyMPC 的
`Kinf`/`Pinf` 会豁然开朗——那就是这里推出来的东西，被当成 MPC 的终端代价缓存起来了。

- [【最优控制】5_线性二次型调节器(LQR)详细数学推导](https://www.bilibili.com/video/BV1dm4y177tA/)（DR_CAN，7.4万播放）
- [【最优控制】6_线性二次型调节器(LQR)案例分析与代码详解](https://www.bilibili.com/video/BV11V4y1t7z7/)（DR_CAN，Octave 演示，代码基本可直接搬到 MATLAB）

**学会的判据**：能自己写出 `Ad = expm(A*Ts)`、`Bd = A⁻¹(Ad − I)B` 并解释为什么 ZOH 是这个形式；
能说清 1 kHz 采样对你这个系统是过采样还是刚好。

---

### 第 1 层：LESO 扩张状态观测器（**第 1 周，下一步要做的**）

**为什么先学它**：

1. **功能③（Roll 稳态 ≤2°）主要由它保证**，不是 MPC。稳态误差靠扰动估计消掉，预测帮不上忙
2. 纯线性代数、无求解器、执行时间恒定，1 kHz 在 C 里毫无压力，**不需要引入 C++/Eigen**
3. 直接补你最大的短板——模型参数不可信（MATLAB 与固件不一致、`I_z` 偏小、质心未标定）
4. 它是 MPC 的底座：模型误差被实时补掉之后，MPC 的预测才有意义

参考工程实测 Pitch 波动从 3.05° 降到 1.56°，**抑制率 50%**。

**核心思想一句话**：把「模型没建准的部分 + 外部扰动」打包成一个**虚拟状态**，
用观测器把它估出来，再从控制量里减掉。于是被控对象在控制器眼里退化成干净的积分串联型。

**必须先懂的三块**（都在现代控制理论范围内）：

1. **状态观测器**：观测器是被观测对象的"克隆"，靠输出误差 `y − Cx̂` 修正
2. **对偶原理**：观测器设计 = 对偶系统的状态反馈设计，所以能用 `place()` 配极点
3. **可观性**：不可观的状态估不出来，扩张之后要重新验证

**英文论文（首选，都很短，配 Zotero 读）**
- **[Han 2009《From PID to Active Disturbance Rejection Control》](https://mechatronics.ucmerced.edu/sites/g/files/ufvvjh1226/f/page/documents/04796887_0.pdf)**
  （IEEE TIE 56(3):900-906，7 页）—— ADRC 之父韩京清亲笔，读它比看几小时视频快
- **[Gao 2003《Scaling and Bandwidth-Parameterization Based Controller Tuning》](http://congres.cran.univ-lorraine.fr/2003/ACC%202003/Papers/FP03-3.PDF)**
  （ACC 2003，6 页）—— **整定的关键**：所有观测器极点放在 `-ω_o`，只调一个带宽参数，
  不用逐个 `place()`。经验值：观测器带宽取控制器带宽的 **5~10 倍**
- **[arXiv 2104.01943《A Minimum-Footprint Implementation of Discrete-Time ADRC》](https://arxiv.org/pdf/2104.01943)**
  —— 直接讲嵌入式上怎么实现离散 LESO，正是你要干的事
- 备查：[arXiv 2211.07309《Tuning and Implementation Variants of Discrete-Time ADRC》](https://arxiv.org/pdf/2211.07309)

#### 先分清四个名词（很多资料混着用）

```
ADRC  自抗扰控制（韩京清）= TD 跟踪微分器 + ESO 扩张状态观测器 + NLSEF 非线性误差反馈
 └─ ESO   扩张状态观测器 —— ADRC 的核心，可以是非线性的
LADRC 线性自抗扰控制（高志强）= 把上面三块全线性化，参数整定变成"调带宽"
 └─ LESO  线性扩张状态观测器 —— LADRC 里的观测器部分
```

**你要做的是「LQR + LESO」，不是完整的 LADRC。**

区别在于：LADRC 用的是自己那套线性误差反馈（本质是 PD），而你的控制律是**十维 LQR**。
你只借用 LESO 这个观测器，把它估出来的总扰动从 LQR 的输出力矩里减掉。
参考工程 Qi-Q26 也是这么干的。

**所以学 ADRC 时别被完整框架带偏**：TD（跟踪微分器）和 NLSEF（误差反馈）这两块你用不上，
重点只在 ESO/LESO 的构造、收敛条件和带宽整定。

#### LADRC 专项材料

- **[【LADRC】线性自抗扰控制](https://blog.csdn.net/weixin_41276397/article/details/127353049)（CSDN）**
  —— 原理、推导、二阶与 n 阶系统、整定策略，覆盖最全
- [LADRC 结构学习与实践](https://blog.csdn.net/qq_38169460/article/details/97243120)（CSDN）
- [线性自抗扰控制（LADRC）实战：从原理到代码](https://www.cnblogs.com/ljbguanli/p/19928796)（博客园）—— 偏工程实现
- **[一种新型控制方法——自抗扰控制技术及其工程应用综述](https://html.rhhz.net/tis/html/201711029.htm)**
  （《智能系统学报》）—— **中文综述，正经期刊**，想一次看清 ADRC 全貌就看这篇
- **[基于 PID 参数整定的线性自抗扰控制参数整定](http://kzyjc.alljournals.cn/html/2021/7/20210706.htm)**
  （《控制与决策》）—— **实用度最高的一篇**：讲怎么把**已有的 PID 参数转化成 LADRC 参数**。
  你手上已经有调好的腿长/横滚 PID，这条路能省掉从零试参数

**中文文章（建立直觉用，快而好读）**
- [自抗扰控制-ADRC](https://zhuanlan.zhihu.com/p/664345718)（知乎）—— ADRC 整体框架
- [自抗扰控制ADRC及其在电机速度环中的应用](https://zhuanlan.zhihu.com/p/692439642)（知乎）—— 有具体工程应用，好对照
- [【自抗扰控制ADRC】扩张观测器ESO](https://blog.csdn.net/m0_37835056/article/details/130541998)（CSDN）—— 专讲 ESO，带 Simulink
- [自抗扰控制（ADRC）—— 扩展状态观测器](https://blog.csdn.net/itnerd/article/details/104426955)（CSDN）

> 中文文章适合先建直觉，**但整定细节以 Gao 2003 为准**——带宽参数化那套中文二手材料讲得普遍不清楚。

**中文视频（作为补充/入门）**
- [【现代控制理论】一题掌握！状态观测器](https://www.bilibili.com/video/BV19K1vBNEsx/) —— 观测器 + 极点配置，最直接对口
- [天津大学《控制理论基础2025》14 从PID到自抗扰控制ADRC](https://www.bilibili.com/video/BV1DACTB1E2r/) —— LESO 的来源背景，讲 ESO 在 ADRC 里的角色
- [《现代控制理论》期末速成课 6 小时](https://www.bilibili.com/video/BV1gu411Y7f8/) —— 状态空间、能控能观、状态反馈与观测器，赶进度时用
- DR_CAN《Advanced 控制理论》合集 —— 状态空间和能观性部分

**搜索关键词**
- 状态观测器 / 龙贝格观测器 / Luenberger observer
- 极点配置 / pole placement、MATLAB `place()` 函数
- 对偶原理 / duality principle
- 可观性 / 能观测性 / observability
- **扩张状态观测器 / ESO / LESO / Extended State Observer**
- **自抗扰控制 / ADRC / 韩京清**（LESO 的出处）
- 总扰动 / lumped disturbance
- **带宽参数化整定 / bandwidth parameterization（高志强方法）**
  ← 比逐个配极点简单得多：所有观测器极点放在 `-ω_o`，只调一个带宽参数

**学会的判据**：能解释为什么 `A_e = [A B; 0 I]` 这样扩张就等于"假设扰动慢变"，
以及观测器带宽调高会带来什么代价（噪声放大）。

**现成模板**：`Others/Qi-Q26-Leg_Robot/LQR_calc.m` 完整走了一遍
`c2d` → `dlqr` → 构造 `A_e/B_e/C_e` → `place()` 求 `L_e` → 按腿长多项式拟合。
最后这步和你固件里 `Algorithm_LQR_FitLqrKPoly22` 是同一个套路，基础设施已经有了。

---

### 第 2 层：MPC 本体（**第 3 周**）

**为什么**：这是主线。

**中文文章（配合视频看，推导更完整）**
- **[Lecture 9 Convex 模型预测控制（MPC）](https://zhuanlan.zhihu.com/p/629148064)（知乎）—— 最对口的一篇**：
  讲 Convex MPC 怎么转成 QP，以及**利用 KKT 系统 Hessian 稀疏性加速求解**，
  这正是 TinyMPC 结构化解法的思路
- [MPC在机器人中的应用（一）](https://zhuanlan.zhihu.com/p/407911153)（知乎）—— 机器人语境，建模/预测/控制三段式
- [学习模型预测控制 (MPC) 原理与数学推导](https://zhuanlan.zhihu.com/p/2012626044969059405)（知乎）—— 带 MATLAB 代码
- [MPC（模型预测控制）原理及理论推导](https://zhuanlan.zhihu.com/p/698526965)（知乎）
- [模型预测控制（MPC）原理及详细推导](https://blog.csdn.net/weixin_43487974/article/details/126352999)（CSDN）—— 代价函数→QP 的推导写得细

**中文视频（首选）**
- **[DR_CAN《MPC模型预测控制器》系列](https://www.bilibili.com/video/BV1cL411n7KV/)** —— 22 集，41 万播放。
  基本概念 → 无约束线性 MPC → **有约束线性 MPC**。最后那部分是重点：
  约束是 MPC 相对 LQR 唯一的实质增量，也是判断该不该上 TinyMPC 的依据。

**搜索关键词**
- 模型预测控制 / Model Predictive Control
- 滚动时域控制 / Receding Horizon Control
- 预测时域 vs 控制时域 / prediction horizon vs control horizon
- 终端代价 / terminal cost、终端约束 / terminal constraint
- 递归可行性 / recursive feasibility
- MPC 稳定性 / stability of MPC
- condensed vs sparse MPC formulation

**学会的判据**：能手推出「把 MPC 问题写成一个标准 QP」的过程；
能解释**为什么终端代价取 LQR 的 `Pinf` 能保证稳定性**（这条直接对应 TinyMPC 的 cache）。

**视频**：[【MPC模型预测控制器】1_最优化控制和基本概念](https://www.bilibili.com/video/BV1cL411n7KV/)
（DR_CAN，41万播放）。整个系列 22 集：基本概念 → 无约束线性 MPC → **有约束线性 MPC**。
最后那部分是你的重点——约束是 MPC 相对 LQR 唯一的实质增量，也是判断该不该上 TinyMPC 的依据。

**重点**：不要一上来钻数学。先搞懂 **N（horizon）怎么选**——太短会短视撞约束，太长算不动。
这是实践中最影响成败的一个参数。

---

### 第 3 层：ADMM（**第 4 周**，凸优化基础已砍）

**为什么**：TinyMPC 是个 ADMM 求解器。不懂 ADMM 就调不了它的 `rho`，也看不懂它为什么迭代次数会变。

**中文资源现状**：B 站确实没有成体系的 ADMM 视频课，
但**知乎的凸优化笔记系列质量不错，足够打底**：

- [【凸优化笔记7】交替方向乘子法（ADMM）](https://zhuanlan.zhihu.com/p/106896627) —— 放在凸优化体系里讲，脉络清楚
- [交替方向乘子法(ADMM)](https://zhuanlan.zhihu.com/p/613061477) —— 推导较完整
- [凸优化-交替方向乘子法（ADMM）](https://zhuanlan.zhihu.com/p/332243047)
- [凸优化笔记整理（B）——再看 ADMM、Frank-Wolfe](https://zhuanlan.zhihu.com/p/267542995) —— 进阶回看

**建议路径**：先用上面知乎文章建立直觉（1~2 天），再读 **Boyd 2011 综述前 3 章**定细节。
先看中文再看英文比直接啃论文省时间，但**别只看中文就收工**——
调 `rho`、看懂 TinyMPC 迭代次数为什么变，这些细节还是原文讲得准。

**搜索关键词**
- 凸优化 公开课（B 站搜「凸优化」有多所高校课程搬运）
- 凸优化 / convex optimization（Boyd 的书前 5 章足够）
- 二次规划 / Quadratic Programming / QP
- ADMM / Alternating Direction Method of Multipliers
- **Boyd 2011 "Distributed Optimization and Statistical Learning via ADMM"** ← ADMM 的标准入门，读前 3 章
- 原始残差 / 对偶残差 / primal residual, dual residual
- 罚参数 / penalty parameter rho、adaptive rho
- 投影算子 / projection onto a box / proximal operator

**学会的判据**：能解释 TinyMPC 每次迭代在干什么——
**一次 LQR 反向传播（无约束最优）+ 一次投影到约束集（拉回可行域）+ 更新对偶变量**，反复直到两者一致。
以及为什么 `rho` 大了收敛快但精度差。

> **这一层 B 站没有成体系的视频课**，只有[零散专栏](https://www.bilibili.com/read/cv8276482)。
> 只能啃 Boyd 2011 综述前 3 章，或看知乎的凸优化笔记系列。ADMM 本身不难，
> 难的是理解它在 MPC 里具体在干什么——那个得配合 TinyMPC 源码看，视频帮不上。

---

### 第 4 层：你这个问题特有的（与第 2 周并行）

#### 4.1 模态解耦 ✅ 已完成

**为什么**：直接解释你的 PID 打架，而且是不用 MPC 就能修的。

**搜索关键词**
- 共模 / 差模分解、common mode / differential mode
- 模态解耦 / modal decomposition
- 解耦控制 / decoupled control
- 对角化 / diagonalization、坐标变换

**要点**：把 `(L0_左, L0_右)` 换成 `((L0_左+L0_右)/2, L0_左−L0_右)`，
前者是车身高度、后者是横滚，两个环天然正交。

#### 4.2 参数标定（**已砍到只剩两件**，与第 2 周并行）

**为什么**：这是你最大的短板，也是 MPC 能否成功的前提。

**搜索关键词**
- 系统辨识 / System Identification
- 参数估计 / parameter estimation、最小二乘辨识 / least squares
- 频率响应辨识 / frequency response、扫频 / chirp / swept sine
- **转动惯量测量**：三线摆法 / trifilar pendulum、双线摆 / bifilar
- **平行轴定理** / parallel axis theorem ← 你的 `I_z` 就是漏了这个
- 质心测量 / center of mass measurement、悬挂法
- 模型验证 / model validation、residual analysis

**中文视频**
- B 站搜「系统辨识」「最小二乘辨识」有多所高校课程搬运
- 卡尔曼滤波与组合导航原理（西北工业大学 严恭敏）—— 讲状态估计基础，和 LESO 是近邻

> 转动惯量这块更依赖动手而非看视频。搜「三线摆法 测转动惯量 实验」能找到大学物理实验教程，够用。

**学会的判据**：能拿实测数据画出「模型预测 vs 实际响应」的对比曲线，并说出误差有多大。
**这条做完，就算不上 MPC，你现在的 LQR 也会明显变好。**

---

### 第 5 层：嵌入式实时（真要上车时再学，第 4 周末）

**搜索关键词**
- 实时迭代 / Real-Time Iteration scheme (RTI)
- 定迭代次数 MPC / fixed-iteration MPC
- 热启动 / warm start
- 嵌入式 MPC 代码生成 / embedded MPC code generation
- 最坏执行时间 / WCET / worst-case execution time

**要点**：ADMM 迭代次数随数据变化 → 1 kHz 硬实时环里必须钉死上限，用**次优但确定**换**最优但不确定**。

---

## 四、观看/阅读路径

日程见「二·五 四周日程」。这里只给顺序：

```
DR_CAN《Advanced控制理论》3.5 连续系统离散化   ← 10分钟
      ↓
【现代控制理论】一题掌握！状态观测器           ← 观测器 + 极点配置
      ↓
Han 2009 (7页) → Gao 2003 (6页) → arXiv 2104.01943
      ↓  ← 到这里就可以动手做 LESO 了（第 2 周）
DR_CAN《最优控制》5、6（LQR 推导 + 代码）
      ↓
DR_CAN《MPC模型预测控制器》1~22（重点看有约束那几集）+ TinyMPC 论文
      ↓
Boyd 2011 ADMM 综述前 3 章
      ↓
主机跑通 TinyMPC，量迭代次数和耗时 → 决定上不上车
```

每一层都可以**先用知乎/CSDN 文章建直觉再看论文**（见「六、推荐资源 → 中文文章」）。

**注意第四步那个分叉**：看完前三步就能着手 LESO，不必等 MPC 学完。
LESO 是下一步实际要做的，MPC 还在后面。

**LESO 做完可能就发现 MPC 想解决的问题小了很多。** 这不是劝退，
是让你在一个"误差已被实时补偿"的系统上再决定要不要 MPC。

---

## 四·五、C++/Eigen 路线要过的坎（已选定这条路）

决定是**引入 C++ 和 Eigen**：CMake 加 `enable_language(CXX)`，直接编 TinyMPC codegen 产物。
下面两个坑是读源码核实过的，动手前要知道。

### 坑 1：codegen 产物是**动态尺寸** Eigen 矩阵，会走堆分配

`types.hpp` 里是 `typedef Matrix<tinytype, Dynamic, Dynamic> tinyMatrix;`，
codegen 生成的也是 `(tinyMatrix(4,4) << ...)`（`codegen.cpp:218-242`）。
那几行定尺寸 typedef（`Matrix<tinytype, NSTATES, 1>` 之类）**是注释掉的，codegen 不用**。

后果：矩阵存在堆上。`tiny_setup()` 里一次性分配尚可接受，
**真正的风险是求解循环内 Eigen 表达式求值产生临时对象**——那会在 100 Hz 环里反复 malloc，
表现为偶发的周期抖动，而且很难查。

**验证手段（必须学会，主机阶段就能暴露）**：

```cpp
#define EIGEN_RUNTIME_NO_MALLOC          // 编译期打开检测开关

Eigen::internal::set_is_malloc_allowed(false);   // 求解前禁止分配
tiny_solve(solver);
Eigen::internal::set_is_malloc_allowed(true);    // 求解后恢复
```

循环内一旦发生分配会直接断言失败。确认有分配，就把 `tinyMatrix` 改成定尺寸模板
（`Matrix<tinytype, 4, 4>` 等）——`nx=4, nu=2` 全是编译期常量，改起来不难。

### 坑 2：`tinytype = double`

H723 有双精度 FPU，这个规模大概率无所谓。实测超预算再考虑换 float，
但注意 codegen 的注释写着"should be double if you want to generate code"。

### 嵌入式 C++ 卫生

引入 C++ 后编译选项要收紧，否则代码体积和不确定性都会失控：

- `-fno-exceptions` —— 关异常。Eigen 在禁用异常时会退化成断言
- `-fno-rtti` —— 关运行时类型信息
- `-fno-threadsafe-statics` —— 函数内 static 初始化不加锁
- 不在控制环里 new/delete；所有对象在初始化阶段构造完毕
- 注意全局对象的构造顺序（static initialization order），别依赖跨编译单元的构造次序

**搜索关键词**：`embedded C++ no exceptions RTTI`、`-fno-exceptions -fno-rtti`、
`EIGEN_RUNTIME_NO_MALLOC`、`Eigen fixed-size vs dynamic`、`static initialization order fiasco`

**材料**：Eigen 官方 [Quick Reference Guide](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)
和 [Preprocessor Directives](https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html)
（讲各种 `EIGEN_*` 宏，包括禁用动态分配和向量化）。
重点看「fixed vs dynamic size」和「表达式模板与临时对象」两节，其余可跳。

---

## 四·六、TinyMPC 库本身怎么上手

**中文资料：没有。** 搜下来一篇像样的中文介绍都没有，这块只能看官方英文。
好在官方文档质量高、例子完整，不难啃。

### 官方资源（第一手，优先看这些）

| 资源 | 用途 |
|---|---|
| **[tinympc.org](https://tinympc.org/)** | 官网首页，先看这里的概览 |
| **[Installation](https://tinympc.org/get-started/installation/)** | 装 Python 接口和 C++ 库 |
| **[Examples](https://tinympc.org/get-started/examples/)** | **最该先跑的**：官方例子，照着改成你的 4 状态腿模型 |
| [GitHub: TinyMPC/TinyMPC](https://github.com/TinyMPC/TinyMPC) | 源码，MIT 协议 |

### 关键工作流：先 Python 原型，再生成 C++

官方提供 **Python 接口**，而且它能**直接生成 C++ 代码和配套 python 模块**——
意思是你可以在 PC 上快速试参数、量迭代次数，确认可行了再把生成的 C++ 集成进固件。

**这正是第 4 周「主机跑通 TinyMPC 量耗时」该走的路**，不必一上来就啃 C++ 源码：

```
Python 接口搭 4 状态腿模型 → 调 Q/R/rho/N，看迭代次数和求解时间
        ↓  确认可行
codegen 出 C++ + 数据文件 → 编进固件（配合「四·五」那节的 Eigen 注意事项）
```

集成时的入口：`tiny_main.cpp` 调用 `tiny_solve()`，
但真正该用的是 `tiny_api.hpp` 里那些便利函数
（`tiny_set_x0()` 设当前状态、`tiny_set_x_ref()` 设参考轨迹等）。
官方明确建议以此为集成起点。

### 论文（配 Zotero 读）

| 论文 | 说明 |
|---|---|
| **[TinyMPC: Model-Predictive Control on Resource-Constrained Microcontrollers](https://github.com/TinyMPC/TinyMPC)**（ICRA 2024） | **原始论文，ICRA 2024 自动化方向最佳论文**。作者 Nguyen, Schoedel, Alavilli, Plancher, Manchester（CMU）。讲清楚它怎么利用 MPC 结构 + ADMM 做到又快又省内存 |
| **[Conic-TinyMPC: Code Generation and Conic Constraints](https://arxiv.org/html/2403.18149v2)** | **代码生成那部分看这篇**，你要用 codegen 就该读。另有 [CDC24 版 PDF](https://rexlab.ri.cmu.edu/papers/tinympc_cdc24.pdf) |
| [视频讲解](https://www.youtube.com/watch?v=NKOrRyhcr6w) | 作者自己讲，比读论文快，适合先过一遍建立框架 |

### 一个参考数据

官方称在单片机 benchmark 上**比 OSQP 最高快 8 倍且内存占用小得多**。
你的规模（`nx=4, nu=2, N=15`）远小于它的 benchmark，所以 100 Hz 有充足余量——
但**这个结论必须你自己在主机上量过才算数**，别直接引用官方数字上车。

---

## 五、已知的三个集成障碍（学的时候心里有数）

1. **语言**：你的工程是 `enable_language(C ASM)`，全仓库 0 个 `.cpp`。
   TinyMPC 的 codegen 产出 `.cpp/.hpp` 且依赖 Eigen。
2. **实时性**：ADMM 迭代次数数据相关，执行时间不定。
3. **数值类型**：`tinytype = double`。H723 有双精度 FPU，但比单精度慢。

---

## 六、推荐资源（中文优先）

### 视频 —— B 站

| 主题 | 资源 |
|---|---|
| **离散化** | DR_CAN《Advanced 控制理论》[3.5 连续系统离散化](https://space.bilibili.com/230105574/channel/seriesdetail?sid=1569601)（整个合集也覆盖状态空间、能控能观、李雅普诺夫） |
| **观测器 / 极点配置** | [【现代控制理论】一题掌握！状态观测器](https://www.bilibili.com/video/BV19K1vBNEsx/) |
| **LESO / ADRC** | [天津大学《控制理论基础2025》14 从PID到自抗扰控制ADRC](https://www.bilibili.com/video/BV1DACTB1E2r/) |
| **现代控制理论速成** | [《现代控制理论》期末速成课 6 小时](https://www.bilibili.com/video/BV1gu411Y7f8/)（赶进度时用，带讲义） |
| **LQR** | DR_CAN《最优控制》[5 数学推导](https://www.bilibili.com/video/BV1dm4y177tA/)、[6 案例+代码](https://www.bilibili.com/video/BV11V4y1t7z7/) |
| **MPC** | [DR_CAN《MPC模型预测控制器》系列](https://www.bilibili.com/video/BV1cL411n7KV/)，22 集 |
| 凸优化 | B 站搜「凸优化」有多所高校课程搬运（中科大、清华等） |
| 系统辨识 | B 站搜「系统辨识」「最小二乘辨识」；状态估计可看西北工业大学 严恭敏《卡尔曼滤波与组合导航原理》 |

### 中文文章 —— 知乎 / CSDN

文章比视频快，适合先建直觉再看论文定细节。**质量参差是常态，下面这些是搜索时挑出来的。**

| 主题 | 文章 |
|---|---|
| **MPC → QP（最对口）** | [Lecture 9 Convex 模型预测控制（MPC）](https://zhuanlan.zhihu.com/p/629148064) —— 讲 QP 转化和 **KKT Hessian 稀疏性加速**，正是 TinyMPC 的思路 |
| MPC 机器人语境 | [MPC在机器人中的应用（一）](https://zhuanlan.zhihu.com/p/407911153) |
| MPC 推导 + 代码 | [学习 MPC 原理与数学推导](https://zhuanlan.zhihu.com/p/2012626044969059405)（含 MATLAB）、[MPC 原理及理论推导](https://zhuanlan.zhihu.com/p/698526965) |
| MPC 代价函数→QP | [模型预测控制（MPC）原理及详细推导](https://blog.csdn.net/weixin_43487974/article/details/126352999)（CSDN） |
| **ADRC / LESO** | [自抗扰控制-ADRC](https://zhuanlan.zhihu.com/p/664345718)、[ADRC 在电机速度环中的应用](https://zhuanlan.zhihu.com/p/692439642) |
| **ESO 专题** | [【自抗扰控制ADRC】扩张观测器ESO](https://blog.csdn.net/m0_37835056/article/details/130541998)（CSDN，带 Simulink）、[ADRC—扩展状态观测器](https://blog.csdn.net/itnerd/article/details/104426955) |
| **LADRC 专项** | [【LADRC】线性自抗扰控制](https://blog.csdn.net/weixin_41276397/article/details/127353049)（覆盖最全）、[LADRC结构学习与实践](https://blog.csdn.net/qq_38169460/article/details/97243120)、[LADRC 实战：从原理到代码](https://www.cnblogs.com/ljbguanli/p/19928796) |
| **中文期刊（正经综述）** | [自抗扰控制技术及其工程应用综述](https://html.rhhz.net/tis/html/201711029.htm)（智能系统学报）—— 一次看清 ADRC 全貌 |
| **PID→LADRC 参数转化** | [基于PID参数整定的线性自抗扰控制参数整定](http://kzyjc.alljournals.cn/html/2021/7/20210706.htm)（控制与决策）—— **能复用你已调好的 PID 参数** |
| **ADMM** | [【凸优化笔记7】ADMM](https://zhuanlan.zhihu.com/p/106896627)、[交替方向乘子法(ADMM)](https://zhuanlan.zhihu.com/p/613061477)、[凸优化-ADMM](https://zhuanlan.zhihu.com/p/332243047)、[笔记整理（B）再看 ADMM](https://zhuanlan.zhihu.com/p/267542995) |
| LQR 手推 | [DR_CAN《控制之美：卷2》学习笔记——LQR手写推导](https://zhuanlan.zhihu.com/p/670056933) |
| 控制教程（在线书） | [第10章：模型预测控制（MPC）](https://zsc.github.io/control_tutorial/html/chapter10.html) |

> **用法**：中文文章建直觉 → 英文论文定细节。
> 两处**不要只看中文就收工**：① LESO 的带宽参数化整定（以 Gao 2003 为准）；
> ② ADMM 的 `rho` 与收敛行为（以 Boyd 2011 为准）。中文二手材料在这两处普遍讲不清。

### 中文书

| 主题 | 资源 |
|---|---|
| LQR + MPC | **DR_CAN《控制之美》卷 2** —— 覆盖 LQR 和 MPC，比啃英文原版轻松得多，适合先建框架（[LQR 推导笔记](https://zhuanlan.zhihu.com/p/670056933)） |
| ADRC / LESO | 韩京清《自抗扰控制技术》—— LESO 的原始出处，理论完整但偏难，可当参考书查 |

### 英文论文（有 Zotero 就优先读这些——都短，且没有等价中文材料）

| 主题 | 资源 | 篇幅 | 为什么值得 |
|---|---|---|---|
| **LESO 原理** | [Han 2009《From PID to ADRC》](https://mechatronics.ucmerced.edu/sites/g/files/ufvvjh1226/f/page/documents/04796887_0.pdf) | 7 页 | ADRC 之父亲笔，读它比看几小时视频快 |
| **LESO 整定** | [Gao 2003《Scaling and Bandwidth-Parameterization》](http://congres.cran.univ-lorraine.fr/2003/ACC%202003/Papers/FP03-3.PDF) | 6 页 | 只调一个 `ω_o`，不用逐个配极点。整定的关键 |
| **LESO 嵌入式实现** | [arXiv 2104.01943《Minimum-Footprint Discrete-Time ADRC》](https://arxiv.org/pdf/2104.01943) | 短 | 直接讲 MCU 上怎么写离散 LESO |
| LESO 变体备查 | [arXiv 2211.07309《Tuning and Implementation Variants of Discrete-Time ADRC》](https://arxiv.org/pdf/2211.07309) | — | 卡住时查 |
| **ADMM** | Boyd 2011《Distributed Optimization ... via ADMM》前 3 章 | 3 章 | 知乎笔记够打底，但 `rho` 与收敛细节以原文为准 |
| **TinyMPC 官方** | [tinympc.org](https://tinympc.org/) + [Examples](https://tinympc.org/get-started/examples/) | — | 文档质量高，**中文完全没有对应资料**，只能看这个 |
| **TinyMPC 原论文** | ICRA 2024（[GitHub 有链接](https://github.com/TinyMPC/TinyMPC)） | — | ICRA 2024 自动化方向最佳论文，讲清结构化 ADMM 为什么快 |
| **TinyMPC codegen** | [Conic-TinyMPC](https://arxiv.org/html/2403.18149v2) | — | 要用代码生成就得读这篇 |

### 工程类（C++/Eigen 路线专用）

| 主题 | 材料 | 什么时候看 |
|---|---|---|
| Eigen 速查 | [Quick Reference Guide](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html) | 第 4 周，只看「fixed vs dynamic size」和表达式模板两节 |
| Eigen 宏开关 | [Preprocessor Directives](https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html) | 同上，重点 `EIGEN_RUNTIME_NO_MALLOC`、`EIGEN_DONT_VECTORIZE` |
| 嵌入式 C++ | 搜索词：`embedded C++ no exceptions RTTI`、`-fno-exceptions -fno-rtti`、`static initialization order fiasco` | 改 CMake 之前 |

### 第二个月才需要的

| 主题 | 搜索词 |
|---|---|
| 接触状态切换 MPC | `hybrid MPC`、`contact-implicit MPC`、`mode switching MPC`、`multi-model MPC` |
| 力变化率约束 | `rate constraint MPC`、`input increment formulation`、`delta-u formulation`、状态增广 |
| 双速率控制 | `multi-rate control`、`dual-rate MPC`、外环慢内环快 |

> 已砍：Rawlings & Mayne 全书（700 页，一个月里读它是浪费）、
> Boyd《Convex Optimization》前 5 章（读 ADMM 综述不需要）、
> Ljung 系统辨识（LESO 做完后紧迫性大降）。要查再查，不排进日程。

---

## 七、一个建议的自测

学完第 1、2 层后，试着回答：

1. 我的 MPC 要施加哪几条约束？各自的物理依据是什么？
2. horizon N 取多少？依据是系统的什么时间常数？
3. 终端代价用什么？为什么这样能保证稳定？
4. 单次求解最坏要多少次 ADMM 迭代？1 kHz 够不够？
5. 如果模型质量偏差 20%，我的 MPC 会怎么表现？

**这五问答得上来，就可以动手了。** 答不上来，说明还差在哪一层是清楚的。
