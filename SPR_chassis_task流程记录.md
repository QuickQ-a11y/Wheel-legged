# SPR chassis_task 流程记录

本文档记录 SPR 参考工程中 `chassis_task` 的控制流程，并作为后续重排本工程 `chassis_task.c` 的依据。

参考工程路径：

```text
E:\AWL_WORK\Wheel-legged\Others\SPR Balanced-infantry\SPR Balanced-infantry\code\chassis\chassis
```

重点参考文件：

```text
Task/chassis_task.c
application/chassis_behaviour.c
```

## 1. 总体任务循环

SPR 的 `chassis_task()` 在初始化和等待电机在线后，进入 1 ms 周期循环。

主循环顺序固定为：

```text
chassis_set_mode()
    -> chassis_feedback_update()
        -> chassis_set_contorl()
            -> chassis_control_loop()
                -> 电机命令发送和安全保护
```

各阶段职责：

| 阶段 | SPR 函数 | 主要职责 |
|---|---|---|
| 设置底盘控制模式 | `chassis_set_mode()` | 读取遥控/上层控制值，切换外层 `chassis_mode` |
| 底盘数据处理和赋值 | `chassis_feedback_update()` | 更新腿部几何、十个状态量、速度融合和支撑力观测量 |
| 底盘控制量设置 | `chassis_set_contorl()` | 根据模式设置腿长、速度、横滚目标、跳跃/上台阶标志 |
| 底盘控制循环 | `chassis_control_loop()` | 计算实际支撑力、K 矩阵、轮/腿力矩、VMC 和模式状态 |
| 电机命令下发 | `chassis_task()` 外层尾部 | 按安全条件发送轮电流，关节命令由发送任务使用最新 set 值 |

后续本工程复现时，任务层应保留这种阶段顺序。复杂计算仍放在底盘模块或算法层，避免把全部控制细节堆到任务文件。

## 2. 两层模式结构

SPR 不是单层模式。它同时使用外层 `chassis_mode` 和内层 `chassis_state`。

外层 `chassis_mode` 来自遥控或上层控制：

| 外层模式 | 含义 | 参考入口 |
|---|---|---|
| `CHASSIS_ZERO_FORCE` | 零力矩模式 | `chassis_behaviour_mode_set()` |
| `CHASSIS_FOLLOW_GIMBAL` | 正常跟随/平衡模式 | `chassis_behaviour_mode_set()` |
| `CHASSIS_TOP` | 小陀螺模式 | `chassis_behaviour_mode_set()` |
| `CHASSIS_SELF_SAVING` | 自救模式 | `chassis_behaviour_mode_set()` |
| `CHASSIS_VMC_TEXT` | VMC 测试模式 | 头文件中定义，主流程未作为重点使用 |

内层 `chassis_state` 决定 `control_loop` 里进入哪个控制函数：

| 内层状态 | 含义 | 当前复现优先级 |
|---|---|---|
| `STANDING` | 正常站立和平衡控制 | 第一阶段实现 |
| `ZERO_FORCE` | 所有电机零输出 | 第一阶段实现 |
| `FALLING_DOWN` | 倒地后的腿部姿态调整 | 第一阶段保留框架 |
| `FALLING_TO_STANDING` | 从倒地姿态回到站立准备姿态 | 第一阶段保留框架 |
| `PID_CONTROL_LEG_AND_WHEEL` | 小板凳模式，便于调试腿和轮 | 第一阶段实现或保留框架 |
| `CLIMB_THE_STAIRS` | 上台阶模式 | 后续实现 |

本工程后续应也保留这两层含义：外层模式面向遥控或上层指令，内层状态面向底盘自身控制阶段。

## 3. chassis_set_mode

`chassis_set_mode()` 只调用：

```c
chassis_behaviour_mode_set(&chassis_move);
```

实际模式切换在 `application/chassis_behaviour.c` 中完成。

SPR 使用 `TwoBoardControlGimbal.mode` 切换外层模式：

| `TwoBoardControlGimbal.mode` | SPR 行为 |
|---:|---|
| `0` | 切到 `CHASSIS_ZERO_FORCE` |
| `1` | 切到 `CHASSIS_FOLLOW_GIMBAL`，清跳跃和上台阶标志 |
| `2` | 切到 `CHASSIS_FOLLOW_GIMBAL` |
| `5` | 切到 `CHASSIS_FOLLOW_GIMBAL` |
| `6` | 切到 `CHASSIS_SELF_SAVING` |
| `7` | 切到 `CHASSIS_FOLLOW_GIMBAL`，允许上台阶标志 |
| `8` | 切到 `CHASSIS_TOP` |
| `9` | 切到 `CHASSIS_FOLLOW_GIMBAL`，置跳跃标志 |

当前工程还没有接入遥控/DBUS 或上层控制输入。后续复现本阶段时，需要先明确：

- 模式值来自哪一个设备或通信协议。
- 每个模式值对应哪个 `chassis_mode_t`。
- 速度、腿长档位、roll/yaw 指令分别来自哪里。

## 4. chassis_feedback_update

`chassis_feedback_update()` 是 SPR 的反馈处理阶段。它不直接做最终控制输出，而是把传感器和电机反馈转换为控制器要用的状态。

主要步骤：

1. 读取四个髋关节电机角度，转换成左右腿主动杆角度。
2. 调用 `leg_position()` 计算左右腿长 `leg_L` 和腿角 `leg_phi0`。
3. 读取四个髋关节电机速度，调用 `leg_speed()` 计算腿长速度和腿角速度。
4. 组合 IMU 俯仰角和腿角，得到腿部倒立摆状态。
5. 调用 `chassis_speed_calc()` 估算机体前进速度。
6. 调用 `Kalman_Filter_Update()` 融合速度和加速度。
7. 更新 `kf_x` 和 `kf_dx`。
8. 调用 `chassis_Fn_calc()` 计算支撑力观测量。

SPR 中的主要状态量：

```text
left_theta
left_d_theta
right_theta
right_d_theta
theta
d_theta
phi
d_phi
kf_x
kf_dx
```

对应本工程当前 LQR 状态量时，需要注意顺序并不完全等同。当前工程的状态顺序是：

```text
s
dot_s
fai
dot_fai
theta_l
dot_theta_l
theta_r
dot_theta_r
theta_b
dot_theta_b
```

SPR 的 `chassis_Fn_calc()` 计算的是离地或接触判断用的支撑力观测量 `Fn`。它和 `chassis_get_F()` 中用于 VMC 的实际虚拟支撑力 `F` 不是同一个量。

当前工程已按这个数据流接入速度融合：

```text
wheel + leg geometry -> rawForwardVelocityMps
IMU motionAccMps2    -> forwardAccelerationMps2
[rawForwardVelocityMps, forwardAccelerationMps2]
    -> Algorithm_Kalman_Update()
        -> controlState[s, dot_s]
```

当前卡尔曼状态为：

```text
x = [dot_s, ddot_s]
z = [轮腿几何原始速度, IMU 前向运动加速度]
F = [1, dt;
     0, 1]
H = [1, 0;
     0, 1]
P = [1, 0;
     0, 1]
Q = [0.1, 0;
     0, 0.1]
R = [100, 0;
     0, 1.0e12]
```

这些参数集中在 `chassis_config.c` 的 `speed_kalman` 中。前向加速度轴向由 `imu.forward_accel_axis` 和 `imu.forward_accel_scale` 配置。零力矩、故障和安全输出路径会清空速度融合状态，等价于 SPR 在零力矩下清 `kf_x/kf_dx`。

第一阶段复现时建议：

- 保留“反馈更新”这个独立阶段。
- 先把现有腿部几何、IMU、轮速和 LQR 状态构造移动到该阶段对应的模块接口。
- 速度融合已经接入公共 `Kalman`，后续只调整模型配置中的 P/Q/R 和 IMU 前向轴向映射。
- `Fn` 观测量先记录到 debug，不直接参与控制闭环。

## 5. chassis_set_contorl

SPR 原函数名为 `chassis_set_contorl()`，名字有拼写问题，但流程含义是“设置本轮控制目标”。

它只调用：

```c
chassis_behaviour_control_set(&chassis_move);
```

该阶段根据外层 `chassis_mode` 设置：

- 总目标腿长 `L_set`。
- 左右腿目标腿长 `leg_L_set`。
- 目标前进速度 `vx_set`。
- 小陀螺角速度或方向目标 `wz_set`。
- roll 目标和左右腿长差补偿。
- 自救、跳跃、上台阶等标志。

第一阶段复现时，本工程应先实现这些最小目标量：

| 目标量 | 用途 |
|---|---|
| 目标腿长 | 腿长 PID 计算支撑力 |
| 目标前进速度 | LQR 状态目标或速度控制目标 |
| 目标 yaw/yaw rate | 后续轮差或小陀螺控制 |
| 目标 roll | 左右支撑力差补偿 |
| 内层状态 | 控制循环进入站立、零力矩、倒地或小板凳 |

## 6. chassis_control_loop

`chassis_control_loop()` 是 SPR 的控制核心。它的顺序是：

```text
计算支撑力 F
    -> 按腿长生成左右腿 K 矩阵
        -> 计算 yaw 转向力矩
            -> 根据外层模式和内层状态分发控制函数
                -> 得到轮电流和关节力矩命令
```

### 6.1 计算实际支撑力 F

SPR 通过 `chassis_get_F()` 计算左右腿 VMC 使用的支撑力。

支撑力组成包括：

- 腿长 PID 输出。
- 支撑力前馈。
- roll 补偿。
- 跳跃力。
- 气弹簧补偿。
- 离心力补偿。

第一阶段只做前三项：

```text
F = 腿长 PID + 前馈 + roll 补偿
```

暂不接入：

- 跳跃力。
- 气弹簧补偿。
- 南科离心力补偿。

这些补偿后续应独立配置，不能把参数硬编码进任务层。

### 6.2 按腿长生成 K 矩阵

SPR 使用：

```c
LQR_K(Leg_L, chassis_move.Left_Leg.K);
LQR_K(Leg_L, chassis_move.Right_Leg.K);
```

`Leg_L` 是左右腿长平均值：

```text
Leg_L = (left_leg_L + right_leg_L) / 2
```

当前工程使用 `chassis.lqr_k` 保存本轮 K 矩阵，并支持左右腿长二元拟合：

```text
chassis_config.lqr.coefficients -> Algorithm_LQR_FitLqrKPoly22() -> chassis.lqr_k[4][10]
```

后续扩展时，应继续使用当前工程的 `lqr_k` 链路，不回退到 SPR 的一维 `K[12]` 数组写法。

### 6.3 模式状态分发

SPR 只在外层模式为 `CHASSIS_FOLLOW_GIMBAL` 或 `CHASSIS_TOP` 时进入主要状态机。

状态分发逻辑：

```text
STANDING
    -> chassis_stay_STANDING()

ZERO_FORCE
    -> chassis_stay_ZERO_FORCE()

FALLING_DOWN
    -> 设置倒地恢复目标腿姿态
    -> chassis_stay_FAILING()

FALLING_TO_STANDING
    -> 设置小板凳姿态
    -> chassis_stay_FAILING()

PID_CONTROL_LEG_AND_WHEEL
    -> chassis_stay_PID_CONTROL_LEG_AND_WHEEL()

CLIMB_THE_STAIRS
    -> chassis_stay_CLIMB_THE_STAIRS()
```

第一阶段先复现：

- 正常站立。
- 零力矩。
- 倒地。
- 重新站立框架。
- 小板凳模式。

上台阶先只保留状态和文档，不接控制逻辑。

### 6.4 正常站立输出

`chassis_stay_STANDING()` 做这些事：

1. 根据腿长档位设置俯仰平衡偏置 `pitchToBadyBalance`。
2. 用 K 矩阵计算左右轮 `Tp`。
3. 用 K 矩阵计算左右腿摆 `Tp`。
4. 对轮力矩限幅。
5. 将轮力矩转换为轮电机电流。
6. 计算双腿协同 PD，防止左右腿劈叉。
7. 执行 KNN 离地检测。
8. 根据离地结果重算腿摆力矩或轮电流。
9. 调用 `leg_force()` 把虚拟支撑力和腿摆力矩映射到关节力矩。

第一阶段建议：

- 保留 K 矩阵计算轮/腿广义力矩。
- 保留轮力矩限幅和力矩转电流接口。
- 保留双腿协同 PD。
- KNN 离地检测先不实现，只预留 debug 和接口说明。

### 6.5 零力矩输出

`chassis_stay_ZERO_FORCE()` 直接清零：

- 左右轮电流。
- 四个关节电机腿部力矩目标。

本工程必须继续保留安全零输出链路。任何未实现模式、掉线、未使能、配置错误都应回到零输出。

### 6.6 倒地和重新站立

SPR 的倒地恢复不是 LQR 平衡控制，而是腿部位置控制：

- 通过 `LegToJoint()` 把目标腿长和目标腿角逆解到关节目标角。
- 关节使用角度环和速度环 PID。
- 轮电机电流保持 0。

第一阶段可先保留状态框架，不立即打开非零输出。真正启用前需要确认：

- 倒地判据。
- 目标腿长。
- 目标腿角。
- 关节角度 PID 参数。
- 关节速度 PID 参数。
- DM 电机方向和限幅。

### 6.7 小板凳模式

SPR 的 `PID_CONTROL_LEG_AND_WHEEL` 用于调试腿和轮，不是主要比赛状态。

第一阶段复现目标：

- 提供独立模式入口。
- 腿长和腿角可以固定到安全目标。
- 轮输出默认保持 0 或很小限幅。
- 主要用于验证腿部几何、关节方向和 VMC 映射。

## 7. 电机 CMD 输出

SPR 在主循环尾部根据安全条件发送轮电机电流：

- 遥控掉线时，左右轮电流清零。
- `CHASSIS_ZERO_FORCE` 或 `CHASSIS_SELF_SAVING` 时，左右轮电流清零。
- 关键电机掉线时，左右轮电流清零。

关节电机命令由其它发送任务读取最新的关节目标值。

本工程当前结构是：

```text
chassis_task.c
    -> chassis_control_loop()
        -> chassis_vmc.c
            -> chassis.joint_torque_nm / wheel_current
                -> chassis_cmd_send()
```

最终 CMD 输出集中在任务层尾部统一下发。控制器只填写 `chassis` 的本轮命令，不直接发送 CAN。

## 8. 当前工程复现建议

建议后续改造顺序：

1. `chassis_task.c` 当前主循环已经实现为：

```text
设置模式
    -> 收集并处理反馈
        -> 设置控制目标
            -> control_loop
                -> 应用输出
```

2. 新增外层模式和内层状态的清晰边界。
3. 把遥控或上层控制输入转成目标腿长、速度、yaw、roll 和模式。
4. 保持 `chassis_control_loop()` 的 SPR 风格主顺序，后续新增能力时继续按支撑力、K 矩阵、LQR 输出、VMC、命令输出分段接入。
5. 保留当前 `chassis.lqr_k`、PID、VMC 和安全输出链路。
6. 第一阶段只接入正常站立、零力矩、倒地框架、小板凳模式。
7. 上台阶、跳跃、气弹簧、离心力、KNN 和功率限制后续再接。

不建议做的事：

- 不照搬 SPR 的多套全局变量；当前只保留唯一实际状态 `chassis`。
- 不把所有控制计算塞进 `chassis_task.c`。
- 不在任务层硬编码机械参数、IMU 方向、电机方向或 LQR 系数。
- 不在未确认实机方向和限幅前打开非零输出。

## 9. 后续需要提供的信息

后续要在当前工程复现这套流程，需要补充以下信息：

1. 遥控或上层控制输入来源。
2. 模式值到外层 `chassis_mode` 的映射。
3. 正常站立、零力矩、倒地恢复、小板凳模式的触发条件。
4. 速度目标、yaw 目标、roll 目标和腿长目标的来源。
5. Kalman 速度融合参数的实机调参结果，重点是 `Q`、`R`、速度积分限幅和 IMU 前向加速度轴向。
6. 倒地恢复和小板凳模式的目标腿长、目标腿角和 PID 参数。
7. 是否保留 SPR 的 KNN 离地检测，以及 KNN 输入和模型数据来源。
8. 轮电机力矩到电流的最终实机换算系数。
9. DM 髋关节电机输出模式和最终限幅。

以上信息明确后，再开始修改当前工程代码。
