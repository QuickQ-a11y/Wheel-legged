# CLAUDE.md — Wheel-legged 工程协作规范

## 项目定位

RoboMaster 本科生竞赛实验工程（轮腿底盘，STM32H723 + HAL + FreeRTOS），长期实验迭代，非产品、不上市。
优先级：功能可跑、代码直白、易 Debug > 健壮性、可复用性。
不要用"产品级"标准写这份代码：不加多余的防护、不做过度封装、不为"未来扩展"预留设计。

## 修改代码前必读

1. `Codex文档/代码风格要求.md` —— 详细代码风格规范（命名、注释、物理符号、坐标系、USB/CAN 协议、安全红线），修改任何业务代码前必须阅读并遵守。
2. 重构底盘相关代码前，还应阅读 `Codex文档/SPR_chassis_task流程记录.md` 与 `Codex文档/三份底盘控制代码对比与实机完善清单.md`。

本文档与 `Codex文档/代码风格要求.md` 冲突时，以本文档为准（本文档是本轮重构期的额外要求）。

## 分层架构（严格单向依赖，上层只能调用下层）

```
5_Task                    chassis_task、task_can、task_imu、task_remote、task_usb、task_can_dispatch
   │                      调度、分发、模块连接；每轮按 反馈 -> 状态选择 -> 控制 -> 命令发送 执行
3_Chariot/1_Module        Chassis（config/control/observer/remote/vmc），功能模块
   │
2_Device                  BMI088、Motor_DM、Motor_DJI、DR16、USB 协议；解析并维护设备状态
   │
1_Middleware/2_Algorithm  PID、LQR、Kalman、QuaternionEKF、Angle、CRC（算法库）
1_Middleware/1_Driver     FDCAN、SPI、UART、USB（只做硬件收发，不写业务解析）
1_Middleware/0_Common     app_config.h、remote_input.h（公共配置与常量）
User_config               CubeMX 工程配置
```

- 依赖规则：上层只能调用下层；同层模块不互相调用；`0_Common` 可被任意层引用。
- 禁止：task 直接读写 device/module 的内部状态、module 直接调 HAL、device 反向调用上层、跨层硬编码常量（集中放 `app_config.h` 或 `chassis_config.c`）。

## 代码风格硬性要求

1. **全局变量与 Watch**：关键状态直接用全局变量暴露（如 `extern Chassis_t Chassis;`），字段带单位注释，方便添加到 Watch 窗口；不复制第二套 debug 结构体。模块内部跨周期的中间量允许 `static`/全局，但同一份事实只保留一个所有者。
2. **数据消费与调用规范**：全局变量只是调试入口；业务数据消费仍走模块公开接口（如 `Motor_DM_GetState()`、`Motor_DM_SetCommand()`、`CAN_Task_GetTxErrorCount()`），跨层访问必须通过函数接口，禁止裸跨层访问内部 `static` 状态。
3. **删除的防护（重构重点）**：
   - 入口参数校验：NULL/范围检查、防御性 `isfinite`/溢出判断；
   - 重复的 `Is`/`Get`/`Check` 查询函数、只消费一次的派生布尔状态（`bench_flag`、`output_flag` 之类）；
   - 统一 status/错误码返回值体系——流程型函数默认 `void`；
   - 一两行就能写完的简单表达式不包函数。
4. **保留的红线（实机安全，绝对不删）**：上电默认零输出、电机限幅与离线保护、数组边界、除零保护、腿长范围与几何奇异判断、`safe_flag` 安全门、fault 检测与最终命令清零、`Chassis.dt` 越界回退、IMU 温度保护。
5. **禁止**：新增任何封装层/接口层/工厂/回调注册；新增任何防御性代码；顺手重构未指定的模块；改变 task 层对外接口与通信协议（除非用户明确要求）。
6. **算法代码只去封装，不改数学**：LQR、PID、Kalman、VMC、五连杆运动学只做结构性简化，公式、状态顺序、参数值一律不动。

## 反面/正面写法对照（重构时照此执行）

反面写法（禁止出现，看到就删）：

```c
if (ptr == NULL) return;                 /* 参数校验：固定调用链已保证有效 */
if (!isfinite(v)) v = 0.0f;              /* 防御性判断：掩盖真实问题，还费分支 */
uint8_t ret = Func(); if (ret != OK) {}  /* 返回值检查：流程型函数用 void */
if (x > MAX) x = MAX;                    /* 冗余限幅：物理限位由电机驱动/机械保证 */
Chassis_Init(&cfg, 0);                   /* 句柄+配置传参：直接初始化全局 Chassis */
```

正面写法（照此写）：

```c
Chassis.lqr.target[CHASSIS_STATE_DOT_S] = goal.d_s;   /* 全局直访，Watch 可见 */
Motor_DM_GetState(i, &s);                              /* 跨层数据消费走接口 */
void Chassis_Control(void)                             /* 流程函数 void，无状态返回 */
Limit_Symmetric(v, lim)                                /* 算法本身需要的限幅保留 */
```

注意区分：控制算法**本身需要**的限幅、奇异判断、积分限位属于功能代码，保留；单纯"防止调用方传错"的防御性判断属于防护，删除。

## 重构工作流

1. 先重构 `User_File/3_Chariot/1_Module/Chassis` 试点，用户确认风格后再铺开到其他层。
2. 每个模块重构完必须构建验证：`cmake --build build/Debug`（Ninja + arm-none-eabi-gcc），零错误零警告才提交。
3. 涉及底盘逻辑时回归主机单测：`Tests/` 下 `test_chassis_*.c` 在 PC 上编译运行（构建产物在 `build/host_tests/`）。
4. 汇报格式（中文）：改了哪些文件、删了哪些防护（标注 `文件:行号`）、保留了什么、如何验证的。删除清单先列给用户确认，不要一次性大批量删完。
