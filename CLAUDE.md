# CLAUDE.md

整车级文档：仓库布局、共享协作规范、两块板通用的构建与测试机制、板间契约。
**板专属的内容在各自的 `Chassis/CLAUDE.md` 和 `Gimbal/CLAUDE.md`。**

> Claude Code 会加载 cwd 及其所有祖先目录的 `CLAUDE.md`。所以开 `Chassis/` 时，
> 本文件和 `Chassis/CLAUDE.md` 都会进上下文——**共享内容只在这里写一份，不要往板下复制**。
> 三对逐字节相同的副本就是这么来的，已于 2026-09-21 清理。

## Repository layout

This repo (`Wheel-legged`) is the **whole robot** — both boards live here:

- `Chassis/` — STM32H723 firmware for the five-bar-linkage wheel-legged balancing **chassis**. One CubeMX project; artifact is `Wheel-legged.elf`.
- `Gimbal/` — STM32H723 firmware for the **gimbal**, derived from the chassis. Two independent CubeMX projects: `Real/` (the real application) and `Debug/` (system identification). Each carries its own full `User_File/` copy — they are allowed to diverge, so **never assume a fix in one propagated to the other**.

Outside the repo, under the workspace root `../`:
- `Matlab/` — offline model derivation (`ABK_LQR.py` is the live one, `ABK_LQR.m` is stale; `VMC.m`). Not built.
- `Others/` — reference open-source projects (HERO_LEG, SPR, ZJU docs …). Read-only.

The two boards talk over a dedicated **FDCAN3** link (PD12/PD13): the receiver (**FS-iA10B**, currently on **S.BUS**; i-BUS and DR16/DBUS also selectable at compile time via `APP_REMOTE_BACKEND`) is on the gimbal board, which forwards remote input down to the chassis. Protocol is defined in each project's `User_File/1_Middleware/0_Common/app_config.h`.

`Chassis/CLAUDE.md` holds the **binding style/collaboration rules** (Chinese) — read it before touching any business code; `Gimbal/CLAUDE.md` is the same document. This file covers build/test mechanics and architecture. When the two conflict, `Chassis/CLAUDE.md` wins.

**Open one project at a time in the editor** — `Chassis/` or `Gimbal/Real/`, never the repo root. Each carries its own `.clangd` and `.vscode/`, and both use `${workspaceFolder}`-relative paths that only resolve when that project is the workspace root.

⚠ Each of the three CubeMX projects ships its own copy of `Drivers/` (~11 MB) and `Middlewares/` (~15 MB) — about **78 MB of duplicated ST HAL/CMSIS** in the repo. This is CubeMX's doing: it regenerates those paths per project. Deduplicating means fighting CubeMX's output layout for every regeneration. **Decided: leave it.** Don't propose sharing them again.

---

# 协作规范（两块板共用）

## 项目定位

RoboMaster 本科生竞赛实验工程（轮腿底盘 + 云台，STM32H723 + HAL + FreeRTOS），
长期实验迭代，非产品、不上市。
优先级：功能可跑、代码直白、易 Debug > 健壮性、可复用性。
不要用"产品级"标准写这份代码：不加多余的防护、不做过度封装、不为"未来扩展"预留设计。

## 修改代码前必读

1. `Codex文档/代码风格要求.md` —— 详细代码风格规范（命名、注释、物理符号、坐标系、
   USB/CAN 协议、安全红线），修改任何业务代码前必须阅读并遵守。**本文件是它的精简版**，
   自动进上下文；完整版按需读。两者冲突时以完整版为准。
2. 动哪块板就读哪块板的 `CLAUDE.md`：`Chassis/CLAUDE.md` / `Gimbal/CLAUDE.md`。
3. 上实机或改底盘参数前，读 `Chassis/Codex文档/实机调试检查清单.md`。
4. 参考 HERO_LEG 前先看 `Chassis/Codex文档/HERO_LEG参考要点.md`——那边源码是 GBK 编码，
   `grep` 不加 `-a` 会静默跳过，看起来就像符号不存在。
5. `Codex文档/handoff-*.md` 是一条按时间排的交接流，**每篇开头注明作用范围**
   （底盘 / 云台 / 跨板）。2026-09-21 之前的均为底盘范围。最新一篇通常是最有用的入口。

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
Chassis.lqr.target[CHASSIS_STATE_D_S] = goal.d_s;     /* 全局直访，Watch 可见 */
Motor_DM_GetState(i, &s);                              /* 跨层数据消费走接口 */
void Chassis_Control(void)                             /* 流程函数 void，无状态返回 */
Limit_Symmetric(v, lim)                                /* 算法本身需要的限幅保留 */
```

注意区分：控制算法**本身需要**的限幅、奇异判断、积分限位属于功能代码，保留；单纯"防止调用方传错"的防御性判断属于防护，删除。

## 重构工作流

1. 先重构 `User_File/3_Chariot/1_Module/Chassis` 试点，用户确认风格后再铺开到其他层。
2. 每个模块重构完必须构建验证：`cmake --build build/Debug`（Ninja + arm-none-eabi-gcc），零错误零警告才提交。
3. 涉及底盘逻辑时回归主机单测：`Tests/` 下 `test_chassis_*.c` 用系统 gcc 直接编译运行。
   仓库里**没有测试脚本或 CMake target**，编译命令、每个测试要链接的源文件、以及
   `test_motor_dm` 无法在主机编译、`test_chassis_recovery` 需要临时把三道输出门置 0
   这两个坑，都记在工作区根目录 `../CLAUDE.md` 的 "Host tests" 一节。
   （`build/host_tests/` 下只是历史产物，不是当前构建输出。）
4. 汇报格式（中文）：改了哪些文件、删了哪些防护（标注 `文件:行号`）、保留了什么、如何验证的。删除清单先列给用户确认，不要一次性大批量删完。

---

# 两块板通用的机制

## Build (firmware)

ARM GCC + CMake + Ninja, target `STM32H723xx`. **Run from `Chassis/`** (the gimbal builds the same way from `Gimbal/Real/` or `Gimbal/Debug/`).

**On this Linux machine the toolchain is not on `PATH`** — it lives in the STM32Cube bundles and must be prepended:

```bash
export PATH="$HOME/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:\
$HOME/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH"

cmake --preset Debug          # configure into build/Debug
cmake --build build/Debug     # -> build/Debug/Wheel-legged.elf
```

Zero warnings is the merge bar (`Chassis/CLAUDE.md` §重构工作流). Presets: `Debug`, `Release`.

New user `.c` files must be registered in `Chassis/CMakeLists.txt` under both `target_sources` and `target_include_directories` — CubeMX does not manage `User_File/`.

## Host tests

`Tests/*.c` are plain `assert()` programs compiled with the **system gcc**. There is no test
runner, CMake target, or script checked in — invoke gcc directly. **Two projects now carry
host tests**: `Chassis/Tests/` and `Gimbal/Real/Tests/`. All `-I` paths below are relative to the project root, so `cd` into it first.

**Always check gcc's exit code before running the binary.** A stale binary from a previous
test is otherwise reported as a pass — this has caused a false PASS more than once. Redirecting
gcc's stderr to `/dev/null` inside a loop is the usual way to hide the failure; don't.

各板的具体配方在各板的 `CLAUDE.md`。

## 板间协议契约

两板通过专用 **FDCAN3**（PD12/PD13）通信：接收机在云台板，遥控输入由云台向下转发给底盘。
两帧的字节布局见任一侧 `app_config.h` 的板间通信注释块；键位语义见
`Codex文档/遥控键位与模式分配.md`。

**架构：每块板只解释自己关心的拨杆，板间只传归一化后的原始快照。**
不要把 `Chassis_Mode_t` 解析到云台再下发——那是反向依赖，还会多出一份必须
两侧同步的协议常量。

⚠ FDCAN3 配的是 `FDCAN_FRAME_CLASSIC`，**帧长封顶 8 字节**，加字段只能在现有
8 个字节里挤。状态帧目前用了 7 个（4 个拨杆压在一个字节里），剩 1 个预留。

⚠ **协议常量在两侧的 `User_File/1_Middleware/0_Common/app_config.h` 里各存一份，
必然漂移——改一侧必须同步另一侧。** 已知的有意差异：

```
APP_BOARD_SEND_DIV    Chassis = 5U    Gimbal/Real = 1U
```

底盘那份是**死配置**（`Chassis/User_File/` 里引用次数为 0，底盘只收不发），
实际发送频率由云台那份决定（DIV=1 即 1 kHz，2000 帧/s）。这是用户有意设的，别改回去。

（`remote_input.h` 曾经也是漂移源，现已收敛成纯输入类型，两板逐字节相同——
派生语义移进了各板的业务层。`Chassis/User_File/1_Middleware/0_Common/remote_input.c`
已删除。）

改协议时用这条命令对一遍：

```bash
diff <(grep APP_BOARD_ Chassis/User_File/1_Middleware/0_Common/app_config.h) \
     <(grep APP_BOARD_ Gimbal/Real/User_File/1_Middleware/0_Common/app_config.h)
```

板间协议的唯一回归保障是 `Gimbal/Real/Tests/test_board_protocol.c`——它同时链两个工程的
`device_board.c`，改协议后必须跑它。

### DM MIT torque scale — the trap that hid for weeks

`APP_DM_TOR_MIN/MAX` must equal the **TMAX register flashed into the motor** (this robot:
**40**). The MIT frame carries torque as a dimensionless 12-bit integer that each side decodes
with its *own* range, so a mismatch scales both directions:

| firmware range | commands | feedback |
|---|---|---|
| ±15 | actual = commanded × **2.667** | read = actual × **0.375** |
| ±25 | actual = commanded × **1.600** | read = actual × **0.625** |
| ±40 | 1.000 | 1.000 |

**The two errors cancel exactly on the round trip** (`command 10 → actual 15.99 → read back
9.99`). Comparing `output.T_joint` against `dm_motor[].torque_nm` therefore always looks
correct and is the one check that cannot detect this. Use an absolute force reference instead:
hang a weighed mass on the wheel axle and check that `ΔChassis.ground.force[].F0` equals `m·g`.

Never shrink `APP_DM_TOR_MAX` to limit torque — that is `output.joint_T_limit`'s job, and that
field only means real N·m once this pair matches TMAX (before that it clamps *full scale*).

Consequence for tuning: K's **wheel rows go through the DJI ESCs and were always correct**,
while its **leg-swing rows go through DM and were scaled**, so historical tuning has a
wheel/leg-swing gain mismatch baked in.

**两块板都用 DM 电机、都走 MIT 编码，所以这个陷阱对两块板都成立。**
各板的实际量程和核对状态见各板的 `CLAUDE.md`。

## Conventions beyond the style doc

- Comments and commit messages are Chinese; match the surrounding language.
- Short physical symbols (`d_s`, `fai`, `theta_b`, `L0`, `F0`, `Tp`, `Fn`, `a_y`) with units in the field comment — do not encode units in names (`_mps`, `_radps` suffixes were deliberately removed).
- The FreeRTOS port is `ARM_CM4F` (CubeMX picked the M4F port for this M7 part).
- `Codex文档/` holds design/handover notes. They lag the code — when they disagree, the source wins.
