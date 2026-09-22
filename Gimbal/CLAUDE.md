# CLAUDE.md — Gimbal（云台）

云台专属。**共享的协作规范、代码风格、构建/单测通则、板间协议、DM MIT 力矩标度陷阱
都在仓库根 `../CLAUDE.md`，那份会自动一起加载，本文件不重复。**

## 两套独立工程

- `Real/` —— 真实应用，产物 `Gimbal_Real.elf`
- `Debug/` —— 系统辨识用，产物 `Gimbal_Debug.elf`

**两者各带一份完整的 `User_File/` 副本，允许发散。⚠ 绝不要假设在一个里改的东西
自动传到了另一个——改之前先 `diff` 一遍对应文件。**

两者都自带 `.clangd` 和 `.vscode/`，用 `${workspaceFolder}` 相对寻址，
所以**编辑器要么开 `Real/` 要么开 `Debug/`**，别开 `Gimbal/` 这一级。

## 电机与力矩标度

- 2 台 **达妙 DM4310**，均挂 **FDCAN3**，与板间通信共用同一条物理总线。
- `APP_DM_TOR_MIN/MAX = ±10`。

⚠⚠ **这一对还没有跟电机 TMAX 寄存器实际核对过。** 底盘那边同样的问题藏了很久
（固件 ±15/±25 而寄存器是 40，命令被放大 2.67 倍），**而且往返相消、常规对比查不出来**。
排查方法和判据见根 `../CLAUDE.md` 的 "DM MIT torque scale" 一节——
核心是：**不能靠比对命令和反馈，必须用绝对力参照**。

## 遥控后端（DR16 / FS-iA10B 二选一）

两种接收机挂**同一路 UART5**，靠 `app_config.h` 的 **`APP_REMOTE_BACKEND`** 编译期二选一：

```c
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_DR16    /* DT7 + DR16 */
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_IA10B   /* FS-i6X + FS-iA10B，i-BUS */
```

换遥控只改这一个宏再重新烧录。后端差异全部收在 `task_remote.c` 顶部的一段编译期别名里
（`Remote_Backend_ParseFrame` / `_MakeRemote` / `REMOTE_FRAME_LEN` / `REMOTE_UART_*`），
函数体保持单一形态。

| | 帧长 | 波特率 | 字长/校验/停止位 | RX 反相 | 帧率 |
|---|---|---|---|---|---|
| DR16 (DBUS) | 18 | 100000 | 9B / EVEN / 1 | **关** | 71 Hz |
| IA10B (i-BUS) | 32 | 115200 | 8B / NONE / 1 | **开** | 129 Hz（实测） |

### ⚠ SBUS 座子带硬件反相器，两种后端的反相设置相反

板子手册 §6 那张表写 `1 | S | PC02 | 反相 GPIO`，两处都要注意：

- **引脚是 PD2 不是 PC02**——手册笔误。引脚图和 `.ioc`（`PD2.Signal=UART5_RX`；
  `PC2_C` 是 `SPI2_MISO`）都是 PD2。
- **座子上有反相器**，板子是按 S.BUS / DBUS 这类反相电平设计的。于是：
  - **DBUS 本身反相** → 被硬件翻正 → MCU 侧 **RXINV 关**
  - **i-BUS 是正常 TTL** → 被硬件翻反 → MCU 侧 **RXINV 开**，翻回来抵消

### ⚠ 判断极性对不对，要看 `rxEventCount`，不能看 `uartErrorCount`

`uartErrorCount` 在**两种极性下都会快速上涨**，拿它当判据必然被骗。有区分度的是空闲事件：

| 现象 | 含义 |
|---|---|
| `lastUartError` 固定 4(FE) + **`rxEventCount` 恒为 0** | 极性反了。线路一直被当成起始位，所以既报 FE 又**永远等不到空闲** |
| `rxEventCount` 按帧率上涨 + `lastRxSize` 稳定 32 | 极性和波特率都对，剩下的是解析层的事 |
| `rxEventCount` 慢速零星增长 + `lastRxSize` 在 0~4 且**随摇杆变化** | 线上是 **PPM/PWM 脉冲**，根本不是串口——去查接收机输出模式 |

### ⚠ 接收机输出模式必须设成 i-BUS

FS-i6X 的输出模式菜单是 **`PWM/i-BUS` 和 `PPM/S.BUS` 成对切换**的。停在 `PPM/S.BUS`
时，`Servo` 口吐的是 PPM 脉冲，固件怎么配 UART 都解不出来。切到 i-BUS 后**接收机要重新上电**。

⚠⚠ **踩过的坑，值得记：这两个故障（输出模式是 PPM + RXINV 没开）是叠加的。**
在 PPM 还没改的情况下试 RXINV，拿到的是**假阴性**，正确的假设因此被丢掉、绕了一大圈。
**一个假设的否定结论，只在其他故障都排除之后才成立。**

⚠ **UART 参数在 `Remote_Task_Init` 里按后端重新 `HAL_UART_Init`，不改 `Core/Src/usart.c`**
——那个文件 CubeMX 重新生成会冲掉。DR16 那组常量就是 CubeMX 现有值，对它是空操作。

⚠ **键鼠、`dialValid` 是 DR16 独有的**，`task_remote_state_t` 里那几个字段和相关逻辑
用 `#if` 守着，切到 i-BUS 后不存在——Watch 表达式要跟着改。

⚠ **i-BUS 的槽位数是协议固定的 14 个**，与遥控器实际通道数无关（i6X 只用前 10 个，
其余由接收机填中位）。别用槽位数去推断型号。

### 通道映射（2026-09-22 实机确认）

| 下标 | FlySky 编号 | 对应 | 说明 |
|---|---|---|---|
| 0 | CH1 | 右摇杆横 | 右增，中位 1500，量程 1000~2000 |
| 1 | CH2 | 右摇杆纵 | 上增 |
| 2 | CH3 | 左摇杆纵 | 上增 |
| 3 | CH4 | 左摇杆横 | 右增 |
| 4 | CH5 | VrA 旋钮 | 顺时针增 |
| 5 | CH6 | VrB 旋钮 | `Remote_t` 只有一个 dial，未用 |
| 6~9 | CH7~CH10 | SwA / SwB / SwC / SwD | 见下 |

四个轴都是"右增 / 上增"，与 `Remote_t` 的 "x 向右为正、y 向上为正" 同向，
所以 `IA10B_Axis` 直接减中位、**不加负号**；回中正好 1500，死区 `APP_IA10B_DB=8` 够用。

⚠ **出厂状态下 SwA~SwD 没有分配到任何通道**，拨动时所有通道都不动、未用槽停在 1500。
要在发射机 **系统菜单 → Aux. channels** 里把 SwA~SwD 依次分配到 CH7~CH10 才会出现。

### ⚠⚠ 拨杆是"上小下大"：UP=1000 / MID=1500 / DOWN=2000

和"值大 = 位置高"的直觉相反。`IA10B_ConvertSwitch` 因此是**低值判 UP、高值判 DOWN**，
不要"顺手改回来"——`test_ia10b.c` 的 `testSwitchThresholds` 专门钉死了这个方向，
改反了单测会红。

**方向搞反的后果是安全性的**：底盘右拨杆管急停/待命/运行，反了会让你以为在打急停、
实际在选运行。

⚠ **后续模式映射的既定约束：UP = 卸力。** 发射机要求所有杆位于 UP 才允许开机
（否则报警），所以 UP 是唯一保证的上电初始状态，必须对应最安全的模式。
这条也让通道值残留 0 的退化方向（落在 UP 一侧）正好是安全的。

### ⚠ i6X 默认只有一个三档开关

SwC 是三档，**SwA / SwB / SwD 都是两档**，挂在两档开关上的那一路读不出
`REMOTE_SWITCH_MID`——这是遥控器硬件限制，不是解析的 bug。

而底盘现有语义**需要两个三档**：右拨杆管"允不允许动"（急停/待命/运行），
左拨杆管"做什么"（STEP/FOLLOW/TOP）。三条出路，手册对应章节：

| 办法 | 手册 | 要拆机 | 能变出第二个三档？ |
|---|---|---|---|
| **改哪个开关驱动哪个通道** | §5.5 Aux. channels | 否 | ❌ 只能挪位置 |
| **物理换三档开关模块** | §9.1 开关分配 | 是 | ✅ 最干净 |
| 混控拼三档 | §5.9 Mixes | 否 | ⚠ 能但不建议 |

- **§5.5**：系统菜单 → `Aux. channels` → `OK` 选通道 → 分配开关/旋钮/`None` → 长按
  `CANCEL` 保存。手册明写"开关关闭时传播通道的较低值，打开时较高值"，
  **两档在菜单层面伪造不出中间档**。
- **§9.1**：开关是插在主板上的模块不是焊死的。拆后盖 → 拧松固定螺帽取出开关 →
  "接入主板上的任意接口并使用螺帽固定"。所以另配一个三档开关模块换上即可。
  手册提醒"确保所有开关朝同一方向安装"。
- **§5.9**：Master 只能选 `Ch1-Ch6, VrA, VrB`，**选不了开关默认所在的 Ch7-Ch10**，
  得先用 §5.5 挪过去；还要占两个物理开关换一个三档，且存在未定义的第四种组合，
  从固件侧看只是一个通道值在跳，排查很不直观。

**建议顺序：先别动遥控器。** 这一步目标只是收到数据，映射都还没确认，
现在改硬件是在没有数据的情况下做决定。先上电记下实际 AETR 顺序和四个开关的通道，
确认到底需要几个三档，再按上表选。过渡期可用 §5.5 把 SwC 挪到更需要三档的那一路
（急停），另一路先用两档凑合。

⚠ 别忘了发射机默认只开 **6** 个通道，要用 10 通道得先在 §7.13 Aux Switches 里改
（同一个菜单还负责启用/关闭各个开关旋钮，见 §9.2）。

### 主机单测配方

```bash
INC="-IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/Communication/DR16 -IUser_File/2_Device/Communication/USB \
-IUser_File/2_Device/Communication/IA10B"
```

| Test | Extra sources |
|---|---|
| `test_angle` | `2_Algorithm/Angle.c` |
| `test_dr16` | `DR16/device_dr16.c` |
| `test_ia10b` | `IA10B/device_ia10b.c` |
| `test_usb_protocol` | `USB/device_usb_protocol.c` `2_Algorithm/CRC.c` |
| `test_board_protocol` | see below — spans both projects |

`test_ia10b` 只需要 `remote_input.h`，不链 `remote_input.c`、不碰 HAL。
它用常量而不是写死下标表达映射，所以回填 `IA10B_CH_*` 之后仍然成立。

`test_board_protocol` packs with the gimbal's `device_board.c` and unpacks with the chassis's,
so it links **one file from each project**. Both projects have a `device_board.h` with the same
name and different contents, so the two `.c` files must be compiled in **separate gcc
invocations** with their own `-I`, then linked; a single invocation picks the wrong header.
It also needs three stub headers on the include path (`stm32h7xx_hal.h` declaring `HAL_GetTick`,
`fdcan.h` with a dummy `FDCAN_HandleTypeDef` + `hfdcan3`, `task_can.h` declaring
`CAN_Task_UpdateTxFrame`); the test file itself defines those symbols. The exact command is in
the comment block at the top of `Gimbal/Real/Tests/test_board_protocol.c`.

## 板专属文档

`Codex文档/` 目前为空，留给将来的云台专属文档。
共享文档和交接流在仓库根 `../Codex文档/`。
