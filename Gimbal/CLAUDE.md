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

## 遥控后端（DBUS / i-BUS / S.BUS 三选一）

三种协议挂**同一路 UART5**，靠 `app_config.h` 的 **`APP_REMOTE_BACKEND`** 编译期三选一：

```c
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_DR16   /* DT7 + DR16，DBUS */
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_IBUS   /* FS-i6X + FS-iA10B，i-BUS */
#define APP_REMOTE_BACKEND APP_REMOTE_BACKEND_SBUS   /* 同一台 FS-iA10B，S.BUS（当前默认） */
```

**IBUS 和 SBUS 用的是同一台接收机**，靠发射机输出模式菜单（`PWM/i-BUS` ↔ `PPM/S.BUS`）
切换，**切完接收机要重新上电**。换协议只改这一个宏再重新烧录。

后端差异全部收在 `task_remote.c` 顶部的一段编译期别名里
（`Remote_Backend_ParseFrame` / `_MakeRemote` / `REMOTE_FRAME_LEN` / `REMOTE_UART_*`），
函数体保持单一形态。设备层在 `2_Device/Communication/`：`DR16/`（DJI）和
`FlySky/`（`device_ibus` + `device_sbus`）。**目录按厂商分、文件按协议命名**——
同一台 FS-iA10B 说两种协议，按接收机型号命名会自相矛盾。

| | 帧长 | 波特率 | 字长/校验/停止位 | RX 反相 | 帧率 | 校验 |
|---|---|---|---|---|---|---|
| DR16 (DBUS) | 18 | 100000 | 9B / EVEN / **1** | **关** | 71 Hz | 无（靠帧长） |
| i-BUS (FS-iA10B) | 32 | 115200 | 8B / NONE / 1 | **开** | 129 Hz（实测） | **校验和** |
| S.BUS (FS-iA10B) | 25 | 100000 | 9B / EVEN / **2** | **关** | 143 / 71 Hz | 无（只有帧头） |

⚠ **S.BUS 和 DR16 的 UART 参数只差停止位**（2 vs 1），反相设置也一样。
电气上它俩同源，差别全在字节布局：DBUS 是 18 字节 DJI 自有打包，
S.BUS 是 25 字节 16×11 bit 位打包。

### ⚠⚠ S.BUS 没有校验和

i-BUS 有一个 `0xFFFF - 前 30 字节之和` 的真校验，**S.BUS 完全没有**。
这个后端能依靠的只剩三样：`task_remote` 先查的帧长 25、帧头 `0x0F`、以及 UART 偶校验。
坏帧漏进解析层的概率明显比 i-BUS 高。

补偿手段是标志字节 `frame[23]` 里的 **failsafe / frame-lost**，这是 S.BUS 唯一强过
i-BUS 的地方——接收机会主动报告它丢了发射机：

- `failsafe` 置位 → `task_remote.c` **立刻**判离线，不等 `APP_REMOTE_TIMEOUT_TICKS`。
  理由：接收机丢失发射机后仍按帧率持续发帧，只是通道值换成预设值（通常全部回中），
  光看帧来没来分辨不出来；等超时那 100 ms 里底盘会把"回中"当成操作者真松了杆。
- `frameLostCount` / `failsafeCount` 挂在 `task_remote_state_t` 上（`#if` 守着），
  **没有校验和之后，这两个计数就是判断链路好坏的主要依据**。

### ⚠⚠ S.BUS 的通道量程按本机实测，不是 Futaba 惯例值

**2026-09-24 实测（FS-iA10B）：`240 / 1024 / 1807`**，不是网上到处抄的
Futaba 惯例 `172 / 992 / 1811`。富斯的 us → 11bit 换算系数和 Futaba 不同：

| | MIN / MID / MAX | counts/us |
|---|---|---|
| Futaba 惯例 | 172 / 992 / 1811 | 1.639 |
| **本机实测** | **240 / 1024 / 1807** | **1.567** |

照抄惯例值的后果：满杆只算出 0.96、死区和拨杆阈值跟着偏。

这组数可信的依据：**两侧半幅对称到 1 个计数以内**（`1024-240=784`、`1807-1024=783`）
——摇杆没推到底的话会明显不对称；而且与 i-BUS 的 `1000/1500/2000` 严格线性对应。
增减方向与 i-BUS 一致，换协议没变。

⚠ 派生常量要一起改：`SBUS_AXIS_MAX`（= 正半幅 783，由端点推出不写死）、
`SBUS_SW_LOW_MAX/HIGH_MIN`（中位 ±392，即半幅的一半）、`APP_SBUS_DB`（13 ≈ 1.6%）。

**通道分配与 i-BUS 完全一致**（那是发射机 Aux. channels 菜单决定的，与协议无关）。
S.BUS 下已实机确认 **VrA=4、VrB=5、SwA~SwD=6/7/8/9**；下标 0~3 的四个摇杆轴是
仅剩的四个槽，没有逐项复测。**VrB 拧到底确认能读到 1807**，所以拨盘的 2% 触发带够得着。

⚠ **`test_sbus.c` 的 `testRangeConstants` 只能查自洽性，查不出这组数对不对**——
主机没有接收机的真值。它能抓的是"改了中位忘了改端点"这类部分修改（靠对称性），
整组换成另一组自洽的数照样全绿。**换接收机或改发射机行程后必须实机复测。**

### ⚠ S.BUS 帧尾不能严格判 0x00

标准说 `frame[24]` 是 `0x00`，但带 SBUS2 遥测的接收机会填 `0x04/0x14/0x24/0x34`
表示遥测时隙。严格判 `0x00` 会把这些接收机的合法帧**全部拒掉**，现象是
「波特率对、`rxEventCount` 正常涨，却一帧都解不出来」——和当初 i-BUS 那次踩的坑
长得一模一样，很难往这上面想。所以 `SBUS_ParseFrame` **只校验帧头**。

### ⚠ SBUS 座子带硬件反相器，i-BUS 的反相设置与另外两个相反

板子手册 §6 那张表写 `1 | S | PC02 | 反相 GPIO`，两处都要注意：

- **引脚是 PD2 不是 PC02**——手册笔误。引脚图和 `.ioc`（`PD2.Signal=UART5_RX`；
  `PC2_C` 是 `SPI2_MISO`）都是 PD2。
- **座子上有反相器**，板子是按 S.BUS / DBUS 这类反相电平设计的。于是：
  - **DBUS 本身反相** → 被硬件翻正 → MCU 侧 **RXINV 关**
  - **S.BUS 本身反相** → 被硬件翻正 → MCU 侧 **RXINV 关**（与 DBUS 同）
  - **i-BUS 是正常 TTL** → 被硬件翻反 → MCU 侧 **RXINV 开**，翻回来抵消

换句话说 **S.BUS 才是这个座子的"原生"用法**，i-BUS 是绕一圈抵消。

### ⚠ 判断极性对不对，要看 `rxEventCount`，不能看 `uartErrorCount`

`uartErrorCount` 在**两种极性下都会快速上涨**，拿它当判据必然被骗。有区分度的是空闲事件：

| 现象 | 含义 |
|---|---|
| `lastUartError` 固定 4(FE) + **`rxEventCount` 恒为 0** | 极性反了。线路一直被当成起始位，所以既报 FE 又**永远等不到空闲** |
| `rxEventCount` 按帧率上涨 + `lastRxSize` 稳定在该后端的帧长（DBUS 18 / i-BUS 32 / S.BUS 25） | 极性和波特率都对，剩下的是解析层的事 |
| `rxEventCount` 慢速零星增长 + `lastRxSize` 在 0~4 且**随摇杆变化** | 线上是 **PPM/PWM 脉冲**，根本不是串口——去查接收机输出模式 |

### ⚠ 接收机输出模式必须与所选后端一致

FS-i6X 的输出模式菜单是 **`PWM/i-BUS` 和 `PPM/S.BUS` 成对切换**的，
**改完接收机要重新上电**。两个模式各自对应哪个后端：

| 发射机输出模式 | 对应后端 |
|---|---|
| `PWM/i-BUS` | `APP_REMOTE_BACKEND_IBUS` |
| `PPM/S.BUS` | `APP_REMOTE_BACKEND_SBUS` |

模式和后端对不上时固件怎么配 UART 都解不出来。**接线到底走哪个口，
在换协议时要用 `lastRxSize` 实测确认**（见上面那张判据表）：
i-BUS 该稳定 32、S.BUS 该稳定 25；若读到 0~4 且随摇杆变化，那是 PPM 脉冲，
说明这根线接的是 PPM 输出而不是串行输出口。

⚠⚠ **踩过的坑，值得记：当初那两个故障（输出模式是 PPM + RXINV 没开）是叠加的。**
在 PPM 还没改的情况下试 RXINV，拿到的是**假阴性**，正确的假设因此被丢掉、绕了一大圈。
**一个假设的否定结论，只在其他故障都排除之后才成立。**

### 通道映射（2026-09-22 实机确认）

| 下标 | FlySky 编号 | 对应 | 常量 |
|---|---|---|---|
| 0 | CH1 | 右摇杆横（右增） | `IBUS_CH_RIGHT_X` |
| 1 | CH2 | 右摇杆纵（上增） | `IBUS_CH_RIGHT_Y` |
| 2 | CH3 | 左摇杆纵（上增） | `IBUS_CH_LEFT_Y` |
| 3 | CH4 | 左摇杆横（右增） | `IBUS_CH_LEFT_X` |
| 4 | CH5 | VrA 旋钮（顺时针增） | `IBUS_CH_KNOB_A` |
| 5 | CH6 | VrB 旋钮 | `IBUS_CH_KNOB_B` |
| 6 | CH7 | SwA（两档） | `IBUS_CH_SW_A` |
| 7 | CH8 | SwB（两档） | `IBUS_CH_SW_B` |
| 8 | CH9 | **SwC（三档）** | `IBUS_CH_SW_C` |
| 9 | CH10 | SwD（两档） | `IBUS_CH_SW_D` |

中位 1500，量程 1000~2000。四个轴都是"右增 / 上增"，与 `Remote_t` 的
"x 向右为正、y 向上为正" 同向，所以 `IBUS_Axis` 直接减中位、**不加负号**；
回中正好 1500，死区 `APP_IBUS_DB = 8` 够用。

⚠ **出厂状态下 SwA~SwD 没有分配到任何通道**，拨动时所有通道都不动、未用槽停在 1500。
要在发射机 **系统菜单 → Aux. channels** 里把 SwA~SwD 依次分配到 CH7~CH10 才会出现。
另外发射机默认只开 **6** 个通道，要用 10 通道得先在 §7.13 Aux Switches 里改
（同一个菜单还负责启用/关闭各个开关旋钮，见 §9.2）。**这两步已经做完。**

⚠ **i-BUS 的槽位数是协议固定的 14 个**，与遥控器实际通道数无关（i6X 只用前 10 个，
其余由接收机填中位）。别用槽位数去推断型号。

### ⚠⚠ 拨杆是"上小下大"：UP=1000 / MID=1500 / DOWN=2000

和"值大 = 位置高"的直觉相反。`IBUS_ConvertSwitch` 因此是**低值判 UP、高值判 DOWN**，
不要"顺手改回来"——`test_ibus.c` 的 `testSwitchThresholds` 专门钉死了这个方向，
改反了单测会红。

**方向搞反的后果是安全性的**：SwC 是使能级，**上=急停、下=运行**，
反了会让你以为在打急停、实际在选运行。

### ⚠ SwC 方向与旧 DR16 右拨杆相反

发射机要求**所有拨杆位于 UP 才允许开机**，所以 UP 是唯一保证的上电初始状态，
必须映射为最安全的模式。于是 SwC **上=卸力+急停**，而旧的 DR16 右拨杆是下=急停。
这条也让通道值残留 0 的退化方向（落在 UP 一侧）正好是安全的。

### i6X 只有一个三档开关——已不是问题

SwC 是三档，SwA / SwB / SwD 都是两档。早先的交接文档把"底盘需要两个三档"
列为模式分配的阻塞项，并列了换开关模块 / 挪通道 / 混控三条出路。

**这个阻塞项已经消解，三条出路一条都不用走。** 现行方案把"允不允许动"和"做什么"
重新切分，三档只用在使能级（SwC）上，模式改由 **VrA 选组 + SwA/SwB 两档**表达。
所以**不需要拆机换三档开关模块，也不需要再改发射机菜单**。

完整键位表见 `../Codex文档/遥控键位与模式分配.md`。

### ⚠ 键鼠是 DR16 独有的

`task_remote_state_t` 里那几个字段和相关逻辑用 `#if` 守着，切到 i-BUS 后不存在——
Watch 表达式要跟着改。

`Remote_t` 本身已经**没有** `dialValid` 了：它是 DR16 独有的概念，i-BUS 侧恒为 1。
`dr16_data_t.dialValid` 仍然保留（那是 DBUS 的真实字段），只是在 `DR16_MakeRemote`
里用来决定要不要写 `knobA`，不再往上传。

⚠ **UART 参数在 `Remote_Task_Init` 里按后端重新 `HAL_UART_Init`，不改 `Core/Src/usart.c`**
——那个文件 CubeMX 重新生成会冲掉。DR16 那组常量就是 CubeMX 现有值，对它是空操作。

### 主机单测配方

```bash
INC="-IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/Communication/DR16 -IUser_File/2_Device/Communication/USB \
-IUser_File/2_Device/Communication/FlySky"
```

| Test | Extra sources |
|---|---|
| `test_angle` | `2_Algorithm/Angle.c` |
| `test_dr16` | `DR16/device_dr16.c` |
| `test_ibus` | `FlySky/device_ibus.c` |
| `test_sbus` | `FlySky/device_sbus.c` |
| `test_usb_protocol` | `USB/device_usb_protocol.c` `2_Algorithm/CRC.c` |
| `test_board_protocol` | see below — spans both projects |

`test_ibus` / `test_sbus` 只需要 `remote_input.h`，不碰 HAL。
两者都用常量而不是写死下标表达映射，所以回填通道常量之后仍然成立。

⚠ **`test_sbus` 的打包函数 `testBuildFrame` 是逐位写入实现的，故意不复用
`device_sbus.c` 里那个窗口移位公式**——打包解包共用同一条公式时，公式本身错了
也能往返自洽，等于没测。`testBitPackingIsolation` 同理：一次只设一路，
"所有通道设成同一个值再往返"照样能过错位的代码，逐路隔离才抓得到。

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
