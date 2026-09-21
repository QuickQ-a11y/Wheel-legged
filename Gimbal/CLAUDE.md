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

| | 帧长 | 波特率 | 字长/校验/停止位 | 电平 |
|---|---|---|---|---|
| DR16 (DBUS) | 18 | 100000 | 9B / EVEN / 1 | 正常 |
| IA10B (i-BUS) | 32 | 115200 | 8B / NONE / 1 | 正常 |

⚠ **UART 参数在 `Remote_Task_Init` 里按后端重新 `HAL_UART_Init`，不改 `Core/Src/usart.c`**
——那个文件 CubeMX 重新生成会冲掉。DR16 那组常量就是 CubeMX 现有值，对它是空操作。

⚠ **键鼠、`dialValid` 是 DR16 独有的**，`task_remote_state_t` 里那几个字段和相关逻辑
用 `#if` 守着，切到 i-BUS 后不存在——Watch 表达式要跟着改。

⚠ **i-BUS 的槽位数是协议固定的 14 个**，与遥控器实际通道数无关（i6X 只用前 10 个，
其余由接收机填中位）。别用槽位数去推断型号。

⚠ **通道→语义的映射尚未实机确认**：`device_ia10b.h` 顶部那组 `IA10B_CH_*` 是按 i6X
出厂默认（Mode 2 / AETR）填的。上电先看裸通道 `remoteTaskDebugState.backendData.channel[]`
确认 AETR 顺序，再回来只改那组常量。

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
