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

### 主机单测配方

```bash
INC="-IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/Communication/DR16 -IUser_File/2_Device/Communication/USB"
```

| Test | Extra sources |
|---|---|
| `test_angle` | `2_Algorithm/Angle.c` |
| `test_dr16` | `DR16/device_dr16.c` |
| `test_usb_protocol` | `USB/device_usb_protocol.c` `2_Algorithm/CRC.c` |
| `test_board_protocol` | see below — spans both projects |

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
