# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

This repo (`Wheel-legged`) is the **whole robot** — both boards live here:

- `Chassis/` — STM32H723 firmware for the five-bar-linkage wheel-legged balancing **chassis**. One CubeMX project; artifact is `Wheel-legged.elf`.
- `Gimbal/` — STM32H723 firmware for the **gimbal**, derived from the chassis. Two independent CubeMX projects: `Real/` (the real application) and `Debug/` (system identification). Each carries its own full `User_File/` copy — they are allowed to diverge, so **never assume a fix in one propagated to the other**.

Outside the repo, under the workspace root `../`:
- `Matlab/` — offline model derivation (`ABK_LQR.py` is the live one, `ABK_LQR.m` is stale; `VMC.m`). Not built.
- `Others/` — reference open-source projects (HERO_LEG, SPR, ZJU docs …). Read-only.

The two boards talk over a dedicated **FDCAN3** link (PD12/PD13): the DR16 receiver is on the gimbal board, which forwards remote input down to the chassis. Protocol is defined in each project's `User_File/1_Middleware/0_Common/app_config.h`.

`Chassis/CLAUDE.md` holds the **binding style/collaboration rules** (Chinese) — read it before touching any business code; `Gimbal/CLAUDE.md` is the same document. This file covers build/test mechanics and architecture. When the two conflict, `Chassis/CLAUDE.md` wins.

**Open one project at a time in the editor** — `Chassis/` or `Gimbal/Real/`, never the repo root. Each carries its own `.clangd` and `.vscode/`, and both use `${workspaceFolder}`-relative paths that only resolve when that project is the workspace root.

⚠ Each of the three CubeMX projects ships its own copy of `Drivers/` (~11 MB) and `Middlewares/` (~15 MB) — about **78 MB of duplicated ST HAL/CMSIS** in the repo. This is CubeMX's doing: it regenerates those paths per project. Deduplicating means fighting CubeMX's output layout for every regeneration. **Decided: leave it.** Don't propose sharing them again.

## MATLAB ↔ firmware coupling

**`../Matlab/ABK_LQR.py` is the live script — `ABK_LQR.m` is stale (still small-wheel values, and a different `lqr_Q`/`lqr_R`). Do not use the `.m`.**

`ABK_LQR.py` derives the A/B matrices, solves LQR across a leg-length grid, and fits every `K(row,col)` to a `poly22` surface in (left leg length, right leg length). It prints **four** blocks that are pasted verbatim into `chassis_config.c`: `K` → `Chassis_Config.lqr.coefficients`, and `Ad`/`Bd`/`L` → `Chassis_Config.leso.{Ad,Bd,L}_coefficients`. `Algorithm_LQR_FitLqrKPoly22` reconstructs `K` at runtime from the live leg lengths.

⚠ **The four blocks must come from the same run.** `leso.comp_scale` is non-zero, so a `K` and a LESO model built from different physical parameters actively fight each other. Element counts to verify after pasting: 240 / 600 / 240 / 840.

- The array is a **flat `float[240]`** (40 rows × 6) specifically so the MATLAB text pastes in with no braces and no `f` suffixes. Row order is output-major: rows 1-10 left wheel, 11-20 right wheel, 21-30 left leg swing, 31-40 right leg swing; within each block the 10 states follow `CHASSIS_STATE_*` order. Coefficient order is `p00,p10,p01,p20,p11,p02` — must match `coeffvalues()`.
- **Physical parameters must be kept identical on both sides** or `K` is not this robot's gain. The firmware mirrors them in `Chassis_Config.model` (mass split, `cg_to_hip`) and `Chassis_Config.wheel` (`R`, `half_track`), each commented with its `*_ac` MATLAB counterpart. Changing any of them means regenerating `K`.
- `Chassis_Config.lqr.L0_min/L0_max` must match the script's sampling range (`LM = 0.00:0.01:0.25` offset by `L0_l = 0.10` → `0.10~0.35 m`; currently consistent). Leg lengths outside it are clamped and `Chassis.lqr.limit_flag` is set — extrapolation is deliberately refused.
- `../Matlab/VMC.m` derives the five-bar kinematics implemented in `chassis_vmc.c`.

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

### Chassis

```bash
INC="-IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/IMU/BMI088 -IUser_File/2_Device/Motor/Motor_DM \
-IUser_File/2_Device/Motor/Motor_DJI -IUser_File/2_Device/Communication/DR16 \
-IUser_File/2_Device/Communication/USB -IUser_File/2_Device/Communication/Board \
-IUser_File/3_Chariot/1_Module/Chassis \
-IMiddlewares/ST/ARM/DSP/Include -IDrivers/CMSIS/Include"
FLAGS="-Wall -Wextra -DARM_MATH_CM7 -DDISABLEFLOAT16"
```

| Test | Extra sources |
|---|---|
| `test_angle` | `2_Algorithm/Angle.c` |
| `test_leso` | `2_Algorithm/LESO.c` |
| `test_dr16` | `DR16/device_dr16.c` `0_Common/remote_input.c` |
| `test_usb_protocol` | `USB/device_usb_protocol.c` `2_Algorithm/CRC.c` |
| `test_chassis_vmc` | `chassis_vmc.c` `chassis_config.c` `Angle.c` |
| `test_chassis_remote` | `chassis_remote.c` `chassis_config.c` `Angle.c` |
| `test_chassis_spring` | `chassis_vmc.c` `chassis_config.c` `Angle.c` |
| `test_chassis_observer` | `chassis_observer.c` `chassis_vmc.c` **`$TMP/cfg_obs.c`** `Angle.c` `LESO.c` `LQR.c` |
| `test_chassis_recovery` | `chassis_control.c` + other three chassis `.c` + **`$TMP/cfg_rec.c`** + **`$TMP/mpc_stub.c`** + `Angle/PID/LQR/LESO/Kalman.c` + 6 × `Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_{init,add,sub,mult,trans,inverse}_f32.c` |

Skipped:

- **`test_motor_dm` does not build on host.** It includes CubeMX's `fdcan.h` and
  `device_motor_dm.c` includes `task_can.h`; there are no stubs.
- **`test_chassis_mpc` needs g++**, `-DEIGEN_NO_DEBUG -I<TinyMPC> -I<TinyMPC/Eigen>`, a
  `stm32h7xx.h` DWT stub, and links `tiny_api.cpp admm.cpp rho_benchmark.cpp`
  (`rho_benchmark.cpp` can't be omitted — `admm.cpp` references it). Recipe in
  `Codex文档/handoff-2026-09-05-1524.md` §6.

#### The config overrides — two tests need *opposite* flag values

`test_chassis_recovery` asserts the gates are `0` (`test_output_disabled`) and aborts the whole
suite on its first test, so both suites need overridden copies. **`off_ground_act_flag` must
differ between the two**: `test_chassis_observer` asserts it is `0` (the shipped-safe default),
while `test_chassis_recovery`'s `test_off_ground_actions` only reaches its code path when it is
`1`. Using one shared `cfg_off.c` makes one of the two fail no matter which value you pick.

**The seds must force the value they need, not assume the current one.** These flags are live
tuning switches — whoever is on the robot flips them between sessions, and a recipe written
against "whatever it is right now" silently rots. Each pair below matches the *other* value, so
one sed is a no-op and the other flips it, whichever state the working tree is in.

```bash
sed 's/APP_CHASSIS_OUTPUT_ENABLE 1U/APP_CHASSIS_OUTPUT_ENABLE 0U/' \
    User_File/1_Middleware/0_Common/app_config.h > $TMP/inc/app_config.h

CFG=User_File/3_Chariot/1_Module/Chassis/chassis_config.c

# recovery: gates off, off_ground_act_flag forced to 1
sed -e 's/\.joint_flag = 1U,/.joint_flag = 0U,/' \
    -e 's/\.wheel_flag = 1U,/.wheel_flag = 0U,/' \
    -e 's/\.off_ground_act_flag = 0U,/.off_ground_act_flag = 1U,/' $CFG > $TMP/cfg_rec.c

# observer: gates off, off_ground_act_flag forced to 0
sed -e 's/\.joint_flag = 1U,/.joint_flag = 0U,/' \
    -e 's/\.wheel_flag = 1U,/.wheel_flag = 0U,/' \
    -e 's/\.off_ground_act_flag = 1U,/.off_ground_act_flag = 0U,/' $CFG > $TMP/cfg_obs.c
```

Both compile with `-I$TMP/inc` **first** so the patched `app_config.h` wins.

`test_chassis_recovery` also needs an MPC shell, because `Chassis_Init()` calls into it.
**Check the signatures in `chassis_mpc.h` before copying this** — the interface has already
changed once (`Chassis_MPC_Solve` used to take the state and reference; it was split into
`SetInput` + a no-arg `Solve`), and a stale stub fails at link time, not compile time:

```c
/* $TMP/mpc_stub.c */
#include "chassis_mpc.h"
Chassis_MPC_t Chassis_MPC;
void Chassis_MPC_Init(void) { Chassis_MPC.ready_flag = 1U; }
void Chassis_MPC_SetInput(const float x0[4], float H_ref) { (void)x0; (void)H_ref; }
void Chassis_MPC_Solve(void) { }
```

### Gimbal/Real

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

## Firmware architecture

CubeMX HAL/FreeRTOS in `Core/`, `Drivers/`, `cmake/stm32cubemx/`; all hand-written code under `User_File/` in strict one-way layers (see `Chassis/CLAUDE.md` for the dependency rules). Tasks are created in `Core/Src/freertos.c`: `CAN_Task_Init`, `Chassis_Task_Init`, `IMU_Task_Init`, `Remote_Task_Init`, `USB_Task_Init`.

There is **no unified status/error type** — process functions return `void`, and safety is carried by device online flags, the `Chassis.fault` bitfield and the final zero-output path.

### Chassis module (`3_Chariot/1_Module/Chassis`)

Five files, each a real algorithm or input boundary — no forwarding wrappers:

- `chassis_config.*` — the single `const Chassis_Config_t Chassis_Config`. **Every** mechanical, model, threshold and gain constant lives here; nothing is hardcoded in task/device/control code. Force-type observer thresholds are expressed as *ratios* of single-leg static load so swapping `model`/`wheel` rescales them automatically (this project is meant to run on more than one machine).
- `chassis_vmc.*` — five-bar forward/inverse kinematics and the virtual-force ↔ joint-torque Jacobian. **`F0 > 0` extends the leg / pushes into the ground** (verified by numerical virtual work; the reference HERO_LEG project uses the opposite convention — do not carry its signs over).
- `chassis_control.*` — the one global `Chassis_t Chassis`, state machine, and the whole control chain.
- `chassis_observer.*` — four independent **read-only** observers (`slip` / `ground` / `turn` / `stuck`), each with `Init` / `Update` / `Calc`. `Update` computes physical observables, `Calc` applies thresholds and hysteresis. They write only their own struct and never touch control quantities.
- `chassis_remote.*` — maps the protocol-agnostic `Remote_t` into `Chassis.goal` and the outer mode.

`Chassis_t Chassis` is the sole runtime state and the long-term Watch entry point; there is no second debug struct.

### Gas spring (气弹簧)

Each leg carries one gas strut spanning the knee, mounted between `l1` (near the hip) and
`l2` (near the knee). It applies a real support force that **never appears in the motor
torques**, so it has to be modelled explicitly in three places, all fed by the single
`Chassis.leg[].F0_spring` computed in `Chassis_Leg_Update`:

- `chassis_control.c` LQR path — **subtracted** from `F0` after the MPC/PID branch (both
  branches produce the *total* axial force; the motors only owe the remainder).
- `chassis_control.c` `Joint_Control` — fed forward as joint torque, because the position
  cascade used by recovery/bench/step has no feedforward channel of its own.
- `chassis_observer.c` — **added back** into `Fn`. This is the one term that must be added
  rather than subtracted: it is a real force the ground feels but `VMC_Force_Calc` cannot
  reconstruct from motor torque. Skip it and `Fn_ratio` drifts off 1.0 at standing, silently
  moving every force threshold defined as a ratio of static load.

Because `l5 = 0`, the knee angle is a function of `L0` alone, so the strut has **no `Tp`
component** — the whole effect is one scalar on `F0`. That proof dies if `l5` ever becomes
non-zero. `model.body_mass` does **not** change when the strut is fitted: it is the total
equivalent support mass, and the strut merely takes over part of it.

The strut is **well sized**, and it is the main reason `joint_T_limit` has headroom. The
single-leg `F0` requirement is 97.5 N (`0.5 · body_mass 16.43 · g · F0_gravity_scale 1.21`);
the strut supplies 87–111 N of it, i.e. 90–114%. The motors cover only the ±14 N residual, so
standing joint torque drops from 10–14 N·m to **0.3–1.9 N·m**.

⚠ An earlier version of this section claimed the strut was "over-strong by 3.3×" and that the
robot had to *pull its legs in* to stand. That was wrong: it compared against `body_mass = 6.7`,
a figure back-derived from command values that the DM torque-scale mismatch had inflated 2.67×
(see the MIT torque scale section). Weighing the robot at 20.8 kg fixed `body_mass` to 16.43 and
reversed the conclusion. **Before judging a spring too stiff or too soft, verify the force
requirement side first.**

The compensation is **pure feedforward with no feedback to correct a scale error**, so
`spring.enable_flag` stays `0` until three preconditions hold: `APP_DM_TOR_MIN/MAX` matches
the motor's TMAX register, `model.body_mass` has been re-derived from a whole-robot weighing
rather than from torque, and the observer's force-ratio thresholds have been re-calibrated at
the correct scale. See `Codex文档/实机调试检查清单.md` §2.5 and §6.

Host tests that inject synthetic DM feedback torque must subtract `F0_spring` from the *total*
axial force they intend to represent — the strut's share never passes through the motors.
`test_chassis_recovery` does this in both off-ground fixtures; getting it wrong makes off-ground
detection silently unreachable.

### Control chain (`Chassis_Control`, per 1 ms tick)

```
device feedback -> five-bar state -> wheel/leg speed fusion (2-state Kalman)
  -> 10-state vector -> leg-length PID + roll PID + gravity feedforward -> F0
  -> poly22 K fit from live L0 -> per-state error clamp -> K·error -> 4 outputs
  -> VMC maps (F0, Tp) to joint torques -> torque/current limit -> safety gates
```

State order is fixed and must never be reordered: `s, d_s, fai, d_fai, theta_l, d_theta_l, theta_r, d_theta_r, theta_b, d_theta_b`. Output order: left wheel, right wheel, left leg swing, right leg swing.

`Chassis.lqr.error[]` holds the **clamped** error actually fed to `K` (`Chassis_Config.lqr.error_limit[]`, `0` = unclamped) — the first thing to read when one channel dominates the output.

Remote scheme in STANDING: sticks command **velocities only**. Position and heading targets are integrated from them; releasing a stick zeroes the rate target and latches the hold reference (`body.s` restarts integrating from zero, `target[FAI]` snaps to the current heading). No target ramps.

CAN mapping (`app_config.h:44-45`): **FDCAN1 = the two RIGHT DM hips (`0x001`/`0x002`) *plus* both DJI wheel ESCs (`0x201`/`0x202`)**; **FDCAN2 = the two LEFT DM hips (`0x003`/`0x004`)**. DM feedback ID = `0x010 + motor ID`. The `0x200` frame byte slot is derived from the feedback ID via `APP_DJI_TX_SLOT()` — never hardcode it.

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

### Safety gating — read before changing any motor output

A computed command reaches a motor only if **all** of these pass, and each is a separate place to look:

1. `APP_CHASSIS_OUTPUT_ENABLE` (`app_config.h`) — master compile-time gate.
2. `Chassis_Config.output.joint_flag` / `wheel_flag` — per-channel gates.
3. `Chassis.fault == CHASSIS_FAULT_NONE` — any bit blocks both channels. Bits: `0x01` disabled, `0x02` IMU, `0x04` DM offline, `0x08` DJI offline, `0x10` CAN, `0x20` posture/config, `0x40` kinematics, `0x100` remote offline/e-stop.
4. `Chassis.output.safe_flag` — cleared only by a successful gate above; `Chassis_Command_Send` zeroes the final arrays whenever it is set.

Faults never block the *computation* — `T_joint_req` / `I_wheel_req` keep the live request for Watch while the final `T_joint` / `I_wheel` go to zero. Each output channel has exactly **one** clamp point (`output.joint_T_limit`, `wheel.T_limit`); do not add a second one.

## Conventions beyond the style doc

- Comments and commit messages are Chinese; match the surrounding language.
- Short physical symbols (`d_s`, `fai`, `theta_b`, `L0`, `F0`, `Tp`, `Fn`, `a_y`) with units in the field comment — do not encode units in names (`_mps`, `_radps` suffixes were deliberately removed).
- The FreeRTOS port is `ARM_CM4F` (CubeMX picked the M4F port for this M7 part).
- `Codex文档/` holds design/handover notes. They lag the code — when they disagree, the source wins.
