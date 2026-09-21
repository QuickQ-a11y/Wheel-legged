# STM32 + VS Code 环境搭建指南（Windows）

从空白的 Windows 电脑配到**能编译、能烧录、能 F5 单步、能用 Live Watch 实时看变量**。适用于任何 STM32CubeMX 生成的 CMake 工程。

参考视频：keysking [《爽！手把手教你用 VSCode 开发 STM32》](https://www.bilibili.com/video/BV1QfbpzGENy/)（2025-08-15）。

---

## 一、这套环境是什么

| 组件 | 管什么 |
|---|---|
| **VS Code** | 编辑器本体 |
| **STM32 VSCode 扩展**（ST 官方） | 工具链、CMake 集成、代码索引。**它会自己下载 GCC / CMake / Ninja / GDB** |
| **Cortex-Debug 扩展** | 接 GDB 和调试器，提供断点、单步、**Live Watch** |
| **OpenOCD 或 J-Link** | GDB Server，真正跟芯片说话的那一层 |

> **别照网上的旧教程单独装 `gcc-arm-none-eabi` / CMake / Ninja。** ST 扩展 3.x 自带 bundle manager，会把工具链拉到 `%LOCALAPPDATA%\stm32cube\bundles\`；只有 GDB Server 不在里面，所以要额外装 OpenOCD 或 J-Link。

---

## 二、动手前必须知道的三件事

### 0. 路径写法约定

**`<尖括号>` 一律是占位符，要换成你自己的值，不能照抄。** 不带尖括号的（如 `%LOCALAPPDATA%\stm32cube\bundles`）是固定位置，不用改。

后面写配置时要填两组值。把这两张表抄到记事本，边做边填：

**表 A — 装软件时得到**

| # | 记什么 | 在哪一步 | 你的值 |
|---|---|---|---|
| ① | ARM 工具链 bundle **版本号** | 步骤 2 | `_______________` |
| ② | Ninja bundle **版本号** | 步骤 2 | `_______________` |
| ③ | **J-Link 安装目录** | 步骤 5 | `_______________` |

**表 B — 你自己工程的参数**

| # | 记什么 | 从哪来 | 你的值 |
|---|---|---|---|
| ④ | **芯片型号**，如 `STM32F103C8` | CubeMX 工程 / 芯片丝印 | `_______________` |
| ⑤ | **OpenOCD 芯片系列配置**，如 `target/stm32f1x.cfg` | 按芯片系列取，见第六节 | `_______________` |
| ⑥ | **工程名**（elf 文件名，不含后缀） | `CMakeLists.txt` 的 `CMAKE_PROJECT_NAME` | `_______________` |

安装步骤里用 📌 标出了填表位置。只用 CMSIS-DAP 就不用填 ③。

### 1. 路径不能有空格和中文

VS Code 安装路径、工程路径、**Windows 用户名**都不行 —— ST 社区有帖子记录 [用户名带空格导致 clangd 直接罢工](https://community.st.com/t5/stm32cubeide-for-visual-studio/stm32-vscode-clangd-not-working-under-windows-with-space-in-user/td-p/870586)。用户名是中文的话**先建一个英文账户**，比后面逐个绕坑省事。

### 2. 第一次烧录前确认板子是安全的

如果板子接了电机、舵机之类的执行器，先断开动力或做好限位 —— 烧完程序立刻就会按固件逻辑动起来。

---

## 三、下载清单

**必装**

| 软件 | 下载地址 | 说明 |
|---|---|---|
| **VS Code** | https://code.visualstudio.com/Download | 选 Windows x64 **User Installer**（免管理员权限），需 Win10/11 64 位 |
| **STM32 VSCode 扩展** | https://marketplace.visualstudio.com/items?itemName=STMicroelectronics.stm32-vscode-extension | ID `stmicroelectronics.stm32-vscode-extension`，也可在扩展市场搜 `STM32` |
| **Cortex-Debug** | https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug | ID `marus25.cortex-debug` |
| **Git for Windows** | https://git-scm.com/download/win | 克隆工程 |

**按调试器二选一（或都装）**

| 软件 | 下载地址 | 说明 |
|---|---|---|
| **xPack OpenOCD** | https://github.com/xpack-dev-tools/openocd-xpack/releases | CMSIS-DAP / ST-Link 用。下 `xpack-openocd-*-win32-x64.zip` |
| **J-Link 软件包** | https://www.segger.com/downloads/jlink/ | J-Link 用。选 **Windows 64-bit Installer**，自带 USB 驱动 |

**可选**

| 软件 | 下载地址 | 什么时候才需要 |
|---|---|---|
| **STM32CubeMX** | https://www.st.com/en/development-tools/stm32cubemx.html | 只有要改 `.ioc`（引脚、时钟、外设）才需要，平时写业务代码用不上。要注册 ST 账号，审批最长 48 小时，建议提前申请 |
| **Zadig** | https://zadig.akeo.ie/ | **别急着装**，见第五节 |

---

## 四、分步安装

**每步跑一遍验收命令，对了再进下一步**，别一路装完最后再排查。

### 步骤 1：VS Code

装完先别急着开工程。

### 步骤 2：STM32 VSCode 扩展

扩展面板搜 `STM32`，装 STMicroelectronics 发布的那个（不是第三方 `stm32-for-vscode`）。装完 bundle manager 会自动下载工具链，**要联网、等几分钟**。

```powershell
dir $env:LOCALAPPDATA\stm32cube\bundles
```

| 目录 | 作用 | 参考版本 |
|---|---|---|
| `gnu-tools-for-stm32` | ARM GCC + GDB | `14.3.1+st.2` |
| `ninja` | 构建器 | `1.13.2+st.1` |
| `cmake` | 构建系统 | `4.2.3+st.1` / `4.3.1+st.1` |
| `st-arm-clangd` | 代码索引 | — |
| `stlink-gdbserver` / `jlink-gdbserver` | ST-Link / J-Link 的 GDB Server | — |

📌 **进目录看 `gnu-tools-for-stm32` 和 `ninja` 下实际的版本号文件夹叫什么，填进表 A 的 ①②。** 右列只是参考，**不必凑成一样**，装什么版本就用什么版本。

```powershell
dir $env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32
dir $env:LOCALAPPDATA\stm32cube\bundles\ninja
```

**验收**（`<GCC版本>` 换成刚记下的值）：

```powershell
& "$env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32\<GCC版本>\bin\arm-none-eabi-gcc.exe" --version
```

### 步骤 3：Cortex-Debug

扩展市场搜 `Cortex-Debug`，认准发布者 `marus25`。**Live Watch 要 v1.6 以上**，装完在详情页确认版本（当前 1.12.1）。

### 步骤 4：OpenOCD

1. 从 [releases 页](https://github.com/xpack-dev-tools/openocd-xpack/releases) 下 `xpack-openocd-*-win32-x64.zip`；
2. 解压到一个**不带空格和中文**的目录，位置随你；
3. 把该目录下的 `bin` 加进用户环境变量 `Path`（设置 → 搜「环境变量」→ 编辑用户变量 `Path`）；
4. **关掉所有 PowerShell 和 VS Code 再重开**，环境变量不作用于已打开的窗口。

```powershell
openocd --version    # 应打印 Open On-Chip Debugger 0.12.0
```

报「不是内部或外部命令」就是 `Path` 没生效或窗口没重开。

### 步骤 5：J-Link（用 J-Link 的话）

下 **J-Link Software and Documentation Pack, Windows 64-bit Installer**，USB 驱动一起装好。向导会让你选目录，默认 `C:\Program Files\SEGGER\JLink`。

📌 **不管用不用默认，把最终目录记进表 A 的 ③** —— 后面配置要填它，而且**它带空格**，写进 JSON 时要小心。

```powershell
Test-Path "<J-Link目录>\JLinkGDBServerCL.exe"
Test-Path "<J-Link目录>\JLink.exe"
```

两条都要返回 `True`。有 `False` 就去安装目录翻实际文件名（不同版本略有出入）。

### 步骤 6：Git

```powershell
git --version
```

---

## 五、接调试器与驱动

### CMSIS-DAP

**先直接插上试，绝大多数情况不用装驱动**：v1（HID）完全免驱，v2（bulk）靠 MS OS 描述符自动绑 WinUSB，Win10/11 上都是即插即用。设备管理器里能看到设备（v1 在「人体学输入设备」，v2 在「通用串行总线设备」）、没有黄色感叹号就算好。

> ⚠ **不要一上来就用 Zadig 换驱动。** 对 **v1（HID）设备**换成 WinUSB 会把本来能用的调试器弄坏（OpenOCD 在 v1 模式下就是走 HID）。只有**同时满足**两条才考虑 Zadig：① `openocd` 报 `unable to find matching CMSIS-DAP device`，② 设备管理器里它是「其他设备 / 未知设备」带黄色感叹号。这时才给它绑 WinUSB。

### ST-Link / J-Link

装完各自的官方软件包即可，驱动自带。J-Link 插上后设备管理器应出现 `J-Link driver`。

---

## 六、建配置文件

下面四个文件放在**工程根目录**（`CMakeLists.txt` 所在那一层），照抄后按表 A / 表 B 替换占位符。

### `openocd.cfg`

烧录任务和调试共用这一份，换调试器只改这个文件。

```tcl
# 调试器接口：按你手上的改，三选一
source [find interface/cmsis-dap.cfg]
# source [find interface/stlink.cfg]
# source [find interface/jlink.cfg]

transport select swd

# 芯片系列配置，填表 B 的 ⑤
source [find <芯片系列cfg>]

# SWD 时钟，单位 kHz。先给保守值，稳定后可往上试 2000 / 4000
adapter speed 1000
```

⑤ 取决于芯片系列，OpenOCD 安装目录的 `share/openocd/scripts/target/` 下能看到全部可选项，常见的有 `target/stm32f1x.cfg`、`target/stm32f4x.cfg`、`target/stm32h7x.cfg`。

> **OpenOCD 报「找不到设备」时**：有些杂牌 CMSIS-DAP 的厂商 VID 不在 OpenOCD 默认扫描表里，要显式指定。设备管理器右键设备 → 属性 → 详细信息 → 硬件 ID，读出 `USB\VID_xxxx&PID_xxxx`，然后在 `source [find interface/cmsis-dap.cfg]` 下面加一行：
>
> ```tcl
> cmsis_dap_vid_pid 0xXXXX 0xYYYY
> ```

### `.vscode/launch.json`

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "OpenOCD Live Watch",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/Debug/<工程名>.elf",
            "servertype": "openocd",
            "searchDir": [
                "${workspaceFolder}"
            ],
            "configFiles": [
                "openocd.cfg"
            ],
            "gdbPath": "${env:LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/<GCC版本>/bin/arm-none-eabi-gdb.exe",
            "device": "<芯片型号>",
            "runToEntryPoint": "main",
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        },
        {
            "name": "J-Link Live Watch",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/Debug/<工程名>.elf",
            "servertype": "jlink",
            "serverpath": "<J-Link目录>/JLinkGDBServerCL.exe",
            "armToolchainPath": "${env:LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/<GCC版本>/bin",
            "device": "<芯片型号>",
            "interface": "swd",
            "runToEntryPoint": "main",
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        }
    ]
}
```

两个坑：

- **gdb 路径必须带 `.exe`** —— 绝对路径不走 PATHEXT 自动补全，漏了会报找不到 gdb。
- **`${env:LOCALAPPDATA}` 是 VS Code 变量语法**，别写成 `%LOCALAPPDATA%`（那是 shell 语法，JSON 里不展开）。路径用正斜杠 `/`，Windows 认。

### `.vscode/tasks.json`

工具链不在系统 `Path` 上，靠 `options.env` 临时补进去。

```json
{
    "version": "2.0.0",
    "options": {
        "cwd": "${workspaceFolder}",
        "env": {
            "PATH": "${env:LOCALAPPDATA}\\stm32cube\\bundles\\gnu-tools-for-stm32\\<GCC版本>\\bin;${env:LOCALAPPDATA}\\stm32cube\\bundles\\ninja\\<NINJA版本>\\bin;${env:PATH}"
        }
    },
    "tasks": [
        {
            "label": "编译",
            "type": "shell",
            "command": "cmake --build build/Debug",
            "problemMatcher": [
                "$gcc"
            ],
            "presentation": {
                "reveal": "always",
                "panel": "shared",
                "clear": true
            },
            "group": "build"
        },
        {
            "label": "烧录 (OpenOCD)",
            "type": "shell",
            "command": "openocd -f openocd.cfg -c \"program build/Debug/<工程名>.elf verify reset exit\"",
            "problemMatcher": [],
            "presentation": {
                "reveal": "always",
                "panel": "shared",
                "clear": true
            }
        },
        {
            "label": "编译 + 烧录 (OpenOCD)",
            "type": "shell",
            "command": "cmake --build build/Debug; if ($LASTEXITCODE -eq 0) { openocd -f openocd.cfg -c \"program build/Debug/<工程名>.elf verify reset exit\" }",
            "problemMatcher": [
                "$gcc"
            ],
            "presentation": {
                "reveal": "always",
                "panel": "shared",
                "clear": true
            },
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "编译 + 烧录 (J-Link)",
            "type": "shell",
            "command": "cmake --build build/Debug; if ($LASTEXITCODE -eq 0) { & '<J-Link目录>/JLink.exe' -device <芯片型号> -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink }",
            "problemMatcher": [
                "$gcc"
            ],
            "presentation": {
                "reveal": "always",
                "panel": "shared",
                "clear": true
            }
        }
    ]
}
```

**⚠ 这里最容易抄错的一点**：网上很多教程写成 `cmake --build build/Debug && openocd ...`，**在 Windows 上会直接报语法错误** —— VS Code 默认用 PowerShell 5.1，而 `&&` 是 PowerShell 7 才加的。所以上面用 `; if ($LASTEXITCODE -eq 0) { ... }`。用 `$LASTEXITCODE` 而不是 `$?`，后者对原生命令在 PS 5.1 下不可靠。

也别用 VS Code 的 `dependsOn` 串联：它在前一个任务失败时**仍会继续**，会把上一次的旧 elf 烧进去。

### `flash.jlink`（只有用 J-Link 才需要）

```
loadfile build/Debug/<工程名>.elf
r
g
qc
```

---

## 七、第一次跑通

### 1. 打开工程

用 VS Code **打开文件夹**（不是单个文件），且是工程根目录 —— 能看到 `CMakeLists.txt` 那一层。

### 2. 配置 CMake

CubeMX 生成的 CMake 工程自带 `CMakePresets.json`，CMake Tools 会提示选 preset，选 **`Debug`**；没提示就 `Ctrl+Shift+P` → `CMake: Select Configure Preset` → `Debug`，再 `CMake: Configure`。成功的标志是生成了 `build/Debug/`。

### 3. 编译 + 烧录

按 **`Ctrl+Shift+B`**（默认任务「编译 + 烧录 (OpenOCD)」）。终端应依次出现：

```
[NN/NN] Linking C executable <工程名>.elf
...
** Programming Started **
** Programming Finished **
** Verified OK **
shutdown command invoked
```

> ⚠ **`Ctrl+Shift+F5` 不是烧录键。** 它在 VS Code 里是「重启调试」，无活动会话时退化成「运行当前 target」，把交叉编译的 ARM `.elf` 当本机程序执行，报「不是有效的 Win32 应用程序」。**VS Code 没有任何默认快捷键的含义是「烧进单片机」**，只有 `Ctrl+Shift+B` 和 `F5`。

### 4. 进调试

左侧「运行和调试」面板下拉框选 **`OpenOCD Live Watch`**（用 J-Link 就选 J-Link 那个），按 **`F5`**。程序会自动烧录、复位、停在 `main`，此时可以下断点单步。

---

## 八、Live Watch 怎么用

Live Watch 是这套环境相对 Keil 最大的优势：**不停机、不打断点，直接看变量实时变化**，特别适合调控制环、看状态机跳转。

进调试后左侧会多出 **LIVE WATCH** 区，点 `+` 加表达式，填全局变量名或 `结构体.成员` 这样的路径。刷新率在 `launch.json` 的 `samplesPerSecond` 里调，4 Hz 够用。

两个注意：

- **只能看全局变量和静态变量**，局部变量出了作用域就没了；
- **别整个展开大结构体** —— 几百个成员既看不过来，也会明显拖慢刷新，按需一条条加。

---

## 九、验收清单

- [ ] `arm-none-eabi-gcc --version` 有输出
- [ ] `openocd --version` 打印 0.12.0（或 J-Link 路径 `Test-Path` 为 `True`）
- [ ] `git --version` 有输出
- [ ] 扩展面板能看到 `STM32CubeIDE for Visual Studio Code` 和 `Cortex-Debug`
- [ ] CMake preset 下拉框里有 `Debug`，配置后生成了 `build/Debug/`
- [ ] `Ctrl+Shift+B` 产出 `<工程名>.elf`，**零警告零错误**
- [ ] 烧录日志有 `** Verified OK **` 和 `shutdown command invoked`
- [ ] `F5` 后停在 `main`，能单步
- [ ] Live Watch 里的变量数值在实时刷新

---

## 十、常见报错对照

| 现象 | 原因 / 处理 |
|---|---|
| `.elf: 不是有效的 Win32 应用程序` | 按了 `Ctrl+Shift+F5`。改用 `Ctrl+Shift+B` 或 `F5` |
| `标记"&&"不是此版本中的有效语句分隔符` | PowerShell 5.1 不支持 `&&`，改用第六节的 `; if ($LASTEXITCODE -eq 0) { }` |
| `openocd 不是内部或外部命令` | `Path` 没加或窗口没重开 |
| `unable to find matching CMSIS-DAP device` | ① 加 `cmsis_dap_vid_pid`，见第六节；② 设备管理器有黄色感叹号，这时才轮到 Zadig |
| `init mode failed (unable to connect to the target)` | SWD 接线或供电。确认板子已上电、SWDIO/SWCLK/GND 都接了；再把 `openocd.cfg` 的 `adapter speed` 降到 500 |
| 编译报找不到 `arm-none-eabi-gcc` | bundle 没下完，或 `tasks.json` 版本号和实际 bundle 对不上 |
| 代码全是红波浪线、跳转失效 | clangd 罢工。九成是路径带空格或中文 |
| CMake preset 下拉框是空的 | 没在工程根目录打开文件夹，`CMakePresets.json` 不在工作区顶层 |
| `F5` 报找不到 `arm-none-eabi-gdb` | `gdbPath` 漏了 `.exe`，或版本号与实际 bundle 不符 |
