# TinyMPC vendor 说明

上游：https://github.com/TinyMPC/TinyMPC  commit `8b65e27`

## 目录

- `tinympc/` —— 上游 `src/tinympc/`，删掉了它自带的 `CMakeLists.txt`（本工程用顶层 CMake 统一管理）
- `Eigen/` —— 上游 `include/Eigen/` 整包，未裁剪
- `LICENSE` —— 上游许可证

## 对上游的改动（跟版本升级时必须重新施加）

1. **`tinympc/types.hpp`**：`tinytype` 由 `double` 改为 `float`。
   Cortex-M7 单精度 FPU 单周期，内存占用减半；TinyMPC 自己的例子也注明
   "tinytype = float if you want to run on microcontrollers"。
   PC 端已验证 float 与 double 的解和迭代次数完全一致。

2. **`tinympc/tiny_api.hpp`**：摘掉 `#include <iostream>`。
   包含它会实例化 `std::ios_base::Init` 静态对象，把整套流机制拖进 flash
   （几十 KB），哪怕一次都不调用。

3. **`tinympc/*.cpp`**：注释掉全部 `std::cout`（配合第 2 条）。

## 编译注意

- `codegen.cpp` **不参与固件构建**：它是离线代码生成器，依赖文件 IO，目标板用不到。
- `admm.cpp` 引用了 `benchmark_rho_adaptation`，所以 `rho_benchmark.cpp` **必须**一起编。
- 求解器只启用箱式约束（`en_input_bound`），锥/线性/时变线性全部关闭——
  只有关掉才不会走进 `project_soc` / `project_hyperplane` 这些会分配临时量的分支，
  控制环里才能做到零堆分配。
