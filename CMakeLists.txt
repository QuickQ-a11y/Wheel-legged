cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# TinyMPC 是 C++17 + Eigen，只有它和 chassis_mpc.cpp 走 C++；业务代码仍是 C。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Wheel-legged)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User_File/1_Middleware/1_Driver/FDCAN/driver_fdcan.c
    User_File/1_Middleware/1_Driver/SPI/driver_spi.c
    User_File/1_Middleware/1_Driver/UART/driver_uart.c
    User_File/1_Middleware/1_Driver/USB/driver_usb.c
    User_File/1_Middleware/2_Algorithm/Angle.c
    User_File/1_Middleware/2_Algorithm/CRC.c
    User_File/1_Middleware/2_Algorithm/Kalman.c
    User_File/1_Middleware/2_Algorithm/LESO.c
    User_File/1_Middleware/2_Algorithm/LQR.c
    User_File/1_Middleware/2_Algorithm/PID.c
    User_File/1_Middleware/2_Algorithm/QuaternionEKF.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    User_File/2_Device/IMU/BMI088/device_bmi088.c
    User_File/2_Device/Motor/Motor_DM/device_motor_dm.c
    User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c
    User_File/2_Device/Communication/USB/device_usb_protocol.c
    User_File/2_Device/Communication/DR16/device_dr16.c
    User_File/3_Chariot/1_Module/Chassis/chassis_config.c
    User_File/3_Chariot/1_Module/Chassis/chassis_control.c
    User_File/3_Chariot/1_Module/Chassis/chassis_observer.c
    User_File/3_Chariot/1_Module/Chassis/chassis_remote.c
    User_File/3_Chariot/1_Module/Chassis/chassis_vmc.c
    User_File/3_Chariot/1_Module/Chassis/chassis_mpc.cpp
    User_File/5_Task/task_can.c
    User_File/5_Task/chassis_task.c
    User_File/5_Task/task_imu.c
    User_File/5_Task/task_can_dispatch.c
    User_File/5_Task/task_usb.c
    User_File/5_Task/task_remote.c
    # TinyMPC 求解器。codegen.cpp 依赖 dirent.h/unistd.h，裸机编不了，故不加；
    # admm.cpp 引用 benchmark_rho_adaptation，rho_benchmark.cpp 必须一起编。
    User_File/1_Middleware/2_Algorithm/TinyMPC/tinympc/admm.cpp
    User_File/1_Middleware/2_Algorithm/TinyMPC/tinympc/tiny_api.cpp
    User_File/1_Middleware/2_Algorithm/TinyMPC/tinympc/rho_benchmark.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User_File/1_Middleware/0_Common
    User_File/1_Middleware/1_Driver/FDCAN
    User_File/1_Middleware/1_Driver/SPI
    User_File/1_Middleware/1_Driver/UART
    User_File/1_Middleware/1_Driver/USB
    User_File/1_Middleware/2_Algorithm
    User_File/2_Device/IMU/BMI088
    User_File/2_Device/Motor/Motor_DM
    User_File/2_Device/Motor/Motor_DJI
    User_File/2_Device/Communication/USB
    User_File/2_Device/Communication/DR16
    User_File/3_Chariot/1_Module/Chassis
    User_File/5_Task
    Middlewares/ST/ARM/DSP/Include
    Middlewares/ST/ARM/DSP/PrivateInclude
    User_File/1_Middleware/2_Algorithm/TinyMPC        # 让 #include <tinympc/...> 生效
    User_File/1_Middleware/2_Algorithm/TinyMPC/Eigen  # 让 #include <Eigen.h> 生效
)

# C++ 编译选项。裸机没有异常处理和 RTTI 的运行时支持，不关掉会链进 unwind
# 全家桶；-fno-threadsafe-statics 去掉函数内静态量的 __cxa_guard 锁。
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# vendored TinyMPC 单独放宽 unused-variable：那几个残差量原本只被 std::cout 使用，
# 我们摘掉打印后就成了未使用量。不为第三方代码改写源码，也不放宽自己代码的警告级别。
set_source_files_properties(
    User_File/1_Middleware/2_Algorithm/TinyMPC/tinympc/admm.cpp
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-variable"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
    DISABLEFLOAT16
    # 关掉 Eigen 的运行时断言。不关的话每个 eigen_assert 都会把文件路径、表达式和
    # __PRETTY_FUNCTION__（模板签名极长）塞进 .rodata，实测 Debug 下多占 170+ KB；
    # 而且那些边界检查在控制环里是真的在跑，直接吃求解耗时预算。
    EIGEN_NO_DEBUG
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
