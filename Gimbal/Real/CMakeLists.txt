cmake_minimum_required(VERSION 3.22)

#
# 云台工程（实际应用）。硬件为达妙 MC02 / STM32H723VGT6。
# 外设配置见 Gimbal_Real.ioc；底层代码与 Gimbal/Debug 共享 ../User_File。
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME Gimbal_Real)

# 让 clangd 能索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

# CubeMX 生成的 HAL / FreeRTOS / USB Device
add_subdirectory(cmake/stm32cubemx)

# 两个云台工程共享的底层代码
set(SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../Real/User_File)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SHARED}/1_Middleware/1_Driver/FDCAN/driver_fdcan.c
    ${SHARED}/1_Middleware/1_Driver/SPI/driver_spi.c
    ${SHARED}/1_Middleware/1_Driver/UART/driver_uart.c
    ${SHARED}/1_Middleware/1_Driver/USB/driver_usb.c
    ${SHARED}/1_Middleware/2_Algorithm/Angle.c
    ${SHARED}/1_Middleware/2_Algorithm/CRC.c
    ${SHARED}/1_Middleware/2_Algorithm/Kalman.c
    ${SHARED}/1_Middleware/2_Algorithm/LESO.c
    ${SHARED}/1_Middleware/2_Algorithm/LQR.c
    ${SHARED}/1_Middleware/2_Algorithm/PID.c
    ${SHARED}/1_Middleware/2_Algorithm/QuaternionEKF.c
    ${SHARED}/2_Device/IMU/BMI088/device_bmi088.c
    ${SHARED}/2_Device/Motor/Motor_DM/device_motor_dm.c
    ${SHARED}/2_Device/Motor/Motor_DJI/device_motor_dji.c
    ${SHARED}/2_Device/Communication/USB/device_usb_protocol.c
    ${SHARED}/2_Device/Communication/DR16/device_dr16.c
    ${SHARED}/2_Device/Communication/Board/device_board.c
    ${SHARED}/3_Chariot/1_Module/Gimbal/gimbal_config.c
    ${SHARED}/3_Chariot/1_Module/Gimbal/gimbal_control.c
    ${SHARED}/5_Task/task_can.c
    ${SHARED}/5_Task/task_can_dispatch.c
    ${SHARED}/5_Task/task_imu.c
    ${SHARED}/5_Task/task_remote.c
    ${SHARED}/5_Task/task_usb.c
    ${SHARED}/5_Task/gimbal_task.c
    # Kalman 和 QuaternionEKF 只用到这 6 个矩阵函数，不整包编 CMSIS-DSP。
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Middlewares/ST/ARM/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${SHARED}/1_Middleware/0_Common
    ${SHARED}/1_Middleware/1_Driver/FDCAN
    ${SHARED}/1_Middleware/1_Driver/SPI
    ${SHARED}/1_Middleware/1_Driver/UART
    ${SHARED}/1_Middleware/1_Driver/USB
    ${SHARED}/1_Middleware/2_Algorithm
    ${SHARED}/2_Device/IMU/BMI088
    ${SHARED}/2_Device/Motor/Motor_DM
    ${SHARED}/2_Device/Motor/Motor_DJI
    ${SHARED}/2_Device/Communication/USB
    ${SHARED}/2_Device/Communication/DR16
    ${SHARED}/2_Device/Communication/Board
    ${SHARED}/5_Task
    ${SHARED}/3_Chariot/1_Module/Gimbal
    # CMSIS-DSP：Kalman 和 QuaternionEKF 依赖 arm_matrix_instance_f32。
    # 源码随工程自带，不走 X-CUBE-ALGOBUILD pack。
    Middlewares/ST/ARM/DSP/Include
    Middlewares/ST/ARM/DSP/PrivateInclude
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    DISABLEFLOAT16
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
