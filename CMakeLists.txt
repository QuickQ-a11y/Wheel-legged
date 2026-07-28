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
enable_language(C ASM)

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
    User_File/1_Middleware/1_Driver/USB/driver_usb.c
    User_File/1_Middleware/2_Algorithm/CRC.c
    User_File/1_Middleware/2_Algorithm/Kalman.c
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
    User_File/3_Chariot/1_Module/Chassis/chassis_config.c
    User_File/3_Chariot/1_Module/Chassis/chassis_control.c
    User_File/3_Chariot/1_Module/Chassis/chassis_vmc.c
    User_File/5_Task/task_can.c
    User_File/5_Task/chassis_task.c
    User_File/5_Task/task_imu.c
    User_File/5_Task/task_can_dispatch.c
    User_File/5_Task/task_usb.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User_File/1_Middleware/0_Common
    User_File/1_Middleware/1_Driver/FDCAN
    User_File/1_Middleware/1_Driver/SPI
    User_File/1_Middleware/1_Driver/USB
    User_File/1_Middleware/2_Algorithm
    User_File/2_Device/IMU/BMI088
    User_File/2_Device/Motor/Motor_DM
    User_File/2_Device/Motor/Motor_DJI
    User_File/2_Device/Communication/USB
    User_File/3_Chariot/1_Module/Chassis
    User_File/5_Task
    Middlewares/ST/ARM/DSP/Include
    Middlewares/ST/ARM/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
    DISABLEFLOAT16
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
