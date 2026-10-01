#!/usr/bin/env bash
#
# 云台全部主机单测，按 Gimbal/CLAUDE.md §主机单测配方 编译运行。配方本身没变，
# 只是每次会话都要重拼一遍、放在 scratchpad 里的临时脚本又丢过两次，固化在这里。
#
#   ./Tests/run_host_tests.sh            全部 17 遍
#   ./Tests/run_host_tests.sh shooter    只跑名字里带 shooter 的（test_shooter 的 5 个变体）
#
# 三条规矩（都吃过亏）：
#   1. 每个二进制编译前先删，gcc 退出码非 0 就不运行——旧二进制冒充通过不止一次。
#   2. 编译出警告也算失败，和固件"零警告"同一条线。
#   3. test_shooter 的两个输出门用 sed【强制】写成要的值，不假设工作区当前是几；
#      -DTEST_FRIC_FLAG/-DTEST_TRIGGER_FLAG 声明期望的门，sed 失配时 main 第一行就会断言失败。
#
# test_motor_lk 跑 2 遍（编码器方向 ±1）、test_can_dispatch 两种拨盘各 1 遍、
# test_shooter 跑 5 遍（M2006 门全开 / 翻转送弹方向 / 只开拨盘门，翎控门全开 / 翻转送弹方向）。
# 每一遍为什么不能省，见 Gimbal/CLAUDE.md。
#
set -u
cd "$(dirname "$0")/.."
ONLY=${1:-}
O=$(mktemp -d); trap 'rm -rf "$O"' EXIT

STUB="-ITests/stub -IUser_File/5_Task"
INC="-IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/Communication/DR16 -IUser_File/2_Device/Communication/USB \
-IUser_File/2_Device/Communication/FlySky"
W="-Wall -Wextra"
fail=0
npass=0

# run <名字> <gcc 参数...>：先删旧二进制 -> 编译 -> 查退出码和警告 -> 运行 -> 查退出码
run() {
    local name=$1
    shift
    if [ -n "$ONLY" ] && [[ "$name" != *"$ONLY"* ]]; then
        return
    fi
    local bin="$O/$name"
    rm -f "$bin"
    if ! gcc $W "$@" -o "$bin" -lm 2>"$O/$name.gcc.log"; then
        echo "COMPILE-FAIL $name"
        cat "$O/$name.gcc.log"
        fail=1
        return
    fi
    if [ -s "$O/$name.gcc.log" ]; then
        echo "WARN-FAIL $name"
        cat "$O/$name.gcc.log"
        fail=1
        return
    fi
    if "$bin" >"$O/$name.run.log" 2>&1; then
        echo "PASS $name: $(tail -1 "$O/$name.run.log")"
        npass=$((npass + 1))
    else
        echo "RUN-FAIL $name"
        tail -5 "$O/$name.run.log"
        fail=1
    fi
}

run test_angle $INC Tests/test_angle.c User_File/1_Middleware/2_Algorithm/Angle.c
run test_dr16 $INC Tests/test_dr16.c User_File/2_Device/Communication/DR16/device_dr16.c
run test_ibus $INC Tests/test_ibus.c User_File/2_Device/Communication/FlySky/device_ibus.c
run test_sbus $INC Tests/test_sbus.c User_File/2_Device/Communication/FlySky/device_sbus.c
run test_usb_protocol $INC Tests/test_usb_protocol.c \
    User_File/2_Device/Communication/USB/device_usb_protocol.c User_File/1_Middleware/2_Algorithm/CRC.c
run test_motor_dji $STUB $INC -IUser_File/2_Device/Motor/Motor_DJI \
    Tests/test_motor_dji.c User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c

LK="$STUB $INC -IUser_File/2_Device/Motor/Motor_LK Tests/test_motor_lk.c \
User_File/2_Device/Motor/Motor_LK/device_motor_lk.c"
run test_motor_lk_dirP $LK
run test_motor_lk_dirN '-DAPP_LK_ENCODER_DIR=(-1.0f)' $LK

DISP="$STUB $INC -IUser_File/2_Device/Motor/Motor_DM -IUser_File/2_Device/Motor/Motor_DJI \
-IUser_File/2_Device/Motor/Motor_LK Tests/test_can_dispatch.c User_File/5_Task/task_can_dispatch.c \
User_File/2_Device/Motor/Motor_DM/device_motor_dm.c User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c \
User_File/2_Device/Motor/Motor_LK/device_motor_lk.c User_File/1_Middleware/2_Algorithm/Angle.c"
run test_can_dispatch_m2006 -DAPP_TRIGGER_MOTOR=APP_TRIGGER_M2006 $DISP
run test_can_dispatch_lk -DAPP_TRIGGER_MOTOR=APP_TRIGGER_LK4005 $DISP

run test_gimbal_fault $STUB $INC -IUser_File/1_Middleware/1_Driver/SPI -IUser_File/2_Device/IMU/BMI088 \
    -IUser_File/2_Device/Motor/Motor_DM -IUser_File/2_Device/Communication/Board \
    -IUser_File/3_Chariot/1_Module/Gimbal Tests/test_gimbal_fault.c \
    User_File/3_Chariot/1_Module/Gimbal/gimbal_control.c User_File/3_Chariot/1_Module/Gimbal/gimbal_config.c \
    User_File/1_Middleware/2_Algorithm/PID.c User_File/1_Middleware/2_Algorithm/Angle.c

CFG=User_File/3_Chariot/1_Module/Shooter/shooter_config.c
sed -e 's/\.fric_flag = 0U,/.fric_flag = 1U,/' \
    -e 's/\.trigger_flag = 0U,/.trigger_flag = 1U,/' $CFG >"$O/shooter_cfg_open.c"
sed -e 's/\.fric_flag = 1U,/.fric_flag = 0U,/' \
    -e 's/\.trigger_flag = 0U,/.trigger_flag = 1U,/' $CFG >"$O/shooter_cfg_trig.c"
SH="-ITests/stub -IUser_File/5_Task -IUser_File/1_Middleware/0_Common -IUser_File/1_Middleware/2_Algorithm \
-IUser_File/2_Device/Communication/FlySky -IUser_File/2_Device/Motor/Motor_DJI -IUser_File/2_Device/Motor/Motor_LK \
-IUser_File/3_Chariot/1_Module/Shooter Tests/test_shooter.c User_File/3_Chariot/1_Module/Shooter/shooter_control.c"
SHLIB="User_File/2_Device/Motor/Motor_DJI/device_motor_dji.c User_File/2_Device/Motor/Motor_LK/device_motor_lk.c \
User_File/1_Middleware/2_Algorithm/PID.c"
M2006="-DAPP_TRIGGER_MOTOR=APP_TRIGGER_M2006"
LK4005="-DAPP_TRIGGER_MOTOR=APP_TRIGGER_LK4005"
DIRN='-DAPP_SHOOT_TRIGGER_DIR=(-1.0f)'
run test_shooter_1_m2006_open $M2006 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1 $SH "$O/shooter_cfg_open.c" $SHLIB
run test_shooter_2_m2006_open_dirN $M2006 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1 "$DIRN" $SH \
    "$O/shooter_cfg_open.c" $SHLIB
run test_shooter_3_m2006_trig $M2006 -DTEST_FRIC_FLAG=0 -DTEST_TRIGGER_FLAG=1 $SH "$O/shooter_cfg_trig.c" $SHLIB
run test_shooter_4_lk_open $LK4005 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1 $SH "$O/shooter_cfg_open.c" $SHLIB
run test_shooter_5_lk_open_dirN $LK4005 -DTEST_FRIC_FLAG=1 -DTEST_TRIGGER_FLAG=1 "$DIRN" $SH \
    "$O/shooter_cfg_open.c" $SHLIB

# test_board_protocol 同时链两个工程的 device_board.c：两份同名头文件内容不同，必须分开调用 gcc。
if [ -z "$ONLY" ] || [[ "test_board_protocol" == *"$ONLY"* ]]; then
    if gcc -c $W -ITests/stub -IUser_File/5_Task -IUser_File/1_Middleware/0_Common \
           -IUser_File/1_Middleware/2_Algorithm -IUser_File/2_Device/Communication/Board \
           -o "$O/gimbal_board.o" User_File/2_Device/Communication/Board/device_board.c &&
       gcc -c $W -ITests/stub -I../../Chassis/User_File/1_Middleware/0_Common \
           -I../../Chassis/User_File/2_Device/Communication/Board -o "$O/chassis_board.o" \
           ../../Chassis/User_File/2_Device/Communication/Board/device_board.c; then
        run test_board_protocol -ITests/stub -IUser_File/1_Middleware/0_Common \
            Tests/test_board_protocol.c "$O/gimbal_board.o" "$O/chassis_board.o"
    else
        echo "COMPILE-FAIL test_board_protocol（两个 device_board.o）"
        fail=1
    fi
fi

echo "== $npass 遍通过，$([ $fail -eq 0 ] && echo 无失败 || echo 有失败)"
exit $fail
