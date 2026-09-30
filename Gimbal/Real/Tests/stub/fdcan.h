/*
 * 主机单测桩：顶替 CubeMX 的 fdcan.h。
 *
 * 句柄只用来比较地址（帧发到哪条总线），内容无意义。
 * 三个句柄都要声明：发射机构跨 FDCAN2/FDCAN3 两条总线，测试靠句柄区分。
 * 句柄对象由测试文件自己定义。
 */
#ifndef TEST_STUB_FDCAN_H
#define TEST_STUB_FDCAN_H

/*
 * 真的 fdcan.h 经 main.h 带进整个 HAL，device_motor_dm.c 的 HAL_GetTick 就是这么
 * 间接拿到声明的。桩照样带上，否则主机上会报隐式声明。
 */
#include "stm32h7xx_hal.h"

typedef struct
{
    int dummy;
} FDCAN_HandleTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#endif
