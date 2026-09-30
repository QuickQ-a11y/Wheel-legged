/*
 * 主机单测桩：顶替 CubeMX 的 HAL 总头文件。
 *
 * 业务代码在主机上只用到 HAL_GetTick，这里只声明，符号由各测试文件自己定义，
 * 这样每个测试都能自己控制时间。
 */
#ifndef TEST_STUB_STM32H7XX_HAL_H
#define TEST_STUB_STM32H7XX_HAL_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif
