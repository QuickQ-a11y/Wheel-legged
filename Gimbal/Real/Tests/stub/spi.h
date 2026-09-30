/*
 * 主机单测桩：顶替 CubeMX 的 spi.h。
 *
 * 真实的 task_imu.h -> device_bmi088.h -> driver_spi.h 只需要这两个类型名，
 * 桩在这一层打，IMU 状态结构体 task_imu_state_t 就能用真的，不必另抄一份。
 */
#ifndef TEST_STUB_SPI_H
#define TEST_STUB_SPI_H

typedef struct
{
    int dummy;
} GPIO_TypeDef;

typedef struct
{
    int dummy;
} SPI_HandleTypeDef;

#endif
