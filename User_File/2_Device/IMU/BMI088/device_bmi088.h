#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver_spi.h"

#include <stdint.h>

#define BMI088_AXIS_COUNT 3U

#define BMI088_ERROR_NONE 0x00000000UL
#define BMI088_ERROR_CONFIG 0x00000001UL
#define BMI088_ERROR_ACC_CHIP_ID 0x00000002UL
#define BMI088_ERROR_GYRO_CHIP_ID 0x00000004UL
#define BMI088_ERROR_ACC_POWER 0x00000008UL
#define BMI088_ERROR_ACC_CONFIG 0x00000010UL
#define BMI088_ERROR_GYRO_CONFIG 0x00000020UL
#define BMI088_ERROR_SPI_TRANSFER 0x80000000UL

typedef struct
{
    SPI_HandleTypeDef *spiHandle;                   /* BMI088 使用的 SPI 句柄。 */
    driver_spi_chip_select_t accChipSelect;            /* 加速度计片选。 */
    driver_spi_chip_select_t gyroChipSelect;           /* 陀螺仪片选。 */
    uint32_t timeoutMs;                             /* 阻塞 SPI 超时时间，单位 ms。 */
} bmi088_config_t;

typedef struct
{
    int16_t accRaw[BMI088_AXIS_COUNT];              /* 加速度原始值。 */
    int16_t gyroRaw[BMI088_AXIS_COUNT];             /* 角速度原始值。 */
    int16_t temperatureRaw;                         /* 温度原始值。 */
    float accMps2[BMI088_AXIS_COUNT];               /* 加速度，单位 m/s^2。 */
    float gyroRadps[BMI088_AXIS_COUNT];             /* 角速度，单位 rad/s。 */
    float temperatureCelsius;                       /* 温度，单位摄氏度。 */
    uint8_t accChipId;                              /* 加速度计芯片 ID。 */
    uint8_t gyroChipId;                             /* 陀螺仪芯片 ID。 */
    uint32_t sampleCount;                           /* 成功读取样本数量。 */
    uint32_t lastUpdateTick;                        /* 最近一次成功读取 HAL tick。 */
} bmi088_data_t;

typedef struct
{
    bmi088_config_t config;                         /* 当前设备配置。 */
    bmi088_data_t data;                             /* 最近一次有效数据。 */
    uint32_t lastErrorCode;                         /* 最近一次错误码。 */
} bmi088_t;

/**
 * @brief 初始化 BMI088 并配置量程、输出频率和数据就绪中断。
 *
 * 当前配置为 ACC 6G/1600Hz，GYRO 2000dps/230Hz。
 */
void BMI088_Init(bmi088_t *bmi088, const bmi088_config_t *config);

/**
 * @brief 阻塞读取 BMI088 加速度、角速度和温度数据。
 */
void BMI088_Read(bmi088_t *bmi088, bmi088_data_t *data);

/**
 * @brief 获取最近一次初始化或读取错误码。
 */
uint32_t BMI088_GetErrorCode(const bmi088_t *bmi088);

#ifdef __cplusplus
}
#endif

#endif
