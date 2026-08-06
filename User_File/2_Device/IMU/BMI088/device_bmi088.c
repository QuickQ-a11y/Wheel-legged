#include "device_bmi088.h"

#include <string.h>

#define BMI088_DEFAULT_TIMEOUT_MS 10U
#define BMI088_READ_BIT 0x80U
#define BMI088_SPI_MAX_DATA_LENGTH 6U
#define BMI088_ACC_READ_OVERHEAD 2U
#define BMI088_GYRO_READ_OVERHEAD 1U

#define BMI088_ACC_CHIP_ID_REG 0x00U
#define BMI088_ACC_CHIP_ID_VALUE 0x1EU
#define BMI088_ACC_XOUT_L_REG 0x12U
#define BMI088_ACC_TEMP_M_REG 0x22U
#define BMI088_ACC_CONF_REG 0x40U
#define BMI088_ACC_RANGE_REG 0x41U
#define BMI088_ACC_INT1_IO_CTRL_REG 0x53U
#define BMI088_ACC_INT_MAP_DATA_REG 0x58U
#define BMI088_ACC_PWR_CONF_REG 0x7CU
#define BMI088_ACC_PWR_CTRL_REG 0x7DU
#define BMI088_ACC_SOFTRESET_REG 0x7EU

#define BMI088_ACC_SOFTRESET_VALUE 0xB6U
#define BMI088_ACC_ENABLE 0x04U
#define BMI088_ACC_POWER_ACTIVE 0x00U
#define BMI088_ACC_CONF_MUST_SET 0x80U
#define BMI088_ACC_NORMAL_MODE 0x20U
#define BMI088_ACC_ODR_1600_HZ 0x0CU
#define BMI088_ACC_RANGE_6G 0x01U
#define BMI088_ACC_INT1_ENABLE 0x08U
#define BMI088_ACC_INT1_PUSH_PULL 0x00U
#define BMI088_ACC_INT1_ACTIVE_HIGH 0x02U
#define BMI088_ACC_INT1_DRDY 0x04U

#define BMI088_GYRO_CHIP_ID_REG 0x00U
#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU
#define BMI088_GYRO_X_L_REG 0x02U
#define BMI088_GYRO_RANGE_REG 0x0FU
#define BMI088_GYRO_BANDWIDTH_REG 0x10U
#define BMI088_GYRO_LPM1_REG 0x11U
#define BMI088_GYRO_SOFTRESET_REG 0x14U
#define BMI088_GYRO_CTRL_REG 0x15U
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG 0x16U
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG 0x18U

#define BMI088_GYRO_SOFTRESET_VALUE 0xB6U
#define BMI088_GYRO_RANGE_2000_DPS 0x00U
#define BMI088_GYRO_BANDWIDTH_MUST_SET 0x80U
#define BMI088_GYRO_ODR_2000_BW_230_HZ 0x01U
#define BMI088_GYRO_NORMAL_MODE 0x00U
#define BMI088_GYRO_DRDY_ENABLE 0x80U
#define BMI088_GYRO_INT3_PUSH_PULL 0x00U
#define BMI088_GYRO_INT3_ACTIVE_HIGH 0x01U
#define BMI088_GYRO_DRDY_TO_INT3 0x01U

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f
#define BMI088_ACC_6G_SCALE 0.00179443359375f
#define BMI088_GYRO_2000DPS_SCALE 0.0010652644360316953f

typedef struct
{
    uint8_t reg;                 /* 待写寄存器地址。 */
    uint8_t value;               /* 寄存器目标值。 */
    uint32_t errorCode;          /* 写入或校验失败时记录的错误码。 */
} bmi088_register_config_t;

static const bmi088_register_config_t bmi088AccInitTable[] = {
    {BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_ENABLE, BMI088_ERROR_ACC_POWER},
    {BMI088_ACC_PWR_CONF_REG, BMI088_ACC_POWER_ACTIVE, BMI088_ERROR_ACC_POWER},
    {BMI088_ACC_CONF_REG, BMI088_ACC_CONF_MUST_SET | BMI088_ACC_NORMAL_MODE | BMI088_ACC_ODR_1600_HZ, BMI088_ERROR_ACC_CONFIG},
    {BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_6G, BMI088_ERROR_ACC_CONFIG},
    {BMI088_ACC_INT1_IO_CTRL_REG, BMI088_ACC_INT1_ENABLE | BMI088_ACC_INT1_PUSH_PULL | BMI088_ACC_INT1_ACTIVE_HIGH, BMI088_ERROR_ACC_CONFIG},
    {BMI088_ACC_INT_MAP_DATA_REG, BMI088_ACC_INT1_DRDY, BMI088_ERROR_ACC_CONFIG},
};

static const bmi088_register_config_t bmi088GyroInitTable[] = {
    {BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000_DPS, BMI088_ERROR_GYRO_CONFIG},
    {BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BANDWIDTH_MUST_SET | BMI088_GYRO_ODR_2000_BW_230_HZ, BMI088_ERROR_GYRO_CONFIG},
    {BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE, BMI088_ERROR_GYRO_CONFIG},
    {BMI088_GYRO_CTRL_REG, BMI088_GYRO_DRDY_ENABLE, BMI088_ERROR_GYRO_CONFIG},
    {BMI088_GYRO_INT3_INT4_IO_CONF_REG, BMI088_GYRO_INT3_PUSH_PULL | BMI088_GYRO_INT3_ACTIVE_HIGH, BMI088_ERROR_GYRO_CONFIG},
    {BMI088_GYRO_INT3_INT4_IO_MAP_REG, BMI088_GYRO_DRDY_TO_INT3, BMI088_ERROR_GYRO_CONFIG},
};

/**
 * @brief 按 BMI088 温度寄存器格式解析有符号 11 位温度原始值。
 */
static int16_t BMI088_MakeTemperatureRaw(const uint8_t data[2])
{
    uint16_t raw = (uint16_t)(((uint16_t)data[0] << 3U) | ((uint16_t)data[1] >> 5U));

    if (raw > 1023U)
    {
        return (int16_t)((int32_t)raw - 2048);
    }

    return (int16_t)raw;
}

/**
 * @brief 读取加速度计寄存器。
 *
 * ACC 读协议需要“命令 + dummy + 数据”处在同一次 CS 有效窗口内。
 */
static void BMI088_ReadAccRegisters(bmi088_t *bmi088,
                                    uint8_t reg,
                                    uint8_t *data,
                                    uint8_t length)
{
    uint8_t txBuffer[BMI088_SPI_MAX_DATA_LENGTH + BMI088_ACC_READ_OVERHEAD] = {0};
    uint8_t rxBuffer[BMI088_SPI_MAX_DATA_LENGTH + BMI088_ACC_READ_OVERHEAD] = {0};

    /* 收发缓冲为固定栈数组，长度超出容量时不得发起传输。 */
    if (length > BMI088_SPI_MAX_DATA_LENGTH)
    {
        return;
    }

    txBuffer[0] = BMI088_READ_BIT | reg;
    Driver_SPI_TransmitReceive(bmi088->config.spiHandle,
                               &bmi088->config.accChipSelect,
                               txBuffer,
                               rxBuffer,
                               (uint16_t)(length + BMI088_ACC_READ_OVERHEAD),
                               bmi088->config.timeoutMs);

    memcpy(data, &rxBuffer[BMI088_ACC_READ_OVERHEAD], length);
}

/**
 * @brief 读取陀螺仪寄存器。
 *
 * GYRO 读协议为“命令 + 数据”，没有加速度计额外 dummy 字节。
 */
static void BMI088_ReadGyroRegisters(bmi088_t *bmi088,
                                     uint8_t reg,
                                     uint8_t *data,
                                     uint8_t length)
{
    uint8_t txBuffer[BMI088_SPI_MAX_DATA_LENGTH + BMI088_GYRO_READ_OVERHEAD] = {0};
    uint8_t rxBuffer[BMI088_SPI_MAX_DATA_LENGTH + BMI088_GYRO_READ_OVERHEAD] = {0};

    /* 收发缓冲为固定栈数组，长度超出容量时不得发起传输。 */
    if (length > BMI088_SPI_MAX_DATA_LENGTH)
    {
        return;
    }

    txBuffer[0] = BMI088_READ_BIT | reg;
    Driver_SPI_TransmitReceive(bmi088->config.spiHandle,
                               &bmi088->config.gyroChipSelect,
                               txBuffer,
                               rxBuffer,
                               (uint16_t)(length + BMI088_GYRO_READ_OVERHEAD),
                               bmi088->config.timeoutMs);

    memcpy(data, &rxBuffer[BMI088_GYRO_READ_OVERHEAD], length);
}

static void BMI088_WriteAccRegister(bmi088_t *bmi088,
                                    uint8_t reg,
                                    uint8_t value)
{
    uint8_t txBuffer[2] = {reg, value};

    Driver_SPI_Transmit(bmi088->config.spiHandle,
                        &bmi088->config.accChipSelect,
                        txBuffer,
                        sizeof(txBuffer),
                        bmi088->config.timeoutMs);
}

static void BMI088_WriteGyroRegister(bmi088_t *bmi088,
                                     uint8_t reg,
                                     uint8_t value)
{
    uint8_t txBuffer[2] = {reg, value};

    Driver_SPI_Transmit(bmi088->config.spiHandle,
                        &bmi088->config.gyroChipSelect,
                        txBuffer,
                        sizeof(txBuffer),
                        bmi088->config.timeoutMs);
}

/**
 * @brief 写入并回读校验加速度计寄存器，回读不一致时记录该寄存器的错误位。
 */
static void BMI088_WriteAndCheckAccRegister(bmi088_t *bmi088,
                                            const bmi088_register_config_t *config)
{
    uint8_t readValue = 0U;

    BMI088_WriteAccRegister(bmi088, config->reg, config->value);
    HAL_Delay(1U);
    BMI088_ReadAccRegisters(bmi088, config->reg, &readValue, 1U);
    if (readValue != config->value)
    {
        bmi088->lastErrorCode |= config->errorCode;
    }
}

/**
 * @brief 写入并回读校验陀螺仪寄存器，回读不一致时记录该寄存器的错误位。
 */
static void BMI088_WriteAndCheckGyroRegister(bmi088_t *bmi088,
                                             const bmi088_register_config_t *config)
{
    uint8_t readValue = 0U;

    BMI088_WriteGyroRegister(bmi088, config->reg, config->value);
    HAL_Delay(1U);
    BMI088_ReadGyroRegisters(bmi088, config->reg, &readValue, 1U);
    if (readValue != config->value)
    {
        bmi088->lastErrorCode |= config->errorCode;
    }
}

/**
 * @brief 初始化 BMI088 加速度计部分。
 */
static void BMI088_InitAcc(bmi088_t *bmi088)
{
    uint8_t chipId = 0U;
    uint32_t index;

    /*
     * ACC 上电后第一次 SPI 读可能只完成接口模式切换。
     * 这里按本家代码连续读两次，后续只用第二次结果判断芯片 ID。
     */
    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);
    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);

    BMI088_WriteAccRegister(bmi088, BMI088_ACC_SOFTRESET_REG, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(150U);

    /*
     * 软复位后也连续读两次，避免复位恢复阶段的首帧无效值影响初始化判断。
     */
    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);
    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_CHIP_ID_REG, &chipId, 1U);

    bmi088->data.accChipId = chipId;
    if (chipId != BMI088_ACC_CHIP_ID_VALUE)
    {
        bmi088->lastErrorCode |= BMI088_ERROR_ACC_CHIP_ID;
        return;
    }

    for (index = 0U; index < (sizeof(bmi088AccInitTable) / sizeof(bmi088AccInitTable[0])); index++)
    {
        BMI088_WriteAndCheckAccRegister(bmi088, &bmi088AccInitTable[index]);
    }
}

/**
 * @brief 初始化 BMI088 陀螺仪部分。
 */
static void BMI088_InitGyro(bmi088_t *bmi088)
{
    uint8_t chipId = 0U;
    uint32_t index;

    /*
     * GYRO 没有 ACC 的额外 dummy 字节，但本家代码同样在复位前后各读两次，
     * 这里保持相同初始化节奏，方便对照实机问题。
     */
    BMI088_ReadGyroRegisters(bmi088, BMI088_GYRO_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);
    BMI088_ReadGyroRegisters(bmi088, BMI088_GYRO_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);

    BMI088_WriteGyroRegister(bmi088, BMI088_GYRO_SOFTRESET_REG, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(80U);

    BMI088_ReadGyroRegisters(bmi088, BMI088_GYRO_CHIP_ID_REG, &chipId, 1U);
    HAL_Delay(1U);
    BMI088_ReadGyroRegisters(bmi088, BMI088_GYRO_CHIP_ID_REG, &chipId, 1U);

    bmi088->data.gyroChipId = chipId;
    if (chipId != BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088->lastErrorCode |= BMI088_ERROR_GYRO_CHIP_ID;
        return;
    }

    for (index = 0U; index < (sizeof(bmi088GyroInitTable) / sizeof(bmi088GyroInitTable[0])); index++)
    {
        BMI088_WriteAndCheckGyroRegister(bmi088, &bmi088GyroInitTable[index]);
    }
}

void BMI088_Init(bmi088_t *bmi088, const bmi088_config_t *config)
{
    memset(bmi088, 0, sizeof(*bmi088));
    bmi088->config = *config;
    if (bmi088->config.timeoutMs == 0U)
    {
        bmi088->config.timeoutMs = BMI088_DEFAULT_TIMEOUT_MS;
    }

    HAL_GPIO_WritePin(bmi088->config.accChipSelect.gpioPort,
                      bmi088->config.accChipSelect.gpioPin,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(bmi088->config.gyroChipSelect.gpioPort,
                      bmi088->config.gyroChipSelect.gpioPin,
                      GPIO_PIN_SET);
    HAL_Delay(10U);

    BMI088_InitAcc(bmi088);
    BMI088_InitGyro(bmi088);
}

void BMI088_Read(bmi088_t *bmi088, bmi088_data_t *data)
{
    uint8_t accBuffer[6] = {0};
    uint8_t gyroBuffer[6] = {0};
    uint8_t tempBuffer[2] = {0};
    uint32_t axis;

    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_XOUT_L_REG, accBuffer, sizeof(accBuffer));
    BMI088_ReadGyroRegisters(bmi088, BMI088_GYRO_X_L_REG, gyroBuffer, sizeof(gyroBuffer));
    BMI088_ReadAccRegisters(bmi088, BMI088_ACC_TEMP_M_REG, tempBuffer, sizeof(tempBuffer));

    /* 寄存器为低字节在前的 16 位补码。 */
    for (axis = 0U; axis < BMI088_AXIS_COUNT; axis++)
    {
        bmi088->data.accRaw[axis] = (int16_t)(((uint16_t)accBuffer[axis * 2U + 1U] << 8U) |
                                              (uint16_t)accBuffer[axis * 2U]);
        bmi088->data.gyroRaw[axis] = (int16_t)(((uint16_t)gyroBuffer[axis * 2U + 1U] << 8U) |
                                               (uint16_t)gyroBuffer[axis * 2U]);
        bmi088->data.accMps2[axis] = (float)bmi088->data.accRaw[axis] * BMI088_ACC_6G_SCALE;
        bmi088->data.gyroRadps[axis] = (float)bmi088->data.gyroRaw[axis] * BMI088_GYRO_2000DPS_SCALE;
    }

    bmi088->data.temperatureRaw = BMI088_MakeTemperatureRaw(tempBuffer);
    bmi088->data.temperatureCelsius =
        (float)bmi088->data.temperatureRaw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    bmi088->data.sampleCount++;
    bmi088->data.lastUpdateTick = HAL_GetTick();
    bmi088->lastErrorCode = BMI088_ERROR_NONE;

    *data = bmi088->data;
}
