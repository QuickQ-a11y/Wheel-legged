#ifndef CRC_H
#define CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ALGORITHM_CRC8_INIT 0xFFU
#define ALGORITHM_CRC16_INIT 0xFFFFU

/**
 * @brief 计算多项式 G(x)=x8+x5+x4+1 的 CRC8。
 */
uint8_t Algorithm_CRC8_Calculate(const uint8_t *data,
                                 uint32_t length,
                                 uint8_t initialValue);

/**
 * @brief 计算与裁判系统协议一致的 CRC16。
 */
uint16_t Algorithm_CRC16_Calculate(const uint8_t *data,
                                   uint32_t length,
                                   uint16_t initialValue);

#ifdef __cplusplus
}
#endif

#endif
