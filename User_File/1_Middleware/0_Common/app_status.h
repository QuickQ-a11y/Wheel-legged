#ifndef APP_STATUS_H
#define APP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 应用层统一返回状态。
 *
 * HAL 返回值只在驱动内部使用，模块公共接口统一返回该类型，避免上层依赖具体硬件库。
 */
typedef enum
{
    APP_STATUS_OK = 0,
    APP_STATUS_ERROR,
    APP_STATUS_BUSY,
    APP_STATUS_TIMEOUT,
    APP_STATUS_INVALID_PARAM,
    APP_STATUS_NOT_READY,
    APP_STATUS_NO_RESOURCE,
} app_status_t;

#ifdef __cplusplus
}
#endif

#endif
