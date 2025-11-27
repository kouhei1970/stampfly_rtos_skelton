/**
 * @file vl53lx_platform.h
 * @brief VL53LX Platform Layer - ESP-IDF I2C Master API Implementation
 *
 * This file provides the platform-specific I2C communication functions
 * required by the VL53LX driver for ESP32-S3.
 */

#ifndef _VL53LX_PLATFORM_H_
#define _VL53LX_PLATFORM_H_

#include <stdint.h>
#include <stddef.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VL53LX device handle structure
 */
typedef struct {
    uint16_t i2c_slave_address;           /*!< I2C slave address (7-bit) */
    i2c_master_dev_handle_t i2c_dev_handle; /*!< ESP-IDF I2C device handle */
} VL53LX_Dev_t;

typedef VL53LX_Dev_t *VL53LX_DEV;

/**
 * @brief Error codes
 */
#define VL53LX_ERROR_NONE                    ((int8_t)  0)
#define VL53LX_ERROR_TIME_OUT                ((int8_t) -1)
#define VL53LX_ERROR_CONTROL_INTERFACE       ((int8_t) -2)
#define VL53LX_ERROR_INVALID_PARAMS          ((int8_t) -3)
#define VL53LX_ERROR_NOT_SUPPORTED           ((int8_t) -4)
#define VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL  ((int8_t) -5)

/**
 * @brief Initialize VL53LX platform (I2C device)
 *
 * @param pdev           Pointer to device handle
 * @param bus_handle     I2C master bus handle
 * @param device_address I2C device address (7-bit)
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_PlatformInit(VL53LX_DEV pdev, i2c_master_bus_handle_t bus_handle, uint16_t device_address);

/**
 * @brief Deinitialize VL53LX platform
 *
 * @param pdev Pointer to device handle
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_PlatformDeinit(VL53LX_DEV pdev);

/**
 * @brief Write multiple bytes to VL53LX device
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param pdata      Pointer to data buffer
 * @param count      Number of bytes to write
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_WriteMulti(VL53LX_DEV pdev, uint16_t index, uint8_t *pdata, uint32_t count);

/**
 * @brief Read multiple bytes from VL53LX device
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param pdata      Pointer to data buffer
 * @param count      Number of bytes to read
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_ReadMulti(VL53LX_DEV pdev, uint16_t index, uint8_t *pdata, uint32_t count);

/**
 * @brief Write single byte to VL53LX device
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param data       Byte to write
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_WriteByte(VL53LX_DEV pdev, uint16_t index, uint8_t data);

/**
 * @brief Write 16-bit word to VL53LX device (big-endian)
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param data       Word to write
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_WriteWord(VL53LX_DEV pdev, uint16_t index, uint16_t data);

/**
 * @brief Write 32-bit double word to VL53LX device (big-endian)
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param data       Double word to write
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_WriteDWord(VL53LX_DEV pdev, uint16_t index, uint32_t data);

/**
 * @brief Read single byte from VL53LX device
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param pdata      Pointer to byte storage
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_ReadByte(VL53LX_DEV pdev, uint16_t index, uint8_t *pdata);

/**
 * @brief Read 16-bit word from VL53LX device (big-endian)
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param pdata      Pointer to word storage
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_ReadWord(VL53LX_DEV pdev, uint16_t index, uint16_t *pdata);

/**
 * @brief Read 32-bit double word from VL53LX device (big-endian)
 *
 * @param pdev       Pointer to device handle
 * @param index      Register address (16-bit)
 * @param pdata      Pointer to double word storage
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_ReadDWord(VL53LX_DEV pdev, uint16_t index, uint32_t *pdata);

/**
 * @brief Wait for specified time in milliseconds
 *
 * @param pdev       Pointer to device handle
 * @param wait_ms    Time to wait in milliseconds
 * @return Status code (VL53LX_ERROR_NONE on success)
 */
int8_t VL53LX_WaitMs(VL53LX_DEV pdev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* _VL53LX_PLATFORM_H_ */
