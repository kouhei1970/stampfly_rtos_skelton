/**
 * @file vl53l3cx_wrapper.hpp
 * @brief VL53L3CX ToF Sensor Driver for StampFly
 *
 * I2C interface driver for ST VL53L3CX Time-of-Flight sensor
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class VL53L3CX {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr;
        gpio_num_t xshut_pin;
        gpio_num_t int_pin;
    };

    struct RangeData {
        uint16_t distance_mm;       // Distance in mm
        uint8_t range_status;       // Status code
        uint8_t signal_rate;        // Signal strength
        int64_t timestamp_us;       // Timestamp
        bool valid;                 // True if measurement is valid
    };

    VL53L3CX();
    ~VL53L3CX();

    /**
     * @brief Initialize the sensor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read distance measurement
     * @param data Output range data structure
     * @return ESP_OK on success
     */
    esp_err_t read(RangeData& data);

    /**
     * @brief Read distance (simple version)
     * @param distance_mm Output distance in mm
     * @return ESP_OK on success
     */
    esp_err_t read(float& distance_mm);

    /**
     * @brief Start continuous ranging
     * @param period_ms Measurement period in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t startRanging(uint32_t period_ms = 33);

    /**
     * @brief Stop continuous ranging
     * @return ESP_OK on success
     */
    esp_err_t stopRanging();

    /**
     * @brief Check if new data is available
     * @return true if data ready
     */
    bool isDataReady();

    /**
     * @brief Change I2C address (useful for multiple sensors)
     * @param new_addr New I2C address
     * @return ESP_OK on success
     */
    esp_err_t setAddress(uint8_t new_addr);

    /**
     * @brief Get sensor model ID
     * @return Model ID
     */
    uint16_t getModelId();

private:
    esp_err_t writeRegister(uint16_t reg, uint8_t value);
    esp_err_t writeRegister16(uint16_t reg, uint16_t value);
    esp_err_t writeRegister32(uint16_t reg, uint32_t value);
    esp_err_t readRegister(uint16_t reg, uint8_t* value);
    esp_err_t readRegister16(uint16_t reg, uint16_t* value);
    esp_err_t readRegisters(uint16_t reg, uint8_t* buffer, size_t len);

    esp_err_t loadDefaultConfig();
    esp_err_t waitForBoot();
    esp_err_t clearInterrupt();

    i2c_port_t i2c_port_;
    uint8_t i2c_addr_;
    gpio_num_t xshut_pin_;
    gpio_num_t int_pin_;
    SemaphoreHandle_t mutex_;
    bool initialized_;
    bool ranging_;

    // VL53L3CX Register addresses
    static constexpr uint16_t REG_SOFT_RESET = 0x0000;
    static constexpr uint16_t REG_MODEL_ID = 0x010F;
    static constexpr uint16_t REG_MODULE_TYPE = 0x0110;
    static constexpr uint16_t REG_FIRMWARE_SYSTEM_STATUS = 0x00E5;
    static constexpr uint16_t REG_PAD_I2C_HV_CONFIG = 0x002D;
    static constexpr uint16_t REG_GPIO_HV_MUX_CTRL = 0x0030;
    static constexpr uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
    static constexpr uint16_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0086;
    static constexpr uint16_t REG_SYSTEM_MODE_START = 0x0087;
    static constexpr uint16_t REG_RESULT_RANGE_STATUS = 0x0089;
    static constexpr uint16_t REG_RESULT_FINAL_RANGE = 0x0096;
    static constexpr uint16_t REG_I2C_SLAVE_DEVICE_ADDRESS = 0x0001;
    static constexpr uint16_t REG_RANGE_CONFIG_TIMEOUT_MACROP_A = 0x005E;
    static constexpr uint16_t REG_RANGE_CONFIG_TIMEOUT_MACROP_B = 0x0061;

    // Status codes
    static constexpr uint8_t RANGE_STATUS_VALID = 0;
    static constexpr uint8_t MODEL_ID_VL53L3CX = 0xEB;
};

}  // namespace stampfly
