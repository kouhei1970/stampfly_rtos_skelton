/**
 * @file vl53l3cx_wrapper.hpp
 * @brief VL53L3CX ToF Sensor C++ Wrapper
 *
 * Dual sensor support (front/bottom), XSHUT control
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class VL53L3CXWrapper {
public:
    struct Config {
        int i2c_port;
        int sda_gpio;
        int scl_gpio;
        int xshut_gpio;
        uint8_t i2c_addr;
    };

    VL53L3CXWrapper() = default;
    ~VL53L3CXWrapper() = default;

    /**
     * @brief Initialize VL53L3CX
     * @param config I2C and GPIO configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Start ranging measurement
     * @return ESP_OK on success
     */
    esp_err_t startRanging();

    /**
     * @brief Stop ranging measurement
     * @return ESP_OK on success
     */
    esp_err_t stopRanging();

    /**
     * @brief Get distance measurement
     * @param distance_mm Distance in millimeters
     * @param status Range status (0-4 valid, >4 error)
     * @return ESP_OK on success
     */
    esp_err_t getDistance(uint16_t& distance_mm, uint8_t& status);

    /**
     * @brief Check if data is ready
     * @return true if new data available
     */
    bool isDataReady();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
};

}  // namespace stampfly
