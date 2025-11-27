/**
 * @file bmm150.hpp
 * @brief BMM150 Magnetometer Driver
 *
 * I2C communication, XYZ magnetic field data
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

struct MagData {
    float x;  // uT (micro Tesla)
    float y;
    float z;
    uint32_t timestamp_us;
};

class BMM150 {
public:
    struct Config {
        int i2c_port;
        int sda_gpio;
        int scl_gpio;
        uint8_t i2c_addr;
    };

    BMM150() = default;
    ~BMM150() = default;

    /**
     * @brief Initialize BMM150
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read magnetometer data
     * @param data Magnetometer data output
     * @return ESP_OK on success
     */
    esp_err_t read(MagData& data);

    /**
     * @brief Perform soft reset
     * @return ESP_OK on success
     */
    esp_err_t softReset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
};

}  // namespace stampfly
