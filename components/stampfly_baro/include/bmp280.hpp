/**
 * @file bmp280.hpp
 * @brief BMP280 Barometric Pressure Sensor Driver
 *
 * I2C communication, temperature compensation, altitude calculation
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

struct BaroData {
    float pressure_pa;     // Pascal
    float temperature_c;   // Celsius
    float altitude_m;      // Meters (calculated)
    uint32_t timestamp_us;
};

class BMP280 {
public:
    struct Config {
        int i2c_port;
        int sda_gpio;
        int scl_gpio;
        uint8_t i2c_addr;
        float sea_level_pressure_pa;  // Reference pressure (default: 101325 Pa)
    };

    BMP280() = default;
    ~BMP280() = default;

    /**
     * @brief Initialize BMP280
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read barometer data
     * @param data Barometer data output
     * @return ESP_OK on success
     */
    esp_err_t read(BaroData& data);

    /**
     * @brief Calculate altitude from pressure
     * @param pressure_pa Pressure in Pascal
     * @return Altitude in meters
     */
    float calculateAltitude(float pressure_pa) const;

    /**
     * @brief Set sea level reference pressure
     * @param pressure_pa Sea level pressure in Pascal
     */
    void setSeaLevelPressure(float pressure_pa);

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    float sea_level_pressure_pa_ = 101325.0f;
};

}  // namespace stampfly
