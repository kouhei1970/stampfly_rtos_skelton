/**
 * @file bmp280.cpp
 * @brief BMP280 Barometric Pressure Sensor Driver Implementation (Stub)
 */

#include "bmp280.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "BMP280";

namespace stampfly {

esp_err_t BMP280::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing BMP280 (stub)");
    config_ = config;
    sea_level_pressure_pa_ = config.sea_level_pressure_pa > 0 ?
                             config.sea_level_pressure_pa : 101325.0f;
    // TODO: Implement I2C initialization and BMP280 configuration
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BMP280::read(BaroData& data)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    // TODO: Implement barometer data reading
    data.pressure_pa = 101325.0f;
    data.temperature_c = 25.0f;
    data.altitude_m = calculateAltitude(data.pressure_pa);
    data.timestamp_us = 0;
    return ESP_OK;
}

float BMP280::calculateAltitude(float pressure_pa) const
{
    // Barometric formula
    return 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pressure_pa_, 0.1903f));
}

void BMP280::setSeaLevelPressure(float pressure_pa)
{
    sea_level_pressure_pa_ = pressure_pa;
}

}  // namespace stampfly
