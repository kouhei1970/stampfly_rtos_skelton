/**
 * @file bmp280.hpp
 * @brief BMP280 Barometer Driver
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class BMP280 {
public:
    struct Config {
        int i2c_port;
        uint8_t i2c_addr;
    };

    struct BaroData {
        float pressure;     // [hPa]
        float temperature;  // [°C]
        float altitude;     // [m]
    };

    esp_err_t init(const Config& config);
    esp_err_t read(BaroData& data);
    void setSeaLevelPressure(float pressure_hpa);
};

}  // namespace stampfly
