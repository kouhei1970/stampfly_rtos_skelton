/**
 * @file bmm150.hpp
 * @brief BMM150 Magnetometer Driver
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class BMM150 {
public:
    struct Config {
        int i2c_port;
        uint8_t i2c_addr;
    };

    struct MagData {
        float x, y, z;  // [uT]
    };

    esp_err_t init(const Config& config);
    esp_err_t read(MagData& data);
};

}  // namespace stampfly
