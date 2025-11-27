/**
 * @file vl53l3cx_wrapper.hpp
 * @brief VL53L3CX ToF Sensor C++ Wrapper (Placeholder)
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class VL53L3CX {
public:
    struct Config {
        int i2c_port;
        uint8_t i2c_addr;
        int xshut_pin;
        int int_pin;
    };

    esp_err_t init(const Config& config);
    esp_err_t read(float& distance_mm);
};

}  // namespace stampfly
