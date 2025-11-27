/**
 * @file bmi270_wrapper.hpp
 * @brief BMI270 IMU C++ Wrapper (Placeholder)
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class BMI270 {
public:
    struct Config {
        int spi_host;
        int cs_pin;
        int int1_pin;
    };

    struct ImuData {
        float accel_x, accel_y, accel_z;  // [m/s^2]
        float gyro_x, gyro_y, gyro_z;     // [rad/s]
    };

    esp_err_t init(const Config& config);
    esp_err_t read(ImuData& data);
};

}  // namespace stampfly
