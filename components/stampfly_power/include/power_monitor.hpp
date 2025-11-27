/**
 * @file power_monitor.hpp
 * @brief INA3221 Power Monitor Driver
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class PowerMonitor {
public:
    struct Config {
        int i2c_port;
        uint8_t i2c_addr;
        float shunt_resistance_mohm;
    };

    struct PowerData {
        float voltage;   // [V]
        float current;   // [A]
        float power;     // [W]
    };

    static constexpr float BATTERY_LOW_VOLTAGE = 3.4f;
    static constexpr float BATTERY_FULL_VOLTAGE = 4.2f;

    static PowerMonitor& getInstance();

    esp_err_t init(const Config& config = Config{});
    esp_err_t read(PowerData& data);
    bool isLowBattery() const;
    float getBatteryPercent() const;

private:
    PowerMonitor() = default;
    float voltage_ = 0.0f;
};

}  // namespace stampfly
