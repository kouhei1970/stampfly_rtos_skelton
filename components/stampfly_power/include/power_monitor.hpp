/**
 * @file power_monitor.hpp
 * @brief INA3221 Power Monitor Driver
 *
 * 3-channel support, low voltage warning (3.4V threshold)
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

struct PowerData {
    float voltage_v;       // Battery voltage (V)
    float current_ma;      // Battery current (mA)
    float power_mw;        // Power consumption (mW)
    uint32_t timestamp_us;
};

class PowerMonitor {
public:
    static constexpr float LOW_BATTERY_THRESHOLD_V = 3.4f;
    static constexpr float FULL_BATTERY_V = 4.2f;
    static constexpr float EMPTY_BATTERY_V = 3.3f;

    struct Config {
        int i2c_port;
        int sda_gpio;
        int scl_gpio;
        uint8_t i2c_addr;
        uint8_t channel;    // INA3221 channel (0-2)
    };

    PowerMonitor() = default;
    ~PowerMonitor() = default;

    /**
     * @brief Initialize INA3221
     * @param config I2C configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read power data
     * @param data Power data output
     * @return ESP_OK on success
     */
    esp_err_t read(PowerData& data);

    /**
     * @brief Check if battery is low (<3.4V)
     * @return true if battery voltage is below threshold
     */
    bool isLowBattery() const { return last_voltage_v_ < LOW_BATTERY_THRESHOLD_V; }

    /**
     * @brief Get battery percentage
     * @return Battery percentage (0-100)
     */
    float getBatteryPercent() const;

    /**
     * @brief Get last read voltage
     * @return Voltage in volts
     */
    float getVoltage() const { return last_voltage_v_; }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    float last_voltage_v_ = 0.0f;
};

}  // namespace stampfly
