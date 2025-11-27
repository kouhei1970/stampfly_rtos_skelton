/**
 * @file power_monitor.hpp
 * @brief INA3221 Power Monitor Driver for StampFly
 *
 * I2C interface driver for TI INA3221 triple-channel power monitor
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class PowerMonitor {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr;
        float shunt_resistance_mohm;
        uint8_t battery_channel;  // 1, 2, or 3
    };

    struct PowerData {
        float voltage;        // [V] Bus voltage
        float current;        // [A] Current
        float power;          // [W] Power
        int64_t timestamp_us;
    };

    struct ChannelData {
        float shunt_voltage_mv;
        float bus_voltage_v;
        float current_a;
        float power_w;
    };

    static constexpr float BATTERY_LOW_VOLTAGE = 3.4f;
    static constexpr float BATTERY_FULL_VOLTAGE = 4.2f;
    static constexpr float BATTERY_MAX_VOLTAGE = 4.35f;  // LiHV

    static PowerMonitor& getInstance();

    /**
     * @brief Initialize the power monitor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read battery channel data
     * @param data Output power data structure
     * @return ESP_OK on success
     */
    esp_err_t read(PowerData& data);

    /**
     * @brief Read specific channel data
     * @param channel Channel number (1-3)
     * @param data Output channel data
     * @return ESP_OK on success
     */
    esp_err_t readChannel(uint8_t channel, ChannelData& data);

    /**
     * @brief Check if battery voltage is low
     * @return true if voltage below threshold
     */
    bool isLowBattery() const;

    /**
     * @brief Get estimated battery percentage
     * @return Battery percentage (0-100)
     */
    float getBatteryPercent() const;

    /**
     * @brief Get last read voltage
     * @return Voltage in volts
     */
    float getVoltage() const { return voltage_; }

    /**
     * @brief Get manufacturer ID
     * @return Manufacturer ID (should be 0x5449 for TI)
     */
    uint16_t getManufacturerId();

    /**
     * @brief Get die ID
     * @return Die ID (should be 0x3220 for INA3221)
     */
    uint16_t getDieId();

    /**
     * @brief Reset device
     * @return ESP_OK on success
     */
    esp_err_t reset();

private:
    PowerMonitor() = default;
    ~PowerMonitor();
    PowerMonitor(const PowerMonitor&) = delete;
    PowerMonitor& operator=(const PowerMonitor&) = delete;

    esp_err_t writeRegister(uint8_t reg, uint16_t value);
    esp_err_t readRegister(uint8_t reg, uint16_t* value);

    i2c_port_t i2c_port_;
    uint8_t i2c_addr_;
    float shunt_resistance_mohm_;
    uint8_t battery_channel_;
    SemaphoreHandle_t mutex_;
    bool initialized_;
    float voltage_;
    float current_;

    // INA3221 Registers
    static constexpr uint8_t REG_CONFIG = 0x00;
    static constexpr uint8_t REG_CH1_SHUNT_VOLTAGE = 0x01;
    static constexpr uint8_t REG_CH1_BUS_VOLTAGE = 0x02;
    static constexpr uint8_t REG_CH2_SHUNT_VOLTAGE = 0x03;
    static constexpr uint8_t REG_CH2_BUS_VOLTAGE = 0x04;
    static constexpr uint8_t REG_CH3_SHUNT_VOLTAGE = 0x05;
    static constexpr uint8_t REG_CH3_BUS_VOLTAGE = 0x06;
    static constexpr uint8_t REG_MANUFACTURER_ID = 0xFE;
    static constexpr uint8_t REG_DIE_ID = 0xFF;

    // INA3221 Constants
    static constexpr float SHUNT_VOLTAGE_LSB_UV = 40.0f;  // 40uV per LSB
    static constexpr float BUS_VOLTAGE_LSB_MV = 8.0f;     // 8mV per LSB
    static constexpr uint16_t MANUFACTURER_ID = 0x5449;   // "TI"
    static constexpr uint16_t DIE_ID = 0x3220;
};

}  // namespace stampfly
