/**
 * @file power_monitor.cpp
 * @brief INA3221 Power Monitor Driver Implementation
 */

#include "power_monitor.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "INA3221";

namespace stampfly {

PowerMonitor& PowerMonitor::getInstance() {
    static PowerMonitor instance;
    return instance;
}

PowerMonitor::~PowerMonitor() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t PowerMonitor::init(const Config& config) {
    esp_err_t ret;

    i2c_port_ = config.i2c_port;
    i2c_addr_ = config.i2c_addr;
    shunt_resistance_mohm_ = config.shunt_resistance_mohm;
    battery_channel_ = config.battery_channel;
    voltage_ = 0.0f;
    current_ = 0.0f;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Verify manufacturer ID
    uint16_t mfr_id = getManufacturerId();
    if (mfr_id != MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Invalid manufacturer ID: 0x%04X (expected 0x%04X)", mfr_id, MANUFACTURER_ID);
        return ESP_ERR_NOT_FOUND;
    }

    // Verify die ID
    uint16_t die_id = getDieId();
    if (die_id != DIE_ID) {
        ESP_LOGE(TAG, "Invalid die ID: 0x%04X (expected 0x%04X)", die_id, DIE_ID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "INA3221 detected, MFR ID: 0x%04X, DIE ID: 0x%04X", mfr_id, die_id);

    // Configure: enable all channels, continuous mode, 1.1ms conversion time
    // Config register: [15] RST=0, [14:12] CH1_EN=1, CH2_EN=1, CH3_EN=1
    // [11:9] AVG=000 (1 sample), [8:6] VBUS_CT=100 (1.1ms), [5:3] VSH_CT=100 (1.1ms)
    // [2:0] MODE=111 (continuous shunt and bus)
    uint16_t config_reg = 0x7127;  // All channels enabled, 1.1ms conversion, continuous mode
    ret = writeRegister(REG_CONFIG, config_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config register");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "INA3221 initialized, battery channel: %d, shunt: %.1f mOhm",
             battery_channel_, shunt_resistance_mohm_);
    return ESP_OK;
}

esp_err_t PowerMonitor::read(PowerData& data) {
    ChannelData ch_data;
    esp_err_t ret = readChannel(battery_channel_, ch_data);
    if (ret != ESP_OK) {
        return ret;
    }

    data.voltage = ch_data.bus_voltage_v;
    data.current = ch_data.current_a;
    data.power = ch_data.power_w;
    data.timestamp_us = esp_timer_get_time();

    // Update cached values
    voltage_ = data.voltage;
    current_ = data.current;

    return ESP_OK;
}

esp_err_t PowerMonitor::readChannel(uint8_t channel, ChannelData& data) {
    if (!initialized_ || channel < 1 || channel > 3) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Calculate register addresses for channel
    uint8_t shunt_reg = REG_CH1_SHUNT_VOLTAGE + (channel - 1) * 2;
    uint8_t bus_reg = REG_CH1_BUS_VOLTAGE + (channel - 1) * 2;

    uint16_t shunt_raw, bus_raw;
    esp_err_t ret;

    ret = readRegister(shunt_reg, &shunt_raw);
    if (ret != ESP_OK) {
        xSemaphoreGive(mutex_);
        return ret;
    }

    ret = readRegister(bus_reg, &bus_raw);
    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Convert raw values (both are signed 13-bit values with 3 LSBs ignored)
    int16_t shunt_signed = (int16_t)shunt_raw >> 3;
    int16_t bus_signed = (int16_t)bus_raw >> 3;

    // Calculate physical values
    data.shunt_voltage_mv = (float)shunt_signed * SHUNT_VOLTAGE_LSB_UV / 1000.0f;
    data.bus_voltage_v = (float)bus_signed * BUS_VOLTAGE_LSB_MV / 1000.0f;

    // Calculate current from shunt voltage and resistance
    // I = V_shunt / R_shunt
    data.current_a = data.shunt_voltage_mv / shunt_resistance_mohm_;

    // Calculate power
    data.power_w = data.bus_voltage_v * data.current_a;

    return ESP_OK;
}

bool PowerMonitor::isLowBattery() const {
    return voltage_ < BATTERY_LOW_VOLTAGE;
}

float PowerMonitor::getBatteryPercent() const {
    // Linear approximation between empty and full voltage
    float percent = (voltage_ - BATTERY_LOW_VOLTAGE) /
                    (BATTERY_FULL_VOLTAGE - BATTERY_LOW_VOLTAGE) * 100.0f;

    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

uint16_t PowerMonitor::getManufacturerId() {
    uint16_t id = 0;
    readRegister(REG_MANUFACTURER_ID, &id);
    return id;
}

uint16_t PowerMonitor::getDieId() {
    uint16_t id = 0;
    readRegister(REG_DIE_ID, &id);
    return id;
}

esp_err_t PowerMonitor::reset() {
    // Set RST bit in config register
    return writeRegister(REG_CONFIG, 0x8000);
}

esp_err_t PowerMonitor::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t PowerMonitor::readRegister(uint8_t reg, uint16_t* value) {
    uint8_t data[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return ret;
}

}  // namespace stampfly
