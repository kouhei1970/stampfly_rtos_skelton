/**
 * @file vl53l3cx_wrapper.cpp
 * @brief VL53L3CX ToF Sensor Driver Implementation
 */

#include "vl53l3cx_wrapper.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "VL53L3CX";

namespace stampfly {

VL53L3CX::VL53L3CX()
    : i2c_port_(I2C_NUM_0)
    , i2c_addr_(0x29)
    , xshut_pin_(GPIO_NUM_NC)
    , int_pin_(GPIO_NUM_NC)
    , mutex_(nullptr)
    , initialized_(false)
    , ranging_(false) {
}

VL53L3CX::~VL53L3CX() {
    if (ranging_) {
        stopRanging();
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t VL53L3CX::init(const Config& config) {
    esp_err_t ret;

    i2c_port_ = config.i2c_port;
    i2c_addr_ = config.i2c_addr;
    xshut_pin_ = config.xshut_pin;
    int_pin_ = config.int_pin;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure XSHUT pin if specified
    if (xshut_pin_ != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << xshut_pin_);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);

        // Ensure sensor is powered on
        gpio_set_level(xshut_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Configure INT pin if specified
    if (int_pin_ != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << int_pin_);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
    }

    // Wait for sensor boot
    ret = waitForBoot();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor boot timeout");
        return ret;
    }

    // Verify model ID
    uint16_t model_id = getModelId();
    if ((model_id >> 8) != MODEL_ID_VL53L3CX) {
        ESP_LOGE(TAG, "Invalid model ID: 0x%04X", model_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "VL53L3CX detected, model ID: 0x%04X", model_id);

    // Load default configuration
    ret = loadDefaultConfig();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load config");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "VL53L3CX initialized at address 0x%02X", i2c_addr_);
    return ESP_OK;
}

esp_err_t VL53L3CX::waitForBoot() {
    uint8_t status = 0;
    int timeout = 100;  // 1 second timeout

    while (timeout > 0) {
        if (readRegister(REG_FIRMWARE_SYSTEM_STATUS, &status) == ESP_OK) {
            if (status == 0x01) {
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t VL53L3CX::loadDefaultConfig() {
    esp_err_t ret;

    // Configure GPIO interrupt: active low
    ret = writeRegister(REG_GPIO_HV_MUX_CTRL, 0x10);
    if (ret != ESP_OK) return ret;

    // Clear interrupt
    ret = clearInterrupt();
    if (ret != ESP_OK) return ret;

    // Set timing budget (default ~33ms)
    ret = writeRegister16(REG_RANGE_CONFIG_TIMEOUT_MACROP_A, 0x01B1);
    if (ret != ESP_OK) return ret;

    ret = writeRegister16(REG_RANGE_CONFIG_TIMEOUT_MACROP_B, 0x01B9);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t VL53L3CX::startRanging(uint32_t period_ms) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clear any pending interrupt
    esp_err_t ret = clearInterrupt();
    if (ret != ESP_OK) return ret;

    // Start continuous measurement
    ret = writeRegister(REG_SYSTEM_MODE_START, 0x40);
    if (ret != ESP_OK) return ret;

    ranging_ = true;
    ESP_LOGI(TAG, "Ranging started");
    return ESP_OK;
}

esp_err_t VL53L3CX::stopRanging() {
    esp_err_t ret = writeRegister(REG_SYSTEM_MODE_START, 0x00);
    if (ret == ESP_OK) {
        ranging_ = false;
        ESP_LOGI(TAG, "Ranging stopped");
    }
    return ret;
}

esp_err_t VL53L3CX::read(RangeData& data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read range status and data
    uint8_t buffer[17];
    esp_err_t ret = readRegisters(REG_RESULT_RANGE_STATUS, buffer, 17);

    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse result
    data.range_status = buffer[0] & 0x1F;
    data.signal_rate = buffer[6];
    data.distance_mm = (uint16_t)(buffer[13] << 8) | buffer[14];
    data.timestamp_us = esp_timer_get_time();
    data.valid = (data.range_status == RANGE_STATUS_VALID);

    // Clear interrupt after reading
    clearInterrupt();

    return ESP_OK;
}

esp_err_t VL53L3CX::read(float& distance_mm) {
    RangeData data;
    esp_err_t ret = read(data);
    if (ret == ESP_OK && data.valid) {
        distance_mm = (float)data.distance_mm;
    } else {
        distance_mm = -1.0f;  // Invalid
    }
    return ret;
}

bool VL53L3CX::isDataReady() {
    if (int_pin_ != GPIO_NUM_NC) {
        // INT pin is active low
        return gpio_get_level(int_pin_) == 0;
    }

    // Fallback: read status register
    uint8_t status;
    if (readRegister(REG_GPIO_TIO_HV_STATUS, &status) == ESP_OK) {
        return (status & 0x01) == 0;
    }
    return false;
}

esp_err_t VL53L3CX::setAddress(uint8_t new_addr) {
    esp_err_t ret = writeRegister(REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (ret == ESP_OK) {
        i2c_addr_ = new_addr;
        ESP_LOGI(TAG, "I2C address changed to 0x%02X", new_addr);
    }
    return ret;
}

uint16_t VL53L3CX::getModelId() {
    uint16_t id = 0;
    readRegister16(REG_MODEL_ID, &id);
    return id;
}

esp_err_t VL53L3CX::clearInterrupt() {
    return writeRegister(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

esp_err_t VL53L3CX::writeRegister(uint16_t reg, uint8_t value) {
    uint8_t data[3] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        value
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

esp_err_t VL53L3CX::writeRegister16(uint16_t reg, uint16_t value) {
    uint8_t data[4] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 4, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t VL53L3CX::writeRegister32(uint16_t reg, uint32_t value) {
    uint8_t data[6] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 6, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t VL53L3CX::readRegister(uint16_t reg, uint8_t* value) {
    uint8_t reg_data[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg_data, 2, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t VL53L3CX::readRegister16(uint16_t reg, uint16_t* value) {
    uint8_t data[2];
    uint8_t reg_data[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg_data, 2, true);
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

esp_err_t VL53L3CX::readRegisters(uint16_t reg, uint8_t* buffer, size_t len) {
    uint8_t reg_data[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg_data, 2, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr_ << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

}  // namespace stampfly
