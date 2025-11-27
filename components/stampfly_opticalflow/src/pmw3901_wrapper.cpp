/**
 * @file pmw3901_wrapper.cpp
 * @brief PMW3901 Optical Flow Sensor Driver Implementation
 */

#include "pmw3901_wrapper.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "PMW3901";

namespace stampfly {

PMW3901::PMW3901()
    : spi_handle_(nullptr)
    , cs_pin_(GPIO_NUM_NC)
    , mutex_(nullptr)
    , initialized_(false) {
}

PMW3901::~PMW3901() {
    if (spi_handle_) {
        spi_bus_remove_device(spi_handle_);
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t PMW3901::init(const Config& config) {
    esp_err_t ret;
    cs_pin_ = config.cs_pin;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_config = {};
    dev_config.command_bits = 0;
    dev_config.address_bits = 0;
    dev_config.dummy_bits = 0;
    dev_config.mode = 3;  // CPOL=1, CPHA=1
    dev_config.duty_cycle_pos = 128;
    dev_config.cs_ena_pretrans = 0;
    dev_config.cs_ena_posttrans = 0;
    dev_config.clock_speed_hz = config.clock_speed_hz;
    dev_config.input_delay_ns = 0;
    dev_config.spics_io_num = config.cs_pin;
    dev_config.flags = 0;
    dev_config.queue_size = 1;
    dev_config.pre_cb = nullptr;
    dev_config.post_cb = nullptr;

    ret = spi_bus_add_device(config.spi_host, &dev_config, &spi_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Power-up reset sequence
    ret = writeRegister(REG_POWER_UP_RESET, 0x5A);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power-up reset failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read and discard motion registers to clear them
    uint8_t dummy;
    readRegister(REG_MOTION, &dummy);
    readRegister(REG_DELTA_X_L, &dummy);
    readRegister(REG_DELTA_X_H, &dummy);
    readRegister(REG_DELTA_Y_L, &dummy);
    readRegister(REG_DELTA_Y_H, &dummy);

    // Verify product ID
    uint8_t product_id = getProductId();
    if (product_id != PRODUCT_ID_PMW3901) {
        ESP_LOGE(TAG, "Invalid product ID: 0x%02X (expected 0x%02X)",
                 product_id, PRODUCT_ID_PMW3901);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PMW3901 detected, product ID: 0x%02X", product_id);

    // Initialize sensor registers
    ret = initRegisters();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register initialization failed");
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "PMW3901 initialized successfully");
    return ESP_OK;
}

esp_err_t PMW3901::initRegisters() {
    // Performance optimization sequence from datasheet
    writeRegister(0x7F, 0x00);
    writeRegister(0x61, 0xAD);
    writeRegister(0x7F, 0x03);
    writeRegister(0x40, 0x00);
    writeRegister(0x7F, 0x05);
    writeRegister(0x41, 0xB3);
    writeRegister(0x43, 0xF1);
    writeRegister(0x45, 0x14);
    writeRegister(0x5B, 0x32);
    writeRegister(0x5F, 0x34);
    writeRegister(0x7B, 0x08);
    writeRegister(0x7F, 0x06);
    writeRegister(0x44, 0x1B);
    writeRegister(0x40, 0xBF);
    writeRegister(0x4E, 0x3F);
    writeRegister(0x7F, 0x08);
    writeRegister(0x65, 0x20);
    writeRegister(0x6A, 0x18);
    writeRegister(0x7F, 0x09);
    writeRegister(0x4F, 0xAF);
    writeRegister(0x5F, 0x40);
    writeRegister(0x48, 0x80);
    writeRegister(0x49, 0x80);
    writeRegister(0x57, 0x77);
    writeRegister(0x60, 0x78);
    writeRegister(0x61, 0x78);
    writeRegister(0x62, 0x08);
    writeRegister(0x63, 0x50);
    writeRegister(0x7F, 0x0A);
    writeRegister(0x45, 0x60);
    writeRegister(0x7F, 0x00);
    writeRegister(0x4D, 0x11);
    writeRegister(0x55, 0x80);
    writeRegister(0x74, 0x1F);
    writeRegister(0x75, 0x1F);
    writeRegister(0x4A, 0x78);
    writeRegister(0x4B, 0x78);
    writeRegister(0x44, 0x08);
    writeRegister(0x45, 0x50);
    writeRegister(0x64, 0xFF);
    writeRegister(0x65, 0x1F);
    writeRegister(0x7F, 0x14);
    writeRegister(0x65, 0x60);
    writeRegister(0x66, 0x08);
    writeRegister(0x63, 0x78);
    writeRegister(0x7F, 0x15);
    writeRegister(0x48, 0x58);
    writeRegister(0x7F, 0x07);
    writeRegister(0x41, 0x0D);
    writeRegister(0x43, 0x14);
    writeRegister(0x4B, 0x0E);
    writeRegister(0x45, 0x0F);
    writeRegister(0x44, 0x42);
    writeRegister(0x4C, 0x80);
    writeRegister(0x7F, 0x10);
    writeRegister(0x5B, 0x02);
    writeRegister(0x7F, 0x07);
    writeRegister(0x40, 0x41);
    writeRegister(0x70, 0x00);

    vTaskDelay(pdMS_TO_TICKS(10));

    writeRegister(0x32, 0x44);
    writeRegister(0x7F, 0x07);
    writeRegister(0x40, 0x40);
    writeRegister(0x7F, 0x06);
    writeRegister(0x62, 0xF0);
    writeRegister(0x63, 0x00);
    writeRegister(0x7F, 0x0D);
    writeRegister(0x48, 0xC0);
    writeRegister(0x6F, 0xD5);
    writeRegister(0x7F, 0x00);
    writeRegister(0x5B, 0xA0);
    writeRegister(0x4E, 0xA8);
    writeRegister(0x5A, 0x50);
    writeRegister(0x40, 0x80);

    return ESP_OK;
}

esp_err_t PMW3901::read(FlowData& data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read motion burst data (12 bytes)
    uint8_t buffer[12];
    esp_err_t ret = readMotionBurst(buffer, 12);

    xSemaphoreGive(mutex_);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse burst data
    // Byte 0: Motion register
    // Byte 1: Observation
    // Byte 2-3: Delta_X (Little Endian)
    // Byte 4-5: Delta_Y (Little Endian)
    // Byte 6: SQUAL
    // Byte 7: Raw_Data_Sum
    // Byte 8: Maximum_Raw_Data
    // Byte 9: Minimum_Raw_Data
    // Byte 10-11: Shutter

    data.motion = (buffer[0] & 0x80) != 0;
    data.delta_x = (int16_t)((buffer[3] << 8) | buffer[2]);
    data.delta_y = (int16_t)((buffer[5] << 8) | buffer[4]);
    data.squal = buffer[6];
    data.raw_sum = buffer[7];
    data.max_raw = buffer[8];
    data.min_raw = buffer[9];
    data.shutter_upper = buffer[11];
    data.shutter_lower = buffer[10];
    data.timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

bool PMW3901::hasMotion() {
    uint8_t motion;
    if (readRegister(REG_MOTION, &motion) == ESP_OK) {
        return (motion & 0x80) != 0;
    }
    return false;
}

uint8_t PMW3901::getProductId() {
    uint8_t id = 0;
    readRegister(REG_PRODUCT_ID, &id);
    return id;
}

void PMW3901::enableFrameCapture(bool enable) {
    if (enable) {
        writeRegister(REG_RAW_GRAB, 0x00);
    }
}

esp_err_t PMW3901::readFrame(uint8_t* buffer) {
    // Frame capture is 35x35 = 1225 bytes
    // This is a simplified implementation
    esp_err_t ret;

    writeRegister(0x7F, 0x07);
    writeRegister(0x4C, 0x00);
    writeRegister(0x7F, 0x08);
    writeRegister(0x6A, 0x38);
    writeRegister(0x7F, 0x00);
    writeRegister(0x55, 0x04);
    writeRegister(0x40, 0x80);
    writeRegister(REG_RAW_GRAB, 0xFF);

    vTaskDelay(pdMS_TO_TICKS(10));

    // Read 1225 pixels
    for (int i = 0; i < 1225; i++) {
        ret = readRegister(REG_RAW_GRAB, &buffer[i]);
        if (ret != ESP_OK) return ret;
    }

    // Restore normal operation
    writeRegister(0x7F, 0x07);
    writeRegister(0x4C, 0x80);
    writeRegister(0x7F, 0x08);
    writeRegister(0x6A, 0x18);
    writeRegister(0x7F, 0x00);
    writeRegister(0x55, 0x80);

    return ESP_OK;
}

esp_err_t PMW3901::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg | 0x80), value};

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = nullptr;

    esp_err_t ret = spi_device_polling_transmit(spi_handle_, &trans);

    // Write requires delay
    esp_rom_delay_us(120);

    return ret;
}

esp_err_t PMW3901::readRegister(uint8_t reg, uint8_t* value) {
    // Send register address (read bit is 0)
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg & 0x7F), 0x00};
    uint8_t rx_buf[2] = {0};

    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    // Address + delay
    esp_err_t ret = spi_device_polling_transmit(spi_handle_, &trans);
    if (ret != ESP_OK) return ret;

    // Read requires delay between address and data
    esp_rom_delay_us(50);

    // Data is in second byte
    *value = rx_buf[1];

    return ESP_OK;
}

esp_err_t PMW3901::readMotionBurst(uint8_t* buffer, size_t len) {
    // Send burst read command
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = REG_MOTION_BURST;

    spi_transaction_t trans = {};
    trans.length = (len + 1) * 8;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_polling_transmit(spi_handle_, &trans);
    if (ret == ESP_OK) {
        memcpy(buffer, &rx_buf[1], len);
    }

    // Minimum delay after burst read
    esp_rom_delay_us(50);

    return ret;
}

}  // namespace stampfly
