/**
 * @file pmw3901_wrapper.hpp
 * @brief PMW3901 Optical Flow Sensor Driver for StampFly
 *
 * SPI interface driver for PixArt PMW3901 optical flow sensor
 */
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class PMW3901 {
public:
    struct Config {
        spi_host_device_t spi_host;
        gpio_num_t cs_pin;
        int clock_speed_hz;
    };

    struct FlowData {
        int16_t delta_x;        // X motion delta (pixels)
        int16_t delta_y;        // Y motion delta (pixels)
        uint8_t squal;          // Surface quality (0-255)
        uint8_t shutter_upper;  // Shutter value (upper)
        uint8_t shutter_lower;  // Shutter value (lower)
        uint8_t raw_sum;        // Raw data sum
        uint8_t max_raw;        // Max raw value
        uint8_t min_raw;        // Min raw value
        int64_t timestamp_us;   // Timestamp
        bool motion;            // True if motion detected
    };

    PMW3901();
    ~PMW3901();

    /**
     * @brief Initialize the sensor
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read flow data
     * @param data Output flow data structure
     * @return ESP_OK on success
     */
    esp_err_t read(FlowData& data);

    /**
     * @brief Check if motion has been detected
     * @return true if motion detected
     */
    bool hasMotion();

    /**
     * @brief Get product ID
     * @return Product ID (should be 0x49 for PMW3901)
     */
    uint8_t getProductId();

    /**
     * @brief Enable/disable frame capture for debugging
     * @param enable True to enable
     */
    void enableFrameCapture(bool enable);

    /**
     * @brief Read a single frame (35x35 pixels) for debugging
     * @param buffer Output buffer (1225 bytes)
     * @return ESP_OK on success
     */
    esp_err_t readFrame(uint8_t* buffer);

private:
    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    esp_err_t readRegister(uint8_t reg, uint8_t* value);
    esp_err_t readMotionBurst(uint8_t* buffer, size_t len);

    esp_err_t performSramDownload();
    esp_err_t initRegisters();

    spi_device_handle_t spi_handle_;
    gpio_num_t cs_pin_;
    SemaphoreHandle_t mutex_;
    bool initialized_;

    // PMW3901 Registers
    static constexpr uint8_t REG_PRODUCT_ID = 0x00;
    static constexpr uint8_t REG_REVISION_ID = 0x01;
    static constexpr uint8_t REG_MOTION = 0x02;
    static constexpr uint8_t REG_DELTA_X_L = 0x03;
    static constexpr uint8_t REG_DELTA_X_H = 0x04;
    static constexpr uint8_t REG_DELTA_Y_L = 0x05;
    static constexpr uint8_t REG_DELTA_Y_H = 0x06;
    static constexpr uint8_t REG_SQUAL = 0x07;
    static constexpr uint8_t REG_RAW_SUM = 0x08;
    static constexpr uint8_t REG_MAX_RAW = 0x09;
    static constexpr uint8_t REG_MIN_RAW = 0x0A;
    static constexpr uint8_t REG_SHUTTER_LOWER = 0x0B;
    static constexpr uint8_t REG_SHUTTER_UPPER = 0x0C;
    static constexpr uint8_t REG_MOTION_BURST = 0x16;
    static constexpr uint8_t REG_POWER_UP_RESET = 0x3A;
    static constexpr uint8_t REG_SHUTDOWN = 0x3B;
    static constexpr uint8_t REG_RAW_GRAB = 0x58;
    static constexpr uint8_t REG_RAW_GRAB_STATUS = 0x59;

    // Expected values
    static constexpr uint8_t PRODUCT_ID_PMW3901 = 0x49;
};

}  // namespace stampfly
