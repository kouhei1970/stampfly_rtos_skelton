/**
 * @file bmi270_wrapper.hpp
 * @brief BMI270 IMU C++ Wrapper
 *
 * SPI communication, FIFO support, 1600Hz internal sampling
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

struct AccelData {
    float x;  // m/s^2
    float y;
    float z;
};

struct GyroData {
    float x;  // rad/s
    float y;
    float z;
};

struct FIFOData {
    AccelData accel;
    GyroData gyro;
    uint32_t timestamp_us;
};

class BMI270Wrapper {
public:
    struct Config {
        int spi_host;
        int cs_gpio;
        int mosi_gpio;
        int miso_gpio;
        int sck_gpio;
        int spi_freq_hz;
    };

    BMI270Wrapper() = default;
    ~BMI270Wrapper() = default;

    /**
     * @brief Initialize BMI270
     * @param config SPI configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Read sensor data (single sample)
     * @param accel Accelerometer data output
     * @param gyro Gyroscope data output
     * @return ESP_OK on success
     */
    esp_err_t readSensorData(AccelData& accel, GyroData& gyro);

    /**
     * @brief Read FIFO data
     * @param buffer Output buffer for FIFO frames
     * @param max_frames Maximum frames to read
     * @param frames_read Actual frames read
     * @return ESP_OK on success
     */
    esp_err_t readFIFO(FIFOData* buffer, size_t max_frames, size_t& frames_read);

    /**
     * @brief Check if device is initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
};

}  // namespace stampfly
