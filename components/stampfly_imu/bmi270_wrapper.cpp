/**
 * @file bmi270_wrapper.cpp
 * @brief BMI270 IMU C++ Wrapper Implementation (Stub)
 */

#include "bmi270_wrapper.hpp"
#include "esp_log.h"

static const char* TAG = "BMI270";

namespace stampfly {

esp_err_t BMI270Wrapper::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing BMI270 (stub)");
    config_ = config;
    // TODO: Implement SPI initialization and BMI270 configuration
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BMI270Wrapper::readSensorData(AccelData& accel, GyroData& gyro)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    // TODO: Implement sensor data reading
    accel = {0.0f, 0.0f, 9.81f};
    gyro = {0.0f, 0.0f, 0.0f};
    return ESP_OK;
}

esp_err_t BMI270Wrapper::readFIFO(FIFOData* buffer, size_t max_frames, size_t& frames_read)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    // TODO: Implement FIFO reading
    frames_read = 0;
    return ESP_OK;
}

}  // namespace stampfly
