/**
 * @file bmi270_wrapper.cpp
 * @brief BMI270 IMU C++ Wrapper Implementation (Placeholder)
 */
#include "bmi270_wrapper.hpp"
#include "esp_log.h"

static const char* TAG = "bmi270";

namespace stampfly {

esp_err_t BMI270::init(const Config& config) {
    ESP_LOGI(TAG, "BMI270 init (placeholder)");
    // TODO: Integrate actual driver from kouhei1970/stampfly_imu
    return ESP_OK;
}

esp_err_t BMI270::read(ImuData& data) {
    // Placeholder: return zeros
    data = {};
    return ESP_OK;
}

}  // namespace stampfly
