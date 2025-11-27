/**
 * @file vl53l3cx_wrapper.cpp
 * @brief VL53L3CX ToF Sensor Implementation (Placeholder)
 */
#include "vl53l3cx_wrapper.hpp"
#include "esp_log.h"

static const char* TAG = "vl53l3cx";

namespace stampfly {

esp_err_t VL53L3CX::init(const Config& config) {
    ESP_LOGI(TAG, "VL53L3CX init (placeholder) addr=0x%02X", config.i2c_addr);
    // TODO: Integrate actual driver from kouhei1970/stampfly_tof
    return ESP_OK;
}

esp_err_t VL53L3CX::read(float& distance_mm) {
    distance_mm = 0.0f;
    return ESP_OK;
}

}  // namespace stampfly
