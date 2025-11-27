/**
 * @file bmp280.cpp
 * @brief BMP280 Barometer Implementation (Placeholder)
 */
#include "bmp280.hpp"
#include "esp_log.h"

static const char* TAG = "bmp280";

namespace stampfly {

esp_err_t BMP280::init(const Config& config) {
    ESP_LOGI(TAG, "BMP280 init (placeholder)");
    // TODO: Implement BMP280 driver
    return ESP_OK;
}

esp_err_t BMP280::read(BaroData& data) {
    data = {};
    return ESP_OK;
}

void BMP280::setSeaLevelPressure(float pressure_hpa) {
    // TODO: Implement
}

}  // namespace stampfly
