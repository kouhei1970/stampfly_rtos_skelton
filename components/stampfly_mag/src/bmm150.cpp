/**
 * @file bmm150.cpp
 * @brief BMM150 Magnetometer Implementation (Placeholder)
 */
#include "bmm150.hpp"
#include "esp_log.h"

static const char* TAG = "bmm150";

namespace stampfly {

esp_err_t BMM150::init(const Config& config) {
    ESP_LOGI(TAG, "BMM150 init (placeholder)");
    // TODO: Implement BMM150 driver
    return ESP_OK;
}

esp_err_t BMM150::read(MagData& data) {
    data = {};
    return ESP_OK;
}

}  // namespace stampfly
