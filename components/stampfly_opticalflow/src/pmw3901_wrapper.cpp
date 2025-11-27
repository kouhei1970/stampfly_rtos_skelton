/**
 * @file pmw3901_wrapper.cpp
 * @brief PMW3901 Optical Flow Implementation (Placeholder)
 */
#include "pmw3901_wrapper.hpp"
#include "esp_log.h"

static const char* TAG = "pmw3901";

namespace stampfly {

esp_err_t PMW3901::init(const Config& config) {
    ESP_LOGI(TAG, "PMW3901 init (placeholder)");
    // TODO: Integrate actual driver from kouhei1970/stampfly_opticalflow
    return ESP_OK;
}

esp_err_t PMW3901::read(FlowData& data) {
    data = {};
    return ESP_OK;
}

}  // namespace stampfly
