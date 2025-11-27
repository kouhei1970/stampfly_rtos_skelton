/**
 * @file power_monitor.cpp
 * @brief INA3221 Power Monitor Implementation (Placeholder)
 */
#include "power_monitor.hpp"
#include "esp_log.h"

static const char* TAG = "power";

namespace stampfly {

PowerMonitor& PowerMonitor::getInstance() {
    static PowerMonitor instance;
    return instance;
}

esp_err_t PowerMonitor::init(const Config& config) {
    ESP_LOGI(TAG, "PowerMonitor init (placeholder)");
    // TODO: Implement INA3221 driver
    return ESP_OK;
}

esp_err_t PowerMonitor::read(PowerData& data) {
    data.voltage = 4.0f;  // Placeholder
    data.current = 0.5f;
    data.power = data.voltage * data.current;
    voltage_ = data.voltage;
    return ESP_OK;
}

bool PowerMonitor::isLowBattery() const {
    return voltage_ < BATTERY_LOW_VOLTAGE;
}

float PowerMonitor::getBatteryPercent() const {
    float percent = (voltage_ - 3.3f) / (4.2f - 3.3f) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

}  // namespace stampfly
