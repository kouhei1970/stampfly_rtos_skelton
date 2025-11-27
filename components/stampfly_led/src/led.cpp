/**
 * @file led.cpp
 * @brief RGB LED Implementation (Placeholder)
 */
#include "led.hpp"
#include "esp_log.h"

static const char* TAG = "led";

namespace stampfly {

LED& LED::getInstance() {
    static LED instance;
    return instance;
}

esp_err_t LED::init(const Config& config) {
    ESP_LOGI(TAG, "LED init (placeholder)");
    // TODO: Implement WS2812 LED strip driver
    return ESP_OK;
}

void LED::setOnboardLED(uint8_t index, uint32_t color) {}
void LED::setEspLED(uint32_t color) {}
void LED::setPattern(Pattern pattern, uint32_t color) {}
void LED::update() {}

void LED::showInit() {}
void LED::showCalibrating() {}
void LED::showIdle() {}
void LED::showArmed() {}
void LED::showFlying() {}
void LED::showPairing() {}
void LED::showConnected() {}
void LED::showDisconnected() {}
void LED::showLowBattery() {}
void LED::showError() {}

}  // namespace stampfly
