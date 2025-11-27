/**
 * @file buzzer.cpp
 * @brief Buzzer Implementation (Placeholder)
 */
#include "buzzer.hpp"
#include "esp_log.h"

static const char* TAG = "buzzer";

namespace stampfly {

Buzzer& Buzzer::getInstance() {
    static Buzzer instance;
    return instance;
}

esp_err_t Buzzer::init(const Config& config) {
    ESP_LOGI(TAG, "Buzzer init (placeholder)");
    // TODO: Implement LEDC PWM buzzer
    return ESP_OK;
}

void Buzzer::playTone(uint16_t frequency, uint32_t duration_ms) {}
void Buzzer::stop() {}
void Buzzer::beep() {}
void Buzzer::startTone() { ESP_LOGI(TAG, "Start tone"); }
void Buzzer::armTone() {}
void Buzzer::disarmTone() {}
void Buzzer::lowBatteryWarning() {}
void Buzzer::errorTone() {}
void Buzzer::pairingTone() {}

}  // namespace stampfly
