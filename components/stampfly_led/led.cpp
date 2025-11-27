/**
 * @file led.cpp
 * @brief WS2812 RGB LED Driver Implementation (Stub)
 */

#include "led.hpp"
#include "esp_log.h"

static const char* TAG = "LED";

namespace stampfly {

esp_err_t LED::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing LED on GPIO%d (stub)", config.gpio);
    config_ = config;
    // TODO: Implement RMT initialization for WS2812
    initialized_ = true;
    return ESP_OK;
}

void LED::setColor(uint8_t index, uint32_t color)
{
    if (!initialized_) return;
    // TODO: Implement color setting
}

void LED::setPattern(Pattern pattern, uint32_t color)
{
    if (!initialized_) return;
    current_pattern_ = pattern;
    current_color_ = color;
}

void LED::update()
{
    if (!initialized_) return;
    // TODO: Implement pattern updates
}

void LED::showInit()      { setPattern(Pattern::BREATHE, 0x0000FF); }  // Blue breathe
void LED::showCalibrating() { setPattern(Pattern::BLINK_FAST, 0xFFFF00); }  // Yellow fast blink
void LED::showIdle()      { setPattern(Pattern::SOLID, 0x00FF00); }  // Green solid
void LED::showArmed()     { setPattern(Pattern::BLINK_SLOW, 0x00FF00); }  // Green slow blink
void LED::showFlying()    { setPattern(Pattern::SOLID, 0x00FF00); }  // Green solid
void LED::showLanding()   { setPattern(Pattern::BLINK_FAST, 0x00FF00); }  // Green fast blink
void LED::showError()     { setPattern(Pattern::BLINK_FAST, 0xFF0000); }  // Red fast blink
void LED::showLowBattery() { setPattern(Pattern::BLINK_SLOW, 0xFF0000); }  // Red slow blink
void LED::showPairing()   { setPattern(Pattern::BLINK_FAST, 0x0000FF); }  // Blue fast blink

}  // namespace stampfly
