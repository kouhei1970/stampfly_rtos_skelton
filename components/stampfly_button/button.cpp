/**
 * @file button.cpp
 * @brief Button Driver Implementation (Stub)
 */

#include "button.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Button";

namespace stampfly {

esp_err_t Button::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing Button on GPIO%d (stub)", config.gpio);
    config_ = config;
    // TODO: Implement GPIO initialization with pull-up
    initialized_ = true;
    return ESP_OK;
}

void Button::setCallback(EventCallback callback)
{
    callback_ = callback;
}

void Button::tick()
{
    if (!initialized_) return;

    // TODO: Read GPIO state
    bool pressed = false;  // gpio_get_level(config_.gpio) == 0
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (pressed && !was_pressed_) {
        // Button just pressed
        press_start_time_ = now;
        was_pressed_ = true;
        long_press_3s_triggered_ = false;
        long_press_5s_triggered_ = false;
    }
    else if (!pressed && was_pressed_) {
        // Button released
        uint32_t duration = now - press_start_time_;
        was_pressed_ = false;

        if (duration < 1000 && !long_press_3s_triggered_) {
            last_event_ = Event::CLICK;
            if (callback_) callback_(Event::CLICK);
        }
    }
    else if (pressed && was_pressed_) {
        // Button held
        uint32_t duration = now - press_start_time_;

        if (duration >= 5000 && !long_press_5s_triggered_) {
            long_press_5s_triggered_ = true;
            last_event_ = Event::LONG_PRESS_5S;
            if (callback_) callback_(Event::LONG_PRESS_5S);
        }
        else if (duration >= 3000 && !long_press_3s_triggered_) {
            long_press_3s_triggered_ = true;
            last_event_ = Event::LONG_PRESS_3S;
            if (callback_) callback_(Event::LONG_PRESS_3S);
        }
    }

    is_pressed_ = pressed;
}

}  // namespace stampfly
