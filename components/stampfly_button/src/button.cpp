/**
 * @file button.cpp
 * @brief Button Implementation (Placeholder)
 */
#include "button.hpp"
#include "esp_log.h"

static const char* TAG = "button";

namespace stampfly {

Button& Button::getInstance() {
    static Button instance;
    return instance;
}

esp_err_t Button::init(const Config& config) {
    ESP_LOGI(TAG, "Button init (placeholder)");
    // TODO: Implement button with debounce
    return ESP_OK;
}

void Button::setCallback(EventCallback callback) {}
void Button::tick() {}
bool Button::isPressed() const { return false; }
Button::Event Button::getLastEvent() {
    Event e = last_event_;
    last_event_ = EVENT_NONE;
    return e;
}

}  // namespace stampfly
