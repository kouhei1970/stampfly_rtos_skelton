/**
 * @file button.hpp
 * @brief Button Control
 */
#pragma once
#include <cstdint>
#include <functional>
#include "esp_err.h"

namespace stampfly {

class Button {
public:
    enum Event {
        EVENT_NONE,
        EVENT_CLICK,
        EVENT_DOUBLE_CLICK,
        EVENT_LONG_PRESS,
        EVENT_LONG_PRESS_START
    };

    using EventCallback = std::function<void(Event)>;

    struct Config {
        int pin;
        bool active_low;
        uint32_t debounce_ms;
        uint32_t long_press_ms;
    };

    static Button& getInstance();

    esp_err_t init(const Config& config = Config{});
    void setCallback(EventCallback callback);
    void tick();
    bool isPressed() const;
    Event getLastEvent();

private:
    Button() = default;
    Event last_event_ = EVENT_NONE;
};

}  // namespace stampfly
