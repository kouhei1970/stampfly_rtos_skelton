/**
 * @file led.hpp
 * @brief RGB LED Control (WS2812)
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class LED {
public:
    static constexpr uint32_t WHITE   = 0xFFFFFF;
    static constexpr uint32_t RED     = 0xFF0000;
    static constexpr uint32_t GREEN   = 0x00FF00;
    static constexpr uint32_t BLUE    = 0x0000FF;
    static constexpr uint32_t YELLOW  = 0xFFFF00;
    static constexpr uint32_t PURPLE  = 0xFF00FF;
    static constexpr uint32_t CYAN    = 0x00FFFF;

    enum Pattern {
        PATTERN_OFF,
        PATTERN_SOLID,
        PATTERN_BLINK_SLOW,
        PATTERN_BLINK_FAST
    };

    struct Config {
        int pin_onboard;
        int pin_esp;
        uint8_t brightness;
    };

    static LED& getInstance();

    esp_err_t init(const Config& config = Config{});
    void setOnboardLED(uint8_t index, uint32_t color);
    void setEspLED(uint32_t color);
    void setPattern(Pattern pattern, uint32_t color);
    void update();

    void showInit();
    void showCalibrating();
    void showIdle();
    void showArmed();
    void showFlying();
    void showPairing();
    void showConnected();
    void showDisconnected();
    void showLowBattery();
    void showError();

private:
    LED() = default;
};

}  // namespace stampfly
