/**
 * @file led.hpp
 * @brief RGB LED Control (WS2812) using ESP-IDF RMT driver
 */
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

namespace stampfly {

class LED {
public:
    // Color definitions (RGB format)
    static constexpr uint32_t OFF     = 0x000000;
    static constexpr uint32_t WHITE   = 0xFFFFFF;
    static constexpr uint32_t RED     = 0xFF0000;
    static constexpr uint32_t GREEN   = 0x00FF00;
    static constexpr uint32_t BLUE    = 0x0000FF;
    static constexpr uint32_t YELLOW  = 0xFFFF00;
    static constexpr uint32_t PURPLE  = 0xFF00FF;
    static constexpr uint32_t CYAN    = 0x00FFFF;
    static constexpr uint32_t ORANGE  = 0xFF8000;

    enum class Pattern {
        OFF,
        SOLID,
        BLINK_SLOW,    // 1Hz
        BLINK_FAST,    // 4Hz
        BREATHE,       // Smooth fade in/out
        RAINBOW        // Cycle through colors
    };

    struct Config {
        int gpio_pin;              // WS2812 data pin (G39)
        uint8_t num_leds;          // Number of LEDs in strip
        uint8_t brightness;        // Default brightness (0-255)

        Config() : gpio_pin(39), num_leds(1), brightness(32) {}
    };

    static LED& getInstance();

    /**
     * @brief Initialize the LED driver
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Set specific LED color
     * @param index LED index (0-based)
     * @param color RGB color value
     */
    void setColor(uint8_t index, uint32_t color);

    /**
     * @brief Set all LEDs to same color
     * @param color RGB color value
     */
    void setAllColor(uint32_t color);

    /**
     * @brief Set LED pattern
     * @param pattern Pattern type
     * @param color Base color for pattern
     */
    void setPattern(Pattern pattern, uint32_t color);

    /**
     * @brief Set global brightness
     * @param brightness Brightness value (0-255)
     */
    void setBrightness(uint8_t brightness);

    /**
     * @brief Update LED output (call periodically for patterns)
     */
    void update();

    /**
     * @brief Force immediate refresh of LED strip
     */
    void refresh();

    // Predefined state displays
    void showInit();           // White blink
    void showCalibrating();    // Purple solid
    void showIdle();           // Green solid
    void showArmed();          // Yellow solid
    void showFlying();         // Yellow blink fast
    void showPairing();        // Blue blink
    void showConnected();      // Green blink
    void showDisconnected();   // Red blink
    void showLowBattery();     // Red fast blink
    void showError();          // Red solid

private:
    LED() = default;
    ~LED();
    LED(const LED&) = delete;
    LED& operator=(const LED&) = delete;

    esp_err_t initRMT();
    void sendPixel(uint32_t color);
    uint32_t applyBrightness(uint32_t color) const;
    uint32_t wheel(uint8_t pos) const;

    static void timerCallback(TimerHandle_t timer);
    void handleTimer();

    Config config_;
    bool initialized_ = false;

    // RMT handle
    void* rmt_channel_ = nullptr;
    void* rmt_encoder_ = nullptr;

    // LED state
    uint32_t* led_colors_ = nullptr;
    Pattern current_pattern_ = Pattern::OFF;
    uint32_t pattern_color_ = OFF;
    uint8_t brightness_ = 32;

    // Pattern state
    uint32_t pattern_tick_ = 0;
    bool blink_state_ = false;
    uint8_t breathe_value_ = 0;
    bool breathe_direction_ = true;
    uint8_t rainbow_offset_ = 0;

    // Timer for pattern updates
    TimerHandle_t pattern_timer_ = nullptr;
};

}  // namespace stampfly
