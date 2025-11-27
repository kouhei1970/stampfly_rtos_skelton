/**
 * @file buzzer.hpp
 * @brief Buzzer Control using ESP-IDF LEDC PWM
 */
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

namespace stampfly {

class Buzzer {
public:
    // Musical notes (frequencies in Hz)
    enum Note : uint16_t {
        NOTE_REST = 0,
        NOTE_C4 = 262,
        NOTE_D4 = 294,
        NOTE_E4 = 330,
        NOTE_F4 = 349,
        NOTE_G4 = 392,
        NOTE_A4 = 440,
        NOTE_B4 = 494,
        NOTE_C5 = 523,
        NOTE_D5 = 587,
        NOTE_E5 = 659,
        NOTE_F5 = 698,
        NOTE_G5 = 784,
        NOTE_A5 = 880,
        NOTE_B5 = 988,
        NOTE_C6 = 1047
    };

    struct Config {
        int gpio_pin;
        int ledc_timer;
        int ledc_channel;
        uint8_t volume;

        Config() : gpio_pin(40), ledc_timer(0), ledc_channel(0), volume(50) {}
    };

    struct ToneCommand {
        uint16_t frequency;
        uint32_t duration_ms;
    };

    static Buzzer& getInstance();

    /**
     * @brief Initialize the buzzer driver
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Play a single tone
     * @param frequency Frequency in Hz (0 = silence)
     * @param duration_ms Duration in milliseconds
     */
    void playTone(uint16_t frequency, uint32_t duration_ms);

    /**
     * @brief Play a tone without blocking
     * @param frequency Frequency in Hz
     * @param duration_ms Duration in milliseconds
     */
    void playToneAsync(uint16_t frequency, uint32_t duration_ms);

    /**
     * @brief Stop any playing tone
     */
    void stop();

    /**
     * @brief Set volume level
     * @param volume Volume level (0-100)
     */
    void setVolume(uint8_t volume);

    /**
     * @brief Check if buzzer is currently playing
     */
    bool isPlaying() const { return playing_; }

    // Predefined sound patterns
    void beep();                // Short beep
    void doubleBeep();          // Two short beeps
    void startTone();           // Power-on melody
    void armTone();             // Arming confirmation
    void disarmTone();          // Disarming confirmation
    void lowBatteryWarning();   // Low battery alert
    void errorTone();           // Error alert
    void pairingTone();         // Pairing mode
    void connectedTone();       // Connection established
    void calibrationTone();     // Calibration start/end

private:
    Buzzer() = default;
    ~Buzzer();
    Buzzer(const Buzzer&) = delete;
    Buzzer& operator=(const Buzzer&) = delete;

    void setFrequency(uint16_t frequency);
    void setDuty(uint8_t duty_percent);
    static void timerCallback(TimerHandle_t timer);
    static void taskFunction(void* param);

    Config config_;
    bool initialized_ = false;
    bool playing_ = false;
    uint8_t volume_ = 50;

    TimerHandle_t tone_timer_ = nullptr;
    QueueHandle_t tone_queue_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
};

}  // namespace stampfly
