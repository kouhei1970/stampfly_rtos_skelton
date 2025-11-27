/**
 * @file buzzer.hpp
 * @brief Buzzer Control
 */
#pragma once
#include <cstdint>
#include "esp_err.h"

namespace stampfly {

class Buzzer {
public:
    enum Note : uint16_t {
        NOTE_D1 = 294,
        NOTE_D2 = 330,
        NOTE_D3 = 350,
        NOTE_D4 = 393,
        NOTE_D5 = 441,
        NOTE_D6 = 495,
        NOTE_D7 = 556
    };

    struct Config {
        int pin;
        int ledc_channel;
        int ledc_timer;
    };

    static Buzzer& getInstance();

    esp_err_t init(const Config& config = Config{});
    void playTone(uint16_t frequency, uint32_t duration_ms);
    void stop();
    void beep();
    void startTone();
    void armTone();
    void disarmTone();
    void lowBatteryWarning();
    void errorTone();
    void pairingTone();

private:
    Buzzer() = default;
};

}  // namespace stampfly
