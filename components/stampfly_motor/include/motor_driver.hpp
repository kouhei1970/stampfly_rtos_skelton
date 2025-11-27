/**
 * @file motor_driver.hpp
 * @brief Motor Driver (PWM)
 */
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace stampfly {

class MotorDriver {
public:
    static constexpr uint8_t NUM_MOTORS = 4;

    // Motor positions (clockwise from front-right)
    enum MotorID {
        MOTOR_FR = 0,  // M1: Front Right - CCW
        MOTOR_RR = 1,  // M2: Rear Right  - CW
        MOTOR_RL = 2,  // M3: Rear Left   - CCW
        MOTOR_FL = 3   // M4: Front Left  - CW
    };

    enum RotationDir {
        CW = 0,
        CCW = 1
    };

    struct Config {
        int pins[NUM_MOTORS];
        RotationDir directions[NUM_MOTORS];
        uint32_t pwm_freq;
        uint8_t pwm_resolution;
        uint16_t min_throttle;
        uint16_t max_throttle;
        uint16_t idle_throttle;
    };

    static MotorDriver& getInstance();

    esp_err_t init(const Config& config = Config{});

    esp_err_t arm();
    esp_err_t disarm();
    bool isArmed() const;

    void setThrottle(MotorID motor_id, uint16_t throttle);
    void setAllThrottle(const uint16_t throttle[NUM_MOTORS]);
    void setMixerOutput(float thrust, float roll, float pitch, float yaw);

    void emergencyStop();
    void testMotor(MotorID motor_id, uint16_t throttle, uint32_t duration_ms);

private:
    MotorDriver() = default;
    Config config_;
    bool armed_ = false;
    uint16_t throttle_[NUM_MOTORS] = {0};
    SemaphoreHandle_t mutex_ = nullptr;

    void applyThrottle();
    uint16_t clampThrottle(uint16_t throttle);
};

}  // namespace stampfly
