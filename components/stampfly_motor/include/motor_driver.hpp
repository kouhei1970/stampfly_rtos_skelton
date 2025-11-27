/**
 * @file motor_driver.hpp
 * @brief Motor Driver using ESP-IDF LEDC PWM for coreless motors
 *
 * StampFly Motor Configuration (X-quad, viewed from above):
 *       Front
 *    FL(M4)  FR(M1)
 *      \    /
 *       \  /
 *       /  \
 *      /    \
 *    RL(M3)  RR(M2)
 *       Rear
 *
 * Motor rotation: M1=CCW, M2=CW, M3=CCW, M4=CW
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

    // Motor positions (X-quad configuration)
    enum MotorID {
        MOTOR_FR = 0,  // M1: Front Right - CCW (GPIO 42)
        MOTOR_RR = 1,  // M2: Rear Right  - CW  (GPIO 41)
        MOTOR_RL = 2,  // M3: Rear Left   - CCW (GPIO 10)
        MOTOR_FL = 3   // M4: Front Left  - CW  (GPIO 5)
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
        uint8_t min_throttle;
        uint8_t max_throttle;
        uint8_t idle_throttle;

        Config() :
            pins{42, 41, 10, 5},  // FR, RR, RL, FL
            directions{CCW, CW, CCW, CW},
            pwm_freq(150000),            // 150kHz PWM frequency
            pwm_resolution(8),           // 8-bit resolution (0-255)
            min_throttle(0),             // Minimum throttle value
            max_throttle(255),           // Maximum throttle value
            idle_throttle(10)            // Idle throttle when armed
        {}
    };

    static MotorDriver& getInstance();

    /**
     * @brief Initialize the motor driver
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Arm the motors (enable output)
     * @return ESP_OK on success
     */
    esp_err_t arm();

    /**
     * @brief Disarm the motors (disable output, set to zero)
     * @return ESP_OK on success
     */
    esp_err_t disarm();

    /**
     * @brief Check if motors are armed
     */
    bool isArmed() const { return armed_; }

    /**
     * @brief Set throttle for single motor
     * @param motor_id Motor ID
     * @param throttle Throttle value (0-255)
     */
    void setThrottle(MotorID motor_id, uint8_t throttle);

    /**
     * @brief Set throttle for all motors
     * @param throttle Array of throttle values
     */
    void setAllThrottle(const uint8_t throttle[NUM_MOTORS]);

    /**
     * @brief Set mixer output (thrust + attitude control)
     * @param thrust Base thrust (0.0 - 1.0)
     * @param roll Roll command (-1.0 to 1.0)
     * @param pitch Pitch command (-1.0 to 1.0)
     * @param yaw Yaw command (-1.0 to 1.0)
     */
    void setMixerOutput(float thrust, float roll, float pitch, float yaw);

    /**
     * @brief Set raw duty cycle directly (0-255)
     * @param motor_id Motor ID
     * @param duty Duty cycle value
     */
    void setDuty(MotorID motor_id, uint8_t duty);

    /**
     * @brief Set duty for all motors directly
     * @param duty Array of duty cycle values
     */
    void setAllDuty(const uint8_t duty[NUM_MOTORS]);

    /**
     * @brief Emergency stop - immediately cut all motors
     */
    void emergencyStop();

    /**
     * @brief Test single motor at specified throttle
     * @param motor_id Motor to test
     * @param throttle Throttle value
     * @param duration_ms Test duration
     */
    void testMotor(MotorID motor_id, uint8_t throttle, uint32_t duration_ms);

    /**
     * @brief Get current throttle value for motor
     */
    uint8_t getThrottle(MotorID motor_id) const;

private:
    MotorDriver() = default;
    ~MotorDriver();
    MotorDriver(const MotorDriver&) = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    void applyThrottle();
    uint8_t clampThrottle(uint8_t throttle) const;

    Config config_;
    bool initialized_ = false;
    bool armed_ = false;
    uint8_t throttle_[NUM_MOTORS] = {0};
    SemaphoreHandle_t mutex_ = nullptr;

    // LEDC channel assignments (4-7 to avoid conflict with buzzer)
    static constexpr int LEDC_TIMER = 1;
    static constexpr int LEDC_CHANNEL_BASE = 4;
};

}  // namespace stampfly
