/**
 * @file motor_driver.cpp
 * @brief Motor Driver Implementation (Stub)
 */

#include "motor_driver.hpp"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = "MotorDriver";

namespace stampfly {

esp_err_t MotorDriver::init(const Config& config)
{
    ESP_LOGI(TAG, "Initializing Motor Driver (stub)");
    ESP_LOGI(TAG, "  M1(FR): GPIO%d, M2(RR): GPIO%d, M3(RL): GPIO%d, M4(FL): GPIO%d",
             config.gpio[0], config.gpio[1], config.gpio[2], config.gpio[3]);
    config_ = config;
    // TODO: Implement LEDC PWM initialization
    initialized_ = true;
    return ESP_OK;
}

esp_err_t MotorDriver::arm()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Motors armed");
    armed_ = true;
    return ESP_OK;
}

esp_err_t MotorDriver::disarm()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Motors disarmed");
    armed_ = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_output_[i] = 0.0f;
    }
    // TODO: Set all PWM outputs to 0
    return ESP_OK;
}

void MotorDriver::setMotor(int motor, float value)
{
    if (!initialized_ || !armed_ || motor < 0 || motor >= NUM_MOTORS) {
        return;
    }
    motor_output_[motor] = std::clamp(value, 0.0f, 1.0f);
    // TODO: Set PWM duty cycle
}

void MotorDriver::setMixerOutput(float thrust, float roll, float pitch, float yaw)
{
    if (!initialized_ || !armed_) {
        return;
    }

    // X-quad mixer
    // M1 (FR, CCW): T - R + P + Y
    // M2 (RR, CW):  T - R - P - Y
    // M3 (RL, CCW): T + R - P + Y
    // M4 (FL, CW):  T + R + P - Y
    float m1 = thrust - roll + pitch + yaw;
    float m2 = thrust - roll - pitch - yaw;
    float m3 = thrust + roll - pitch + yaw;
    float m4 = thrust + roll + pitch - yaw;

    setMotor(MOTOR_FR, m1);
    setMotor(MOTOR_RR, m2);
    setMotor(MOTOR_RL, m3);
    setMotor(MOTOR_FL, m4);
}

void MotorDriver::testMotor(int motor, int throttle_percent)
{
    if (!initialized_ || motor < 0 || motor >= NUM_MOTORS) {
        return;
    }
    float value = std::clamp(throttle_percent, 0, 100) / 100.0f;
    ESP_LOGI(TAG, "Testing motor %d at %d%%", motor + 1, throttle_percent);
    // TODO: Directly set PWM for testing (bypasses arm check)
}

}  // namespace stampfly
