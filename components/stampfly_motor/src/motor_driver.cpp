/**
 * @file motor_driver.cpp
 * @brief Motor Driver Implementation (Placeholder)
 */
#include "motor_driver.hpp"
#include "esp_log.h"

static const char* TAG = "motor";

namespace stampfly {

MotorDriver& MotorDriver::getInstance() {
    static MotorDriver instance;
    return instance;
}

esp_err_t MotorDriver::init(const Config& config) {
    config_ = config;
    mutex_ = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "MotorDriver init (placeholder)");
    // TODO: Implement LEDC PWM motor control
    return ESP_OK;
}

esp_err_t MotorDriver::arm() {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    armed_ = true;
    ESP_LOGI(TAG, "Motors armed");
    if (mutex_) xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t MotorDriver::disarm() {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    armed_ = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = 0;
    }
    applyThrottle();
    ESP_LOGI(TAG, "Motors disarmed");
    if (mutex_) xSemaphoreGive(mutex_);
    return ESP_OK;
}

bool MotorDriver::isArmed() const {
    return armed_;
}

void MotorDriver::setThrottle(MotorID motor_id, uint16_t throttle) {
    if (!armed_) return;
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    throttle_[motor_id] = clampThrottle(throttle);
    applyThrottle();
    if (mutex_) xSemaphoreGive(mutex_);
}

void MotorDriver::setAllThrottle(const uint16_t throttle[NUM_MOTORS]) {
    if (!armed_) return;
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = clampThrottle(throttle[i]);
    }
    applyThrottle();
    if (mutex_) xSemaphoreGive(mutex_);
}

void MotorDriver::setMixerOutput(float thrust, float roll, float pitch, float yaw) {
    if (!armed_) return;

    // Mixer calculation (X-quad configuration)
    // M1 (FR, CCW): +thrust -roll +pitch +yaw
    // M2 (RR, CW):  +thrust -roll -pitch -yaw
    // M3 (RL, CCW): +thrust +roll -pitch +yaw
    // M4 (FL, CW):  +thrust +roll +pitch -yaw
    float m1 = thrust - roll + pitch + yaw;
    float m2 = thrust - roll - pitch - yaw;
    float m3 = thrust + roll - pitch + yaw;
    float m4 = thrust + roll + pitch - yaw;

    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    throttle_[MOTOR_FR] = clampThrottle(static_cast<uint16_t>(m1));
    throttle_[MOTOR_RR] = clampThrottle(static_cast<uint16_t>(m2));
    throttle_[MOTOR_RL] = clampThrottle(static_cast<uint16_t>(m3));
    throttle_[MOTOR_FL] = clampThrottle(static_cast<uint16_t>(m4));
    applyThrottle();
    if (mutex_) xSemaphoreGive(mutex_);
}

void MotorDriver::emergencyStop() {
    ESP_LOGW(TAG, "Emergency stop!");
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    armed_ = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = 0;
    }
    applyThrottle();
    if (mutex_) xSemaphoreGive(mutex_);
}

void MotorDriver::testMotor(MotorID motor_id, uint16_t throttle, uint32_t duration_ms) {
    ESP_LOGI(TAG, "Testing motor %d at throttle %d for %lums",
             motor_id, throttle, (unsigned long)duration_ms);
    // TODO: Implement motor test
}

void MotorDriver::applyThrottle() {
    // TODO: Implement actual PWM output
}

uint16_t MotorDriver::clampThrottle(uint16_t throttle) {
    if (throttle < config_.min_throttle) return config_.min_throttle;
    if (throttle > config_.max_throttle) return config_.max_throttle;
    return throttle;
}

}  // namespace stampfly
