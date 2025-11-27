/**
 * @file motor_driver.cpp
 * @brief Motor Driver Implementation using ESP-IDF LEDC PWM
 */
#include "motor_driver.hpp"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "Motor";

namespace stampfly {

MotorDriver& MotorDriver::getInstance() {
    static MotorDriver instance;
    return instance;
}

MotorDriver::~MotorDriver() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t MotorDriver::init(const Config& config) {
    config_ = config;

    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure LEDC timer for motors
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.timer_num = static_cast<ledc_timer_t>(LEDC_TIMER);
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(config_.pwm_resolution);
    timer_config.freq_hz = config_.pwm_freq;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channels for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_channel_config_t channel_config = {};
        channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_config.channel = static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i);
        channel_config.timer_sel = static_cast<ledc_timer_t>(LEDC_TIMER);
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.gpio_num = config_.pins[i];
        channel_config.duty = 0;
        channel_config.hpoint = 0;

        ret = ledc_channel_config(&channel_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGI(TAG, "Motor %d configured on GPIO %d", i, config_.pins[i]);
    }

    // Initialize all motors to zero
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = 0;
    }
    applyThrottle();

    initialized_ = true;
    ESP_LOGI(TAG, "Motor driver initialized: %lu Hz PWM, %d-bit resolution",
             config_.pwm_freq, config_.pwm_resolution);

    return ESP_OK;
}

esp_err_t MotorDriver::arm() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(mutex_, portMAX_DELAY);

    armed_ = true;

    // Set idle throttle on all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = config_.idle_throttle;
    }
    applyThrottle();

    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Motors armed (idle throttle: %d)", config_.idle_throttle);
    return ESP_OK;
}

esp_err_t MotorDriver::disarm() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(mutex_, portMAX_DELAY);

    armed_ = false;

    // Stop all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = 0;
    }
    applyThrottle();

    xSemaphoreGive(mutex_);

    ESP_LOGI(TAG, "Motors disarmed");
    return ESP_OK;
}

void MotorDriver::setThrottle(MotorID motor_id, uint8_t throttle) {
    if (!initialized_ || !armed_ || motor_id >= NUM_MOTORS) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    throttle_[motor_id] = clampThrottle(throttle);
    applyThrottle();
    xSemaphoreGive(mutex_);
}

void MotorDriver::setAllThrottle(const uint8_t throttle[NUM_MOTORS]) {
    if (!initialized_ || !armed_) return;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = clampThrottle(throttle[i]);
    }
    applyThrottle();
    xSemaphoreGive(mutex_);
}

void MotorDriver::setMixerOutput(float thrust, float roll, float pitch, float yaw) {
    if (!initialized_ || !armed_) return;

    // Clamp inputs
    thrust = std::fmax(0.0f, std::fmin(1.0f, thrust));
    roll = std::fmax(-1.0f, std::fmin(1.0f, roll));
    pitch = std::fmax(-1.0f, std::fmin(1.0f, pitch));
    yaw = std::fmax(-1.0f, std::fmin(1.0f, yaw));

    // Convert to 0-255 scale
    float thrust_scaled = thrust * 255.0f;
    float roll_scaled = roll * 50.0f;     // Scale factor for roll
    float pitch_scaled = pitch * 50.0f;   // Scale factor for pitch
    float yaw_scaled = yaw * 30.0f;       // Scale factor for yaw

    // X-quad mixer calculation
    // M1 (FR, CCW): +thrust -roll +pitch +yaw
    // M2 (RR, CW):  +thrust -roll -pitch -yaw
    // M3 (RL, CCW): +thrust +roll -pitch +yaw
    // M4 (FL, CW):  +thrust +roll +pitch -yaw
    float m1 = thrust_scaled - roll_scaled + pitch_scaled + yaw_scaled;
    float m2 = thrust_scaled - roll_scaled - pitch_scaled - yaw_scaled;
    float m3 = thrust_scaled + roll_scaled - pitch_scaled + yaw_scaled;
    float m4 = thrust_scaled + roll_scaled + pitch_scaled - yaw_scaled;

    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Clamp and apply
    throttle_[MOTOR_FR] = clampThrottle(static_cast<uint8_t>(std::fmax(0.0f, std::fmin(255.0f, m1))));
    throttle_[MOTOR_RR] = clampThrottle(static_cast<uint8_t>(std::fmax(0.0f, std::fmin(255.0f, m2))));
    throttle_[MOTOR_RL] = clampThrottle(static_cast<uint8_t>(std::fmax(0.0f, std::fmin(255.0f, m3))));
    throttle_[MOTOR_FL] = clampThrottle(static_cast<uint8_t>(std::fmax(0.0f, std::fmin(255.0f, m4))));

    applyThrottle();
    xSemaphoreGive(mutex_);
}

void MotorDriver::setDuty(MotorID motor_id, uint8_t duty) {
    if (!initialized_ || motor_id >= NUM_MOTORS) return;

    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id),
                  duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id));
}

void MotorDriver::setAllDuty(const uint8_t duty[NUM_MOTORS]) {
    if (!initialized_) return;

    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,
                      static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i),
                      duty[i]);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,
                         static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i));
    }
}

void MotorDriver::emergencyStop() {
    ESP_LOGW(TAG, "EMERGENCY STOP!");

    // Don't wait for mutex - immediate stop is critical
    armed_ = false;

    // Immediately set all motors to zero
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle_[i] = 0;
        ledc_set_duty(LEDC_LOW_SPEED_MODE,
                      static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i),
                      0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,
                         static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i));
    }
}

void MotorDriver::testMotor(MotorID motor_id, uint8_t throttle, uint32_t duration_ms) {
    if (!initialized_ || motor_id >= NUM_MOTORS) return;

    ESP_LOGI(TAG, "Testing motor %d at throttle %d for %lu ms",
             motor_id, throttle, duration_ms);

    // Set single motor
    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id),
                  throttle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id));

    // Wait
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop
    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id),
                  0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + motor_id));

    ESP_LOGI(TAG, "Motor test complete");
}

uint8_t MotorDriver::getThrottle(MotorID motor_id) const {
    if (motor_id >= NUM_MOTORS) return 0;
    return throttle_[motor_id];
}

void MotorDriver::applyThrottle() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,
                      static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i),
                      throttle_[i]);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,
                         static_cast<ledc_channel_t>(LEDC_CHANNEL_BASE + i));
    }
}

uint8_t MotorDriver::clampThrottle(uint8_t throttle) const {
    if (throttle < config_.min_throttle) return config_.min_throttle;
    if (throttle > config_.max_throttle) return config_.max_throttle;
    return throttle;
}

}  // namespace stampfly
