/**
 * @file buzzer.cpp
 * @brief Buzzer Driver Implementation (LEDC PWM)
 */

#include "buzzer.hpp"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char* TAG = "Buzzer";

namespace stampfly {

// LEDC mode
static constexpr ledc_mode_t BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;

// Async tone state
static struct {
    bool active;
    uint32_t end_time_ms;
    ledc_channel_t channel;
} s_async_state = {false, 0, LEDC_CHANNEL_0};

esp_err_t Buzzer::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    ESP_LOGI(TAG, "Initializing Buzzer on GPIO%d", config.gpio);

    // Configure LEDC timer for buzzer
    ledc_timer_config_t timer_config = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,  // 10-bit for good frequency range
        .timer_num = static_cast<ledc_timer_t>(config.ledc_timer),
        .freq_hz = 1000,  // Default frequency, will be changed
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_config = {
        .gpio_num = config.gpio,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = static_cast<ledc_channel_t>(config.ledc_channel),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = static_cast<ledc_timer_t>(config.ledc_timer),
        .duty = 0,  // Start with 0% duty (silent)
        .hpoint = 0,
        .flags = {
            .output_invert = 0,
        },
    };

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    s_async_state.channel = static_cast<ledc_channel_t>(config.ledc_channel);

    initialized_ = true;
    ESP_LOGI(TAG, "Buzzer initialized successfully");

    return ESP_OK;
}

void Buzzer::playTone(uint16_t frequency, uint32_t duration_ms)
{
    if (!initialized_) return;

    if (frequency == 0) {
        // Rest (silence)
        ledc_set_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel), 0);
        ledc_update_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel));
    } else {
        // Set frequency
        ledc_set_freq(BUZZER_LEDC_MODE, static_cast<ledc_timer_t>(config_.ledc_timer), frequency);

        // Set 50% duty cycle for square wave tone
        uint32_t duty = (1 << 10) / 2;  // 50% of 10-bit resolution
        ledc_set_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel), duty);
        ledc_update_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel));
    }

    // Wait for duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop tone after duration
    ledc_set_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel), 0);
    ledc_update_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel));
}

void Buzzer::playToneAsync(uint16_t frequency, uint32_t duration_ms)
{
    if (!initialized_) return;

    if (frequency == 0) {
        stop();
        return;
    }

    // Set frequency
    ledc_set_freq(BUZZER_LEDC_MODE, static_cast<ledc_timer_t>(config_.ledc_timer), frequency);

    // Set 50% duty cycle for square wave tone
    uint32_t duty = (1 << 10) / 2;  // 50% of 10-bit resolution
    ledc_set_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel), duty);
    ledc_update_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel));

    // Set async state for automatic stop
    s_async_state.active = true;
    s_async_state.end_time_ms = esp_timer_get_time() / 1000 + duration_ms;
}

void Buzzer::stop()
{
    if (!initialized_) return;

    ledc_set_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel), 0);
    ledc_update_duty(BUZZER_LEDC_MODE, static_cast<ledc_channel_t>(config_.ledc_channel));
    s_async_state.active = false;
}

void Buzzer::beep()
{
    playTone(NOTE_C5, 100);
}

void Buzzer::startTone()
{
    playTone(NOTE_C5, 100);
    playTone(NOTE_E5, 100);
    playTone(NOTE_G5, 200);
}

void Buzzer::armTone()
{
    playTone(NOTE_E5, 100);
    playTone(NOTE_G5, 100);
}

void Buzzer::disarmTone()
{
    playTone(NOTE_G5, 100);
    playTone(NOTE_E5, 100);
}

void Buzzer::lowBatteryWarning()
{
    playTone(NOTE_A4, 200);
    playTone(0, 100);
    playTone(NOTE_A4, 200);
}

void Buzzer::errorTone()
{
    playTone(NOTE_C4, 500);
}

void Buzzer::pairingTone()
{
    playTone(NOTE_C5, 100);
    playTone(NOTE_G5, 100);
}

}  // namespace stampfly
