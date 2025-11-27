/**
 * @file buzzer.cpp
 * @brief Buzzer Implementation using ESP-IDF LEDC PWM
 */
#include "buzzer.hpp"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/task.h"

static const char* TAG = "Buzzer";

namespace stampfly {

Buzzer& Buzzer::getInstance() {
    static Buzzer instance;
    return instance;
}

Buzzer::~Buzzer() {
    if (tone_timer_) {
        xTimerStop(tone_timer_, 0);
        xTimerDelete(tone_timer_, 0);
    }
    if (tone_queue_) {
        vQueueDelete(tone_queue_);
    }
    if (task_handle_) {
        vTaskDelete(task_handle_);
    }
}

esp_err_t Buzzer::init(const Config& config) {
    config_ = config;
    volume_ = config.volume;

    // Configure LEDC timer
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.timer_num = static_cast<ledc_timer_t>(config_.ledc_timer);
    timer_config.duty_resolution = LEDC_TIMER_10_BIT;
    timer_config.freq_hz = 1000;  // Initial frequency
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_config = {};
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = static_cast<ledc_channel_t>(config_.ledc_channel);
    channel_config.timer_sel = static_cast<ledc_timer_t>(config_.ledc_timer);
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.gpio_num = config_.gpio_pin;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create tone queue for async playback
    tone_queue_ = xQueueCreate(16, sizeof(ToneCommand));
    if (!tone_queue_) {
        ESP_LOGE(TAG, "Failed to create tone queue");
        return ESP_ERR_NO_MEM;
    }

    // Create tone timer for duration control
    tone_timer_ = xTimerCreate("buzzer", pdMS_TO_TICKS(100), pdFALSE, this, timerCallback);
    if (!tone_timer_) {
        ESP_LOGE(TAG, "Failed to create tone timer");
        return ESP_ERR_NO_MEM;
    }

    // Create background task for async tones
    BaseType_t task_ret = xTaskCreate(taskFunction, "buzzer_task", 2048, this, 5, &task_handle_);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create buzzer task");
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Buzzer initialized: GPIO %d", config_.gpio_pin);

    return ESP_OK;
}

void Buzzer::setFrequency(uint16_t frequency) {
    if (!initialized_ || frequency == 0) {
        setDuty(0);
        return;
    }

    ledc_set_freq(LEDC_LOW_SPEED_MODE,
                  static_cast<ledc_timer_t>(config_.ledc_timer),
                  frequency);
}

void Buzzer::setDuty(uint8_t duty_percent) {
    if (!initialized_) return;

    // Convert percentage to 10-bit value (0-1023)
    uint32_t duty = (duty_percent * 1023) / 100;

    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  static_cast<ledc_channel_t>(config_.ledc_channel),
                  duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     static_cast<ledc_channel_t>(config_.ledc_channel));
}

void Buzzer::playTone(uint16_t frequency, uint32_t duration_ms) {
    if (!initialized_) return;

    if (frequency > 0) {
        setFrequency(frequency);
        setDuty(volume_ / 2);  // 50% of volume for duty cycle
        playing_ = true;
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    setDuty(0);
    playing_ = false;
}

void Buzzer::playToneAsync(uint16_t frequency, uint32_t duration_ms) {
    if (!initialized_ || !tone_queue_) return;

    ToneCommand cmd = {frequency, duration_ms};
    xQueueSend(tone_queue_, &cmd, 0);
}

void Buzzer::stop() {
    if (!initialized_) return;

    setDuty(0);
    playing_ = false;

    // Clear the queue
    if (tone_queue_) {
        xQueueReset(tone_queue_);
    }
}

void Buzzer::setVolume(uint8_t volume) {
    if (volume > 100) volume = 100;
    volume_ = volume;
}

void Buzzer::timerCallback(TimerHandle_t timer) {
    Buzzer* buzzer = static_cast<Buzzer*>(pvTimerGetTimerID(timer));
    if (buzzer) {
        buzzer->setDuty(0);
        buzzer->playing_ = false;
    }
}

void Buzzer::taskFunction(void* param) {
    Buzzer* buzzer = static_cast<Buzzer*>(param);
    ToneCommand cmd;

    while (true) {
        if (xQueueReceive(buzzer->tone_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.frequency > 0) {
                buzzer->setFrequency(cmd.frequency);
                buzzer->setDuty(buzzer->volume_ / 2);
                buzzer->playing_ = true;
            }

            vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));

            buzzer->setDuty(0);
            buzzer->playing_ = false;
        }
    }
}

// Predefined sound patterns
void Buzzer::beep() {
    playTone(NOTE_C5, 100);
}

void Buzzer::doubleBeep() {
    playTone(NOTE_C5, 80);
    vTaskDelay(pdMS_TO_TICKS(50));
    playTone(NOTE_C5, 80);
}

void Buzzer::startTone() {
    // Rising melody
    playTone(NOTE_C5, 100);
    playTone(NOTE_E5, 100);
    playTone(NOTE_G5, 100);
    playTone(NOTE_C6, 200);
}

void Buzzer::armTone() {
    // Two ascending beeps
    playTone(NOTE_G4, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    playTone(NOTE_C5, 150);
}

void Buzzer::disarmTone() {
    // Two descending beeps
    playTone(NOTE_C5, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    playTone(NOTE_G4, 150);
}

void Buzzer::lowBatteryWarning() {
    // Three short high-pitched beeps
    for (int i = 0; i < 3; i++) {
        playTone(NOTE_A5, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Buzzer::errorTone() {
    // Low frequency error sound
    playTone(NOTE_C4, 200);
    vTaskDelay(pdMS_TO_TICKS(100));
    playTone(NOTE_C4, 200);
    vTaskDelay(pdMS_TO_TICKS(100));
    playTone(NOTE_C4, 400);
}

void Buzzer::pairingTone() {
    // Alternating tones
    playTone(NOTE_E5, 100);
    playTone(NOTE_A5, 100);
    playTone(NOTE_E5, 100);
}

void Buzzer::connectedTone() {
    // Happy ascending melody
    playTone(NOTE_C5, 80);
    playTone(NOTE_E5, 80);
    playTone(NOTE_G5, 120);
}

void Buzzer::calibrationTone() {
    // Single clear tone
    playTone(NOTE_A4, 300);
}

}  // namespace stampfly
