/**
 * @file power_task.cpp
 * @brief 電源監視タスク (10Hz) - INA3221読み取り、バッテリー状態監視
 */

#include "tasks_common.hpp"

static const char* TAG = "PowerTask";

using namespace config;
using namespace globals;

void PowerTask(void* pvParameters)
{
    ESP_LOGI(TAG, "PowerTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz

    auto& state = stampfly::StampFlyState::getInstance();
    static uint32_t log_counter = 0;
    static bool first_read = true;
    static bool low_battery_warned = false;

    while (true) {
        if (g_power.isInitialized()) {
            stampfly::PowerData power;
            if (g_power.read(power) == ESP_OK) {
                state.updatePower(power.voltage_v, power.current_ma / 1000.0f);

                // Log first reading immediately, then every 5 seconds
                if (first_read || ++log_counter >= 50) {
                    ESP_LOGI(TAG, "Battery: %.2fV, %.1fmA, LowBat=%d",
                             power.voltage_v, power.current_ma, g_power.isLowBattery());
                    log_counter = 0;
                    first_read = false;
                }

                // Low battery warning (only warn once to avoid continuous buzzing)
                if (g_power.isLowBattery() && !low_battery_warned) {
                    ESP_LOGW(TAG, "LOW BATTERY WARNING: %.2fV", power.voltage_v);
                    state.setError(stampfly::ErrorCode::LOW_BATTERY);
                    g_led.setPattern(stampfly::LED::Pattern::BLINK_FAST, 0xFF0000);
                    g_buzzer.lowBatteryWarning();
                    low_battery_warned = true;
                }
                // Reset warning flag when battery is charged again
                if (!g_power.isLowBattery()) {
                    low_battery_warned = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
