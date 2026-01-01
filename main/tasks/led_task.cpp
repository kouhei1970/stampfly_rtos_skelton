/**
 * @file led_task.cpp
 * @brief LEDタスク (30Hz) - LED状態更新
 */

#include "tasks_common.hpp"

static const char* TAG = "LEDTask";

using namespace config;
using namespace globals;

void LEDTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LEDTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(32);  // ~30Hz

    // Low battery threshold for battery replace warning (cyan)
    constexpr float LOW_BATTERY_THRESHOLD = 3.4f;

    auto& state = stampfly::StampFlyState::getInstance();
    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;
    bool prev_low_battery = false;

    while (true) {
        // Check for low battery (< 3.4V = battery replace warning)
        float voltage = state.getVoltage();
        bool low_battery = (voltage > 0.5f) && (voltage < LOW_BATTERY_THRESHOLD);

        // Update LED pattern based on flight state
        stampfly::FlightState flight_state = state.getFlightState();

        // Low battery takes priority over flight state
        if (low_battery != prev_low_battery || flight_state != prev_flight_state) {
            if (low_battery) {
                // Battery replace warning - cyan blink
                g_led.showLowBatteryCyan();
            } else {
                switch (flight_state) {
                    case stampfly::FlightState::INIT:
                        g_led.showInit();
                        break;
                    case stampfly::FlightState::CALIBRATING:
                        g_led.showCalibrating();
                        break;
                    case stampfly::FlightState::IDLE:
                        g_led.showIdle();
                        break;
                    case stampfly::FlightState::ARMED:
                        g_led.showArmed();
                        break;
                    case stampfly::FlightState::FLYING:
                        g_led.showFlying();
                        break;
                    case stampfly::FlightState::LANDING:
                        g_led.showLanding();
                        break;
                    case stampfly::FlightState::ERROR:
                        g_led.showError();
                        break;
                }
            }
            prev_flight_state = flight_state;
            prev_low_battery = low_battery;
        }

        // Update LED animation
        g_led.update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
