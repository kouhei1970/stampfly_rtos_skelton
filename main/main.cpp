/**
 * @file main.cpp
 * @brief StampFly RTOS Skeleton - Main Entry Point
 *
 * This is the main entry point for the StampFly flight controller skeleton.
 * It initializes all subsystems and starts the FreeRTOS tasks.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "StampFly RTOS Skeleton starting...");
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // TODO: Initialize all subsystems
    // - SystemManager
    // - Sensors (IMU, ToF, OptFlow, Mag, Baro, Power)
    // - Actuators (Motor, LED, Buzzer)
    // - Communication (ESP-NOW)
    // - CLI

    // TODO: Start FreeRTOS tasks
    // - IMUTask (400Hz)
    // - OptFlowTask (100Hz)
    // - MagTask (100Hz)
    // - BaroTask (50Hz)
    // - ToFTask (30Hz)
    // - PowerTask (10Hz)
    // - LEDTask (30Hz)
    // - ButtonTask (100Hz)
    // - CommTask (50Hz)

    ESP_LOGI(TAG, "StampFly RTOS Skeleton initialized (stub)");

    // Main loop placeholder
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
