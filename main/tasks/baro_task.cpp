/**
 * @file baro_task.cpp
 * @brief 気圧タスク (50Hz) - BMP280読み取り
 */

#include "tasks_common.hpp"

static const char* TAG = "BaroTask";

using namespace config;
using namespace globals;

void BaroTask(void* pvParameters)
{
    ESP_LOGI(TAG, "BaroTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスモニター設定
    g_health.baro.setThresholds(5, 10);  // 5連続成功/10連続失敗

    while (true) {
        if (g_baro.isInitialized()) {
            stampfly::BaroData baro;
            if (g_baro.read(baro) == ESP_OK) {
                g_health.baro.recordSuccess();
                g_baro_task_healthy = g_health.baro.isHealthy();
                // Use altitude from read() directly (already calculated)
                state.updateBaro(baro.pressure_pa, baro.temperature_c, baro.altitude_m);

                // 初回測定で基準高度を設定
                if (!g_baro_reference_set) {
                    g_baro_reference_altitude = baro.altitude_m;
                    g_baro_reference_set = true;
                    state.setBaroReferenceAltitude(g_baro_reference_altitude);
                    ESP_LOGI(TAG, "Baro reference set: %.3f m", g_baro_reference_altitude);
                }

                // ESKF用データをキャッシュしてフラグを立てる（50Hz）
                // ESKF updateはIMUTask内で行う（レースコンディション防止）
                if (g_baro_reference_set) {
                    float relative_alt = baro.altitude_m - g_baro_reference_altitude;
                    g_baro_data_cache = relative_alt;
                    g_baro_data_ready = true;
                }

                // Fallback to simple altitude estimator (センサーフュージョン未使用時)
                if (!g_fusion.isInitialized() && g_altitude_est.isInitialized()) {
                    g_altitude_est.updateBaro(baro.altitude_m);
                }
            } else {
                g_health.baro.recordFailure();
                g_baro_task_healthy = g_health.baro.isHealthy();
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
