/**
 * @file main.cpp
 * @brief StampFly RTOS Skeleton - Main Entry Point
 *
 * This is the main entry point for the StampFly flight controller skeleton.
 * It initializes all subsystems and starts the FreeRTOS tasks.
 */

#include <stdio.h>
#include <cmath>
#include <cstddef>  // for offsetof
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "mag_calibration.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators and peripherals
#include "motor_driver.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// State management and estimation
#include "stampfly_state.hpp"
#include "system_manager.hpp"
#include "eskf.hpp"
#include "sensor_fusion.hpp"
#include "filter.hpp"
#include "sensor_health.hpp"

// Communication
#include "controller_comm.hpp"

// CLI
#include "cli.hpp"

// Logger
#include "logger.hpp"

// Telemetry
#include "telemetry.hpp"

// Configuration
#include "config.hpp"
#include "globals.hpp"
#include "init.hpp"
#include "tasks/tasks.hpp"

static const char* TAG = "main";

// Namespace shortcuts for backward compatibility
using namespace config;
using namespace globals;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Mag参照ベクトルをバッファの平均値で設定
 *
 * ARM時またはbinlog開始時に呼び出す
 */
/**
 * @brief 加速度と地磁気のバッファ平均を使って姿勢を初期化
 *
 * 加速度計からロール/ピッチを計算し、ヨー=0で初期化。
 * 地磁気リファレンスも正しくNED座標系で設定される。
 */
void initializeAttitudeFromBuffers()
{
    if (g_accel_buffer_count == 0) {
        ESP_LOGW(TAG, "No accel samples in buffer, cannot initialize attitude");
        return;
    }
    if (g_mag_buffer_count == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot initialize attitude");
        return;
    }

    // 加速度バッファの平均を計算
    stampfly::math::Vector3 accel_sum = stampfly::math::Vector3::zero();
    int accel_count = std::min(g_accel_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < accel_count; i++) {
        accel_sum += g_accel_buffer[i];
    }
    stampfly::math::Vector3 accel_avg = accel_sum * (1.0f / accel_count);

    // 地磁気バッファの平均を計算
    stampfly::math::Vector3 mag_sum = stampfly::math::Vector3::zero();
    int mag_count = std::min(g_mag_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < mag_count; i++) {
        mag_sum += g_mag_buffer[i];
    }
    stampfly::math::Vector3 mag_avg = mag_sum * (1.0f / mag_count);

    // センサーフュージョンの姿勢を初期化
    if (g_fusion.isInitialized()) {
        g_fusion.initializeAttitude(accel_avg, mag_avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Attitude initialized from buffers: accel=%d samples, mag=%d samples",
                 accel_count, mag_count);
    }
}

/**
 * @brief 地磁気リファレンスのみをバッファから設定（レガシー互換）
 * @deprecated initializeAttitudeFromBuffers()を使用してください
 */
void setMagReferenceFromBuffer()
{
    if (g_mag_buffer_count == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot set reference");
        return;
    }

    // バッファの平均を計算
    stampfly::math::Vector3 sum = stampfly::math::Vector3::zero();
    int count = std::min(g_mag_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < count; i++) {
        sum += g_mag_buffer[i];
    }
    stampfly::math::Vector3 avg = sum * (1.0f / count);

    // センサーフュージョンに設定
    if (g_fusion.isInitialized()) {
        g_fusion.setMagReference(avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Mag reference set from %d samples: (%.1f, %.1f, %.1f) uT",
                 count, avg.x, avg.y, avg.z);
    }
}

/**
 * @brief binlog開始時のコールバック
 *
 * ESKFをリセットしてmag_refを設定する
 * PC版ESKFとの同期のため、binlog開始時に初期化
 */
void onBinlogStart()
{
    // センサーフュージョンをリセット（PC版と同じ初期状態にする）
    if (g_fusion.isInitialized()) {
        g_fusion.reset();
        // ジャイロバイアスを復元（reset()でゼロになるため）
        g_fusion.setGyroBias(g_initial_gyro_bias);
        ESP_LOGI(TAG, "Sensor fusion reset for binlog, gyro bias restored");
    }

    // 姿勢を初期化（バッファの最新値で更新）
    initializeAttitudeFromBuffers();
}

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * @brief ESP Timer callback for 400Hz IMU timing
 *
 * This callback runs from the esp_timer task context and gives a semaphore
 * to wake up the IMU task with precise 2.5ms timing.
 *
 * Note: ESP_TIMER_TASK dispatch runs in a high-priority FreeRTOS task,
 * not in ISR context, so we use xSemaphoreGive instead of xSemaphoreGiveFromISR.
 */
static void imu_timer_callback(void* arg)
{
    static uint32_t timer_count = 0;
    static uint32_t last_imu_loop = 0;
    static uint32_t last_optflow_loop = 0;
    timer_count++;

    // 10秒ごと（4000回 @ 400Hz）にタイマー生存確認
    if (timer_count % 4000 == 0) {
        // IMUタスクが進行しているかチェック
        if (g_imu_last_loop == last_imu_loop) {
            ESP_LOGW(TAG, "IMU STUCK at checkpoint=%u, loop=%lu",
                     g_imu_checkpoint, g_imu_last_loop);
        } else {
            ESP_LOGI(TAG, "IMU timer alive: count=%lu, imu_loop=%lu",
                     timer_count, g_imu_last_loop);
        }
        last_imu_loop = g_imu_last_loop;

        // OptFlowタスクが進行しているかチェック（10秒で1000回 @ 100Hz）
        if (g_optflow_last_loop == last_optflow_loop && g_optflow_last_loop > 0) {
            ESP_LOGW(TAG, "OptFlow STUCK at checkpoint=%u, loop=%lu",
                     g_optflow_checkpoint, g_optflow_last_loop);
        }
        last_optflow_loop = g_optflow_last_loop;
    }
    xSemaphoreGive(g_imu_semaphore);
}

// =============================================================================
// Task Functions
// =============================================================================

/**
 * @brief IMU Task - 400Hz (2.5ms period)
 * Reads BMI270 FIFO, applies filters, updates estimators
 *
 * Timing: Uses ESP Timer for precise 2.5ms (400Hz) period.
 * The timer callback gives a semaphore which this task waits on.
 */

// =============================================================================
// Button Event Handler
// =============================================================================

void onButtonEvent(stampfly::Button::Event event)
{
    auto& state = stampfly::StampFlyState::getInstance();

    switch (event) {
        case stampfly::Button::Event::CLICK:
            ESP_LOGI(TAG, "Button: CLICK");
            // Toggle arm/disarm in IDLE state
            if (state.getFlightState() == stampfly::FlightState::IDLE) {
                if (state.requestArm()) {
                    // ARM時に姿勢を初期化（現在の向き=Yaw 0°）
                    initializeAttitudeFromBuffers();
                    g_buzzer.armTone();
                    ESP_LOGI(TAG, "Motors ARMED");
                }
            } else if (state.getFlightState() == stampfly::FlightState::ARMED) {
                if (state.requestDisarm()) {
                    g_buzzer.disarmTone();
                    ESP_LOGI(TAG, "Motors DISARMED");
                }
            }
            break;

        case stampfly::Button::Event::DOUBLE_CLICK:
            ESP_LOGI(TAG, "Button: DOUBLE_CLICK");
            break;

        case stampfly::Button::Event::LONG_PRESS_START:
            ESP_LOGI(TAG, "Button: LONG_PRESS_START");
            break;

        case stampfly::Button::Event::LONG_PRESS_3S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (3s) - Entering pairing mode");
            g_comm.enterPairingMode();
            state.setPairingState(stampfly::PairingState::PAIRING);
            g_led.setPattern(stampfly::LED::Pattern::BLINK_FAST, 0x0000FF);  // Blue fast blink
            g_buzzer.beep();
            break;

        case stampfly::Button::Event::LONG_PRESS_5S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (5s) - System reset");
            g_buzzer.beep();
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_restart();
            break;

        default:
            break;
    }
}

// =============================================================================
// Control Packet Handler
// =============================================================================

void onControlPacket(const stampfly::ControlPacket& packet)
{
    auto& state = stampfly::StampFlyState::getInstance();

    // Update control inputs (raw ADC values)
    state.updateControlInput(
        packet.throttle,
        packet.roll,
        packet.pitch,
        packet.yaw
    );

    // Update control flags
    state.updateControlFlags(packet.flags);

    // Handle arm/disarm toggle from controller (rising edge detection)
    static bool prev_arm_flag = false;
    bool arm_flag = (packet.flags & stampfly::CTRL_FLAG_ARM) != 0;

    // Detect rising edge (button press)
    if (arm_flag && !prev_arm_flag) {
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state == stampfly::FlightState::IDLE ||
            flight_state == stampfly::FlightState::ERROR) {
            // IDLE/ERROR → ARM
            if (state.requestArm()) {
                g_motor.arm();  // Enable motor driver
                initializeAttitudeFromBuffers();
                g_buzzer.armTone();
                ESP_LOGI(TAG, "Motors ARMED (from controller)");
            }
        } else if (flight_state == stampfly::FlightState::ARMED) {
            // ARMED → DISARM
            if (state.requestDisarm()) {
                g_motor.disarm();  // Disable motor driver and stop motors
                g_buzzer.disarmTone();
                ESP_LOGI(TAG, "Motors DISARMED (from controller)");
            }
        }
    }
    prev_arm_flag = arm_flag;
}


static void startTasks()
{
    ESP_LOGI(TAG, "Starting FreeRTOS tasks...");

    // Peripheral tasks (Core 0)
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", STACK_SIZE_LED, nullptr,
                            PRIORITY_LED_TASK, &g_led_task_handle, 0);

    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", STACK_SIZE_BUTTON, nullptr,
                            PRIORITY_BUTTON_TASK, &g_button_task_handle, 0);

    xTaskCreatePinnedToCore(PowerTask, "PowerTask", STACK_SIZE_POWER, nullptr,
                            PRIORITY_POWER_TASK, &g_power_task_handle, 0);

    // Initialize IMU timer and semaphores for precise 400Hz timing
    g_imu_semaphore = xSemaphoreCreateBinary();
    if (g_imu_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create IMU semaphore");
    }

    g_control_semaphore = xSemaphoreCreateBinary();
    if (g_control_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create control semaphore");
    }

    esp_timer_create_args_t imu_timer_args = {
        .callback = imu_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,  // Run in esp_timer task context
        .name = "imu_timer",
        .skip_unhandled_events = true,      // Skip if semaphore already given
    };
    esp_err_t ret = esp_timer_create(&imu_timer_args, &g_imu_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IMU timer: %s", esp_err_to_name(ret));
    } else {
        // Start periodic timer: 2500μs = 2.5ms = 400Hz
        ret = esp_timer_start_periodic(g_imu_timer, 2500);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start IMU timer: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU timer started: 2.5ms period (400Hz)");
        }
    }

    // Sensor tasks (Core 1)
    xTaskCreatePinnedToCore(IMUTask, "IMUTask", STACK_SIZE_IMU, nullptr,
                            PRIORITY_IMU_TASK, &g_imu_task_handle, 1);

    // Control task (Core 1) - runs at 400Hz, synchronized with IMU via semaphore
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", STACK_SIZE_CONTROL, nullptr,
                            PRIORITY_CONTROL_TASK, &g_control_task_handle, 1);

    // I2C sensor tasks (Core 0) - moved from Core 1 to avoid contention with 400Hz IMU
    xTaskCreatePinnedToCore(MagTask, "MagTask", STACK_SIZE_MAG, nullptr,
                            PRIORITY_MAG_TASK, &g_mag_task_handle, 0);

    xTaskCreatePinnedToCore(BaroTask, "BaroTask", STACK_SIZE_BARO, nullptr,
                            PRIORITY_BARO_TASK, &g_baro_task_handle, 0);

    xTaskCreatePinnedToCore(ToFTask, "ToFTask", STACK_SIZE_TOF, nullptr,
                            PRIORITY_TOF_TASK, &g_tof_task_handle, 0);

    xTaskCreatePinnedToCore(OptFlowTask, "OptFlowTask", STACK_SIZE_OPTFLOW, nullptr,
                            PRIORITY_OPTFLOW_TASK, &g_optflow_task_handle, 1);

    // Communication task (Core 0)
    xTaskCreatePinnedToCore(CommTask, "CommTask", STACK_SIZE_COMM, nullptr,
                            PRIORITY_COMM_TASK, &g_comm_task_handle, 0);

    // CLI task (Core 0)
    xTaskCreatePinnedToCore(CLITask, "CLITask", STACK_SIZE_CLI, nullptr,
                            PRIORITY_CLI_TASK, &g_cli_task_handle, 0);

    // Telemetry task (Core 0) - WebSocket broadcast at 50Hz
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", STACK_SIZE_TELEMETRY, nullptr,
                            PRIORITY_TELEMETRY_TASK, &g_telemetry_task_handle, 0);

    ESP_LOGI(TAG, "All tasks started");
}

// =============================================================================
// Main Entry Point
// =============================================================================

extern "C" void app_main(void)
{
    // Delay to allow USB to connect for debug output
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Direct printf for early debug (before ESP_LOG may be configured)
    printf("\n\n*** StampFly Boot Start ***\n\n");
    fflush(stdout);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  StampFly RTOS Skeleton");
    ESP_LOGI(TAG, "  ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize network interface and event loop (required for WiFi/ESP-NOW)
    ESP_LOGI(TAG, "Initializing network interface...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network interface initialized");

    // Initialize state manager
    ESP_LOGI(TAG, "Initializing state manager...");
    auto& state = stampfly::StampFlyState::getInstance();
    ESP_ERROR_CHECK(state.init());

    // Initialize system manager
    ESP_LOGI(TAG, "Initializing system manager...");
    auto& sys_mgr = stampfly::SystemManager::getInstance();
    stampfly::SystemManager::Config sys_cfg;
    sys_cfg.init_timeout_ms = 5000;
    sys_cfg.calib_timeout_ms = 10000;
    ESP_ERROR_CHECK(sys_mgr.init(sys_cfg));

    // Initialize I2C bus first (required for I2C sensors)
    ESP_LOGI(TAG, "Initializing I2C...");
    init::i2c();

    // Initialize actuators first (for buzzer feedback)
    ESP_LOGI(TAG, "Initializing actuators...");
    init::actuators();

    // Play startup tone early to indicate boot progress
    ESP_LOGI(TAG, "Playing startup tone...");
    g_buzzer.startTone();
    g_led.showInit();

    // Initialize sensors (may fail partially, that's OK)
    ESP_LOGI(TAG, "Initializing sensors...");
    init::sensors();

    // Initialize estimators
    ESP_LOGI(TAG, "Initializing estimators...");
    init::estimators();

    // Initialize communication (ESP-NOW)
    ESP_LOGI(TAG, "Initializing communication...");
    init::communication();

    // Initialize CLI
    ESP_LOGI(TAG, "Initializing CLI...");
    init::cli();

    // Initialize Logger (400Hz binary log)
    ESP_LOGI(TAG, "Initializing Logger...");
    init::logger();

    // Initialize Telemetry (WebSocket server)
    ESP_LOGI(TAG, "Initializing Telemetry...");
    init::telemetry();

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    startTasks();

    // Transition to IDLE state after initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    state.setFlightState(stampfly::FlightState::IDLE);

    // Wait for sensors to stabilize (sensor fusion is NOT running during this period)
    // g_eskf_ready = false なので IMUTask は sensor fusion 処理をスキップしている
    ESP_LOGI(TAG, "Waiting for sensors to stabilize (sensor fusion paused)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Initialize sensor fusion with fresh state and stable sensor data
    // センサーが安定した状態でリセット・キャリブレーション
    if (g_fusion.isInitialized()) {
        g_fusion.reset();
        g_fusion.setGyroBias(g_initial_gyro_bias);
        initializeAttitudeFromBuffers();

        // センサーフュージョン処理を開始
        g_eskf_ready = true;
        ESP_LOGI(TAG, "Sensor fusion initialized with stable sensor data, ready to run");
    }

    // ESKF ready notification - 3 short beeps + green LED
    ESP_LOGI(TAG, "ESKF ready - playing notification...");
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();
    g_led.setPattern(stampfly::LED::Pattern::SOLID, 0x00FF00);  // Green = ready

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "StampFly initialized successfully!");
    ESP_LOGI(TAG, "ESKF ready - you can start logging now");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
    // Log level is now managed by CLI (loaded from NVS at CLI::init())
    // Use 'loglevel' command to change. Default (no NVS): none

    // Main loop - just monitor system health
    while (true) {
        // Heap monitoring disabled by default (ESP_LOG suppressed)
        // Use 'loglevel info' CLI command to re-enable if needed

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
