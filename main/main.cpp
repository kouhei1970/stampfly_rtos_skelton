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
#include "esp_netif.h"
#include "esp_event.h"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
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
#include "filter.hpp"

// Communication
#include "controller_comm.hpp"

// CLI
#include "cli.hpp"

static const char* TAG = "main";

// =============================================================================
// GPIO Definitions (from implementation_plan.md)
// =============================================================================

// SPI Bus
static constexpr int GPIO_SPI_MOSI = 14;
static constexpr int GPIO_SPI_MISO = 43;
static constexpr int GPIO_SPI_SCK = 44;
static constexpr int GPIO_IMU_CS = 46;
static constexpr int GPIO_FLOW_CS = 12;

// I2C Bus
static constexpr int GPIO_I2C_SDA = 3;
static constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT
static constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
static constexpr int GPIO_TOF_XSHUT_FRONT = 9;

// Motors (LEDC PWM)
static constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
static constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
static constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
static constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
static constexpr int GPIO_LED = 39;
static constexpr int GPIO_BUZZER = 40;
static constexpr int GPIO_BUTTON = 0;

// =============================================================================
// Task Priorities (from implementation_plan.md)
// =============================================================================

static constexpr UBaseType_t PRIORITY_IMU_TASK = 24;
static constexpr UBaseType_t PRIORITY_OPTFLOW_TASK = 20;
static constexpr UBaseType_t PRIORITY_MAG_TASK = 18;
static constexpr UBaseType_t PRIORITY_BARO_TASK = 16;
static constexpr UBaseType_t PRIORITY_COMM_TASK = 15;
static constexpr UBaseType_t PRIORITY_TOF_TASK = 14;
static constexpr UBaseType_t PRIORITY_POWER_TASK = 12;
static constexpr UBaseType_t PRIORITY_BUTTON_TASK = 10;
static constexpr UBaseType_t PRIORITY_LED_TASK = 8;
static constexpr UBaseType_t PRIORITY_CLI_TASK = 5;

// =============================================================================
// Task Stack Sizes
// =============================================================================

static constexpr uint32_t STACK_SIZE_IMU = 8192;
static constexpr uint32_t STACK_SIZE_OPTFLOW = 8192;
static constexpr uint32_t STACK_SIZE_MAG = 8192;
static constexpr uint32_t STACK_SIZE_BARO = 8192;
static constexpr uint32_t STACK_SIZE_TOF = 8192;
static constexpr uint32_t STACK_SIZE_POWER = 4096;
static constexpr uint32_t STACK_SIZE_LED = 4096;
static constexpr uint32_t STACK_SIZE_BUTTON = 4096;
static constexpr uint32_t STACK_SIZE_COMM = 4096;
static constexpr uint32_t STACK_SIZE_CLI = 4096;

// =============================================================================
// Global Component Instances
// =============================================================================

namespace {
    // Sensors
    stampfly::BMI270Wrapper g_imu;
    stampfly::BMM150 g_mag;
    stampfly::BMP280 g_baro;
    stampfly::VL53L3CXWrapper g_tof_bottom;
    stampfly::VL53L3CXWrapper g_tof_front;
    stampfly::PMW3901* g_optflow = nullptr;
    stampfly::PowerMonitor g_power;

    // Actuators
    stampfly::MotorDriver g_motor;
    stampfly::LED g_led;
    stampfly::Buzzer g_buzzer;
    stampfly::Button g_button;

    // Estimators
    stampfly::ESKF g_eskf;
    stampfly::AttitudeEstimator g_attitude_est;
    stampfly::AltitudeEstimator g_altitude_est;

    // Filters for IMU
    stampfly::LowPassFilter g_accel_lpf[3];
    stampfly::LowPassFilter g_gyro_lpf[3];

    // Communication
    stampfly::ControllerComm g_comm;

    // CLI
    stampfly::CLI g_cli;

    // Task handles
    TaskHandle_t g_imu_task_handle = nullptr;
    TaskHandle_t g_optflow_task_handle = nullptr;
    TaskHandle_t g_mag_task_handle = nullptr;
    TaskHandle_t g_baro_task_handle = nullptr;
    TaskHandle_t g_tof_task_handle = nullptr;
    TaskHandle_t g_power_task_handle = nullptr;
    TaskHandle_t g_led_task_handle = nullptr;
    TaskHandle_t g_button_task_handle = nullptr;
    TaskHandle_t g_comm_task_handle = nullptr;
    TaskHandle_t g_cli_task_handle = nullptr;
}

// =============================================================================
// Task Functions
// =============================================================================

/**
 * @brief IMU Task - 400Hz (2.5ms period)
 * Reads BMI270 FIFO, applies filters, updates estimators
 */
static void IMUTask(void* pvParameters)
{
    ESP_LOGI(TAG, "IMUTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);  // ~400Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_imu.isInitialized()) {
            stampfly::AccelData accel;
            stampfly::GyroData gyro;

            if (g_imu.readSensorData(accel, gyro) == ESP_OK) {
                // Apply low-pass filters
                float filtered_accel[3] = {
                    g_accel_lpf[0].apply(accel.x),
                    g_accel_lpf[1].apply(accel.y),
                    g_accel_lpf[2].apply(accel.z)
                };
                float filtered_gyro[3] = {
                    g_gyro_lpf[0].apply(gyro.x),
                    g_gyro_lpf[1].apply(gyro.y),
                    g_gyro_lpf[2].apply(gyro.z)
                };

                // Update state
                stampfly::StateVector3 accel_vec(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                stampfly::StateVector3 gyro_vec(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);
                state.updateIMU(accel_vec, gyro_vec);

                // TODO: Re-enable after debugging
                // Update ESKF predict step
                // if (g_eskf.isInitialized()) {
                //     stampfly::math::Vector3 a(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                //     stampfly::math::Vector3 g(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);
                //     g_eskf.predict(a, g, 0.0025f);  // 2.5ms
                // }

                // Update simple attitude estimator
                // if (g_attitude_est.isInitialized()) {
                //     stampfly::math::Vector3 a(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                //     stampfly::math::Vector3 g(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);
                //     g_attitude_est.update(a, g, 0.0025f);
                // }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Optical Flow Task - 100Hz (10ms period)
 */
static void OptFlowTask(void* pvParameters)
{
    ESP_LOGI(TAG, "OptFlowTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_optflow != nullptr) {
            try {
                auto burst = g_optflow->readMotionBurst();
                // Check quality
                if (stampfly::OutlierDetector::isFlowValid(burst.squal)) {
                    state.updateOpticalFlow(burst.delta_x, burst.delta_y, burst.squal);

                    // Update ESKF with flow data (need height for velocity calculation)
                    if (g_eskf.isInitialized()) {
                        float height = state.getAltitude();
                        if (height > 0.05f) {  // Only update if height is valid
                            g_eskf.updateFlow(burst.delta_x * 0.001f, burst.delta_y * 0.001f, height);
                        }
                    }
                }
            } catch (const stampfly::PMW3901Exception& e) {
                // Sensor read error, continue
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Magnetometer Task - 100Hz (10ms period)
 */
static void MagTask(void* pvParameters)
{
    ESP_LOGI(TAG, "MagTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_mag.isInitialized()) {
            stampfly::MagData mag;
            if (g_mag.read(mag) == ESP_OK) {
                // Update state (no outlier filter for now - needs calibration first)
                state.updateMag(mag.x, mag.y, mag.z);

                // TODO: Re-enable after debugging
                // Update ESKF magnetometer
                // if (g_eskf.isInitialized()) {
                //     stampfly::math::Vector3 m(mag.x, mag.y, mag.z);
                //     g_eskf.updateMag(m);
                // }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Barometer Task - 50Hz (20ms period)
 */
static void BaroTask(void* pvParameters)
{
    ESP_LOGI(TAG, "BaroTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_baro.isInitialized()) {
            stampfly::BaroData baro;
            if (g_baro.read(baro) == ESP_OK) {
                // Use altitude from read() directly (already calculated)
                state.updateBaro(baro.pressure_pa, baro.temperature_c, baro.altitude_m);
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief ToF Task - 30Hz (33ms period)
 */
static void ToFTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ToFTask started, bottom_init=%d, front_init=%d",
             g_tof_bottom.isInitialized(), g_tof_front.isInitialized());

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    auto& state = stampfly::StampFlyState::getInstance();

    // Error counters for sensor disable on repeated failures
    const int MAX_ERRORS = 10;
    int bottom_errors = 0;
    int front_errors = 0;
    bool bottom_disabled = false;
    bool front_disabled = false;

    static int log_count = 0;

    while (true) {
        // Bottom ToF
        if (g_tof_bottom.isInitialized() && !bottom_disabled) {
            // Check if data is ready
            bool data_ready = false;
            g_tof_bottom.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                esp_err_t ret = g_tof_bottom.getDistance(distance_mm, status);
                if (ret == ESP_OK) {
                    bottom_errors = 0;  // Reset on success

                    // Only update if valid measurement (status 0-4)
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;
                        state.updateToF(stampfly::ToFPosition::BOTTOM, distance_m, status);
                    }

                    // Debug log every 30 readings (~1 second)
                    if (++log_count >= 30) {
                        ESP_LOGI(TAG, "ToF Bottom: %d mm, status=%d", distance_mm, status);
                        log_count = 0;
                    }

                    // Clear interrupt and start next measurement
                    g_tof_bottom.clearInterruptAndStartMeasurement();
                } else {
                    if (++bottom_errors >= MAX_ERRORS) {
                        ESP_LOGW(TAG, "Bottom ToF disabled: err=%s", esp_err_to_name(ret));
                        bottom_disabled = true;
                    }
                }
            }
        }

        // Front ToF
        if (g_tof_front.isInitialized() && !front_disabled) {
            bool data_ready = false;
            g_tof_front.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                if (g_tof_front.getDistance(distance_mm, status) == ESP_OK) {
                    front_errors = 0;  // Reset on success
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;
                        state.updateToF(stampfly::ToFPosition::FRONT, distance_m, status);
                    }
                    g_tof_front.clearInterruptAndStartMeasurement();
                } else {
                    if (++front_errors >= MAX_ERRORS) {
                        ESP_LOGW(TAG, "Front ToF disabled due to repeated errors");
                        front_disabled = true;
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Power Monitor Task - 10Hz (100ms period)
 */
static void PowerTask(void* pvParameters)
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

/**
 * @brief LED Task - 30Hz (32ms period)
 */
static void LEDTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LEDTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(32);  // ~30Hz

    auto& state = stampfly::StampFlyState::getInstance();
    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;

    while (true) {
        // Update LED pattern based on flight state
        stampfly::FlightState flight_state = state.getFlightState();

        if (flight_state != prev_flight_state) {
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
            prev_flight_state = flight_state;
        }

        // Update LED animation
        g_led.update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Button Task - 100Hz (10ms period)
 */
static void ButtonTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ButtonTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        if (g_button.isInitialized()) {
            g_button.tick();
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Communication Task - 50Hz (20ms period)
 */
static void CommTask(void* pvParameters)
{
    ESP_LOGI(TAG, "CommTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_comm.isInitialized()) {
            // Check connection timeout
            g_comm.tick();

            // Send telemetry if connected
            if (g_comm.isConnected()) {
                stampfly::TelemetryPacket telem = {};

                // Fill telemetry data
                telem.battery_mv = static_cast<uint16_t>(state.getVoltage() * 1000);
                telem.altitude_cm = static_cast<int16_t>(state.getAltitude() * 100);

                stampfly::StateVector3 vel = state.getVelocity();
                telem.velocity_x = static_cast<int16_t>(vel.x * 1000);
                telem.velocity_y = static_cast<int16_t>(vel.y * 1000);
                telem.velocity_z = static_cast<int16_t>(vel.z * 1000);

                stampfly::StateVector3 att = state.getAttitude();
                telem.roll_deg10 = static_cast<int16_t>(att.x * 180.0f / M_PI * 10);
                telem.pitch_deg10 = static_cast<int16_t>(att.y * 180.0f / M_PI * 10);
                telem.yaw_deg10 = static_cast<int16_t>(att.z * 180.0f / M_PI * 10);

                telem.state = static_cast<uint8_t>(state.getFlightState());

                // Set warning flags
                telem.flags = 0;
                if (g_power.isLowBattery()) {
                    telem.flags |= stampfly::TELEM_FLAG_LOW_BATTERY;
                }
                if (state.getErrorCode() != stampfly::ErrorCode::NONE) {
                    telem.flags |= stampfly::TELEM_FLAG_SENSOR_ERROR;
                }
                if (state.getFlightState() == stampfly::FlightState::CALIBRATING) {
                    telem.flags |= stampfly::TELEM_FLAG_CALIBRATING;
                }

                g_comm.sendTelemetry(telem);
            }

            // Update connection state
            if (!g_comm.isConnected() && state.getPairingState() == stampfly::PairingState::PAIRED) {
                state.setPairingState(stampfly::PairingState::PAIRED);  // Keep paired but disconnected
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief CLI Task - Low priority, processes USB serial input
 */
static void CLITask(void* pvParameters)
{
    ESP_LOGI(TAG, "CLITask started");

    // Print initial prompt
    g_cli.print("\r\n=== StampFly RTOS Skeleton ===\r\n");
    g_cli.print("Type 'help' for available commands\r\n");
    g_cli.print("> ");

    TickType_t last_teleplot = xTaskGetTickCount();
    const TickType_t teleplot_period = pdMS_TO_TICKS(50);  // 20Hz teleplot output

    while (true) {
        if (g_cli.isInitialized()) {
            g_cli.processInput();

            // Output teleplot data at fixed interval
            TickType_t now = xTaskGetTickCount();
            if (g_cli.isTeleplotEnabled() && (now - last_teleplot) >= teleplot_period) {
                g_cli.outputTeleplot();
                last_teleplot = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling
    }
}

// =============================================================================
// Button Event Handler
// =============================================================================

static void onButtonEvent(stampfly::Button::Event event)
{
    auto& state = stampfly::StampFlyState::getInstance();

    switch (event) {
        case stampfly::Button::Event::CLICK:
            ESP_LOGI(TAG, "Button: CLICK");
            // Toggle arm/disarm in IDLE state
            if (state.getFlightState() == stampfly::FlightState::IDLE) {
                if (state.requestArm()) {
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

static void onControlPacket(const stampfly::ControlPacket& packet)
{
    auto& state = stampfly::StampFlyState::getInstance();

    // Update control inputs
    state.updateControlInput(
        packet.throttle,
        packet.roll,
        packet.pitch,
        packet.yaw
    );

    // Handle arm/disarm from controller
    bool arm_requested = (packet.flags & stampfly::CTRL_FLAG_ARM) != 0;

    if (arm_requested && state.getFlightState() == stampfly::FlightState::IDLE) {
        if (state.requestArm()) {
            g_buzzer.armTone();
            ESP_LOGI(TAG, "Motors ARMED (from controller)");
        }
    } else if (!arm_requested && state.getFlightState() == stampfly::FlightState::ARMED) {
        if (state.requestDisarm()) {
            g_buzzer.disarmTone();
            ESP_LOGI(TAG, "Motors DISARMED (from controller)");
        }
    }
}

// =============================================================================
// Global I2C bus handle
// =============================================================================

static i2c_master_bus_handle_t g_i2c_bus = nullptr;

// =============================================================================
// Initialization Functions
// =============================================================================

static esp_err_t initI2C()
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = static_cast<gpio_num_t>(GPIO_I2C_SCL);
    bus_config.sda_io_num = static_cast<gpio_num_t>(GPIO_I2C_SDA);
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

static esp_err_t initSensors()
{
    ESP_LOGI(TAG, "Initializing sensors...");
    esp_err_t ret;

    // IMU (BMI270) - SPI with default StampFly config
    {
        auto cfg = stampfly::BMI270Wrapper::Config::defaultStampFly();
        ret = g_imu.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU initialized");
        }
    }

    // Magnetometer (BMM150) - I2C
    {
        stampfly::BMM150::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::BMM150_I2C_ADDR_DEFAULT;
        cfg.data_rate = stampfly::BMM150DataRate::ODR_10HZ;
        cfg.preset = stampfly::BMM150Preset::REGULAR;

        ret = g_mag.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Magnetometer initialized");
        }
    }

    // Barometer (BMP280) - I2C
    {
        stampfly::BMP280::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::BMP280_I2C_ADDR_DEFAULT;
        cfg.mode = stampfly::BMP280Mode::NORMAL;
        cfg.press_os = stampfly::BMP280Oversampling::X4;
        cfg.temp_os = stampfly::BMP280Oversampling::X2;
        cfg.standby = stampfly::BMP280Standby::MS_62_5;
        cfg.filter = stampfly::BMP280Filter::COEF_4;

        ret = g_baro.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Barometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Barometer initialized");
        }
    }

    // ToF sensors (VL53L3CX) - I2C with XSHUT control
    // Dual sensor initialization: bottom (altitude) and front (obstacle detection)
    // Note: Front ToF is optional (removable for battery adapter)
    {
        ret = stampfly::VL53L3CXWrapper::initDualSensors(
            g_tof_bottom,
            g_tof_front,
            g_i2c_bus,
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_BOTTOM),
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_FRONT)
        );
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ToF init failed: %s", esp_err_to_name(ret));
        } else {
            // Start ranging for initialized sensors
            if (g_tof_bottom.isInitialized()) {
                g_tof_bottom.startRanging();
                ESP_LOGI(TAG, "Bottom ToF initialized and ranging");
            }
            if (g_tof_front.isInitialized()) {
                g_tof_front.startRanging();
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(true);
                ESP_LOGI(TAG, "Front ToF initialized and ranging");
            } else {
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(false);
                ESP_LOGI(TAG, "Front ToF not available (optional sensor)");
            }
        }
    }

    // Optical Flow (PMW3901) - SPI (uses constructor with default config)
    {
        try {
            auto cfg = stampfly::PMW3901::Config::defaultStampFly();
            g_optflow = new stampfly::PMW3901(cfg);
            ESP_LOGI(TAG, "Optical Flow initialized");
        } catch (const stampfly::PMW3901Exception& e) {
            ESP_LOGW(TAG, "Optical Flow init failed: %s", e.what());
        }
    }

    // Power Monitor (INA3221) - I2C
    {
        stampfly::PowerMonitor::Config cfg;
        cfg.i2c_bus = g_i2c_bus;
        cfg.i2c_addr = stampfly::INA3221_I2C_ADDR_GND;
        cfg.battery_channel = 1;  // Battery is connected to CH1
        cfg.shunt_resistor_ohm = 0.1f;

        ret = g_power.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power Monitor init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Power Monitor initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initActuators()
{
    ESP_LOGI(TAG, "Initializing actuators...");
    esp_err_t ret;

    // Motor Driver
    {
        stampfly::MotorDriver::Config cfg;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FR] = GPIO_MOTOR_M1;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RR] = GPIO_MOTOR_M2;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RL] = GPIO_MOTOR_M3;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FL] = GPIO_MOTOR_M4;
        cfg.pwm_freq_hz = 150000;  // 150kHz
        cfg.pwm_resolution_bits = 8;

        ret = g_motor.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor Driver init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Motor Driver initialized");
    }

    // LED
    {
        stampfly::LED::Config cfg;
        cfg.gpio = GPIO_LED;
        cfg.num_leds = 1;

        ret = g_led.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LED init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "LED initialized");
        }
    }

    // Buzzer
    {
        stampfly::Buzzer::Config cfg;
        cfg.gpio = GPIO_BUZZER;
        cfg.ledc_channel = 4;
        cfg.ledc_timer = 1;

        ret = g_buzzer.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Buzzer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Buzzer initialized");
        }
    }

    // Button
    {
        stampfly::Button::Config cfg;
        cfg.gpio = GPIO_BUTTON;
        cfg.debounce_ms = 50;

        ret = g_button.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Button init failed: %s", esp_err_to_name(ret));
        } else {
            g_button.setCallback(onButtonEvent);
            ESP_LOGI(TAG, "Button initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initEstimators()
{
    ESP_LOGI(TAG, "Initializing estimators...");

    // Initialize IMU filters
    for (int i = 0; i < 3; i++) {
        g_accel_lpf[i].init(400.0f, 50.0f);   // 400Hz sampling, 50Hz cutoff
        g_gyro_lpf[i].init(400.0f, 100.0f);   // 400Hz sampling, 100Hz cutoff
    }

    // ESKF
    {
        auto cfg = stampfly::ESKF::Config::defaultConfig();
        esp_err_t ret = g_eskf.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ESKF init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "ESKF initialized");
        }
    }

    // Simple Attitude Estimator
    {
        stampfly::AttitudeEstimator::Config cfg;
        cfg.gyro_weight = 0.98f;
        cfg.mag_declination = 0.0f;  // Adjust for local declination
        esp_err_t ret = g_attitude_est.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AttitudeEstimator init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AttitudeEstimator initialized");
        }
    }

    // Simple Altitude Estimator
    {
        stampfly::AltitudeEstimator::Config cfg;
        cfg.process_noise_alt = 0.01f;
        cfg.process_noise_vel = 0.1f;
        cfg.measurement_noise_baro = 1.0f;
        cfg.measurement_noise_tof = 0.05f;
        esp_err_t ret = g_altitude_est.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AltitudeEstimator init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AltitudeEstimator initialized");
        }
    }

    return ESP_OK;
}

static esp_err_t initCommunication()
{
    ESP_LOGI(TAG, "Initializing communication...");

    // ESP-NOW Controller Communication
    {
        stampfly::ControllerComm::Config cfg;
        cfg.wifi_channel = 1;
        cfg.timeout_ms = 500;

        esp_err_t ret = g_comm.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ControllerComm init failed: %s", esp_err_to_name(ret));
            return ret;
        }

        g_comm.setControlCallback(onControlPacket);

        // Load pairing from NVS
        if (g_comm.loadPairingFromNVS() == ESP_OK && g_comm.isPaired()) {
            ESP_LOGI(TAG, "Loaded pairing from NVS");
            stampfly::StampFlyState::getInstance().setPairingState(stampfly::PairingState::PAIRED);
        }

        g_comm.start();
        ESP_LOGI(TAG, "ControllerComm initialized");
    }

    return ESP_OK;
}

static esp_err_t initCLI()
{
    ESP_LOGI(TAG, "Initializing CLI...");

    esp_err_t ret = g_cli.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CLI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_cli.registerDefaultCommands();
    ESP_LOGI(TAG, "CLI initialized");
    return ESP_OK;
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

    // Sensor tasks (Core 1)
    xTaskCreatePinnedToCore(IMUTask, "IMUTask", STACK_SIZE_IMU, nullptr,
                            PRIORITY_IMU_TASK, &g_imu_task_handle, 1);

    xTaskCreatePinnedToCore(MagTask, "MagTask", STACK_SIZE_MAG, nullptr,
                            PRIORITY_MAG_TASK, &g_mag_task_handle, 1);

    xTaskCreatePinnedToCore(BaroTask, "BaroTask", STACK_SIZE_BARO, nullptr,
                            PRIORITY_BARO_TASK, &g_baro_task_handle, 1);

    xTaskCreatePinnedToCore(ToFTask, "ToFTask", STACK_SIZE_TOF, nullptr,
                            PRIORITY_TOF_TASK, &g_tof_task_handle, 1);

    xTaskCreatePinnedToCore(OptFlowTask, "OptFlowTask", STACK_SIZE_OPTFLOW, nullptr,
                            PRIORITY_OPTFLOW_TASK, &g_optflow_task_handle, 1);

    // Communication task (Core 0)
    xTaskCreatePinnedToCore(CommTask, "CommTask", STACK_SIZE_COMM, nullptr,
                            PRIORITY_COMM_TASK, &g_comm_task_handle, 0);

    // CLI task (Core 0)
    xTaskCreatePinnedToCore(CLITask, "CLITask", STACK_SIZE_CLI, nullptr,
                            PRIORITY_CLI_TASK, &g_cli_task_handle, 0);

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
    initI2C();

    // Initialize actuators first (for buzzer feedback)
    ESP_LOGI(TAG, "Initializing actuators...");
    initActuators();

    // Play startup tone early to indicate boot progress
    ESP_LOGI(TAG, "Playing startup tone...");
    g_buzzer.startTone();
    g_led.showInit();

    // Initialize sensors (may fail partially, that's OK)
    ESP_LOGI(TAG, "Initializing sensors...");
    initSensors();

    // Initialize estimators
    ESP_LOGI(TAG, "Initializing estimators...");
    initEstimators();

    // Initialize communication (ESP-NOW)
    ESP_LOGI(TAG, "Initializing communication...");
    initCommunication();

    // Initialize CLI
    ESP_LOGI(TAG, "Initializing CLI...");
    initCLI();

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    startTasks();

    // Transition to IDLE state after initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    state.setFlightState(stampfly::FlightState::IDLE);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "StampFly initialized successfully!");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    // Main loop - just monitor system health
    while (true) {
        // Log heap usage periodically (for debugging)
        static uint32_t last_log_time = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_log_time > 30000) {  // Every 30 seconds
            ESP_LOGI(TAG, "Free heap: %lu bytes, Min free: %lu bytes",
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
            last_log_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
