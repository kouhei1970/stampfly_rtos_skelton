/**
 * @file init.cpp
 * @brief 初期化関数の実装
 */

#include "init.hpp"
#include "config.hpp"
#include "globals.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators
#include "motor_driver.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// Estimation
#include "sensor_fusion.hpp"
#include "system_manager.hpp"
#include "filter.hpp"

// Communication
#include "controller_comm.hpp"
#include "cli.hpp"
#include "logger.hpp"
#include "telemetry.hpp"

// State
#include "stampfly_state.hpp"

static const char* TAG = "init";

using namespace config;
using namespace globals;

// =============================================================================
// File-local variables
// =============================================================================

static i2c_master_bus_handle_t s_i2c_bus = nullptr;

// =============================================================================
// Callbacks (defined in main.cpp, declared here for reference)
// =============================================================================

extern void onButtonEvent(stampfly::Button::Event event);
extern void onControlPacket(const stampfly::ControlPacket& packet);
extern void onBinlogStart();

// =============================================================================
// Initialization Functions
// =============================================================================

namespace init {

i2c_master_bus_handle_t getI2CBus()
{
    return s_i2c_bus;
}

esp_err_t i2c()
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = static_cast<gpio_num_t>(GPIO_I2C_SCL);
    bus_config.sda_io_num = static_cast<gpio_num_t>(GPIO_I2C_SDA);
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

esp_err_t sensors()
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
        cfg.i2c_bus = s_i2c_bus;
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
        cfg.i2c_bus = s_i2c_bus;
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
            s_i2c_bus,
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
        cfg.i2c_bus = s_i2c_bus;
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

esp_err_t actuators()
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
        g_motor_ptr = &g_motor;  // Set pointer for CLI access
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
            g_led_ptr = &g_led;  // Set pointer for CLI access
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
            g_buzzer_ptr = &g_buzzer;  // Set pointer for CLI access
            g_buzzer.loadFromNVS();    // Load mute setting from NVS
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

esp_err_t estimators()
{
    ESP_LOGI(TAG, "Initializing estimators...");

    // Initialize IMU filters
    for (int i = 0; i < 3; i++) {
        g_accel_lpf[i].init(400.0f, 50.0f);   // 400Hz sampling, 50Hz cutoff
        g_gyro_lpf[i].init(400.0f, 100.0f);   // 400Hz sampling, 100Hz cutoff
    }

    // Initialize magnetometer calibrator and load from NVS
    g_mag_calibrator = &g_mag_cal;  // Set global pointer for CLI access
    if (g_mag_cal.loadFromNVS() == ESP_OK) {
        ESP_LOGI(TAG, "Magnetometer calibration loaded from NVS");
    } else {
        ESP_LOGW(TAG, "No magnetometer calibration found in NVS");
    }

    // センサーフュージョン (ESKFをラップ)
    {
        auto& state = stampfly::StampFlyState::getInstance();
        // デフォルト設定で初期化（全センサー有効）
        bool ok = g_fusion.init();
        if (!ok) {
            ESP_LOGW(TAG, "Sensor fusion init failed");
            state.setESKFInitialized(false);
        } else {
            ESP_LOGI(TAG, "Sensor fusion initialized (predict at 400Hz)");
            state.setESKFInitialized(true);

            // ジャイロバイアスキャリブレーション（静止状態で実行）
            if (g_imu.isInitialized()) {
                ESP_LOGI(TAG, "Calibrating gyro bias (keep device still)...");
                constexpr int CALIB_SAMPLES = 200;  // 200サンプル @ 400Hz = 0.5秒
                float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
                int valid_samples = 0;

                for (int i = 0; i < CALIB_SAMPLES; i++) {
                    stampfly::AccelData accel;
                    stampfly::GyroData gyro;
                    if (g_imu.readSensorData(accel, gyro) == ESP_OK) {
                        // BMI270座標系 → 機体座標系(NED) 変換
                        float gyro_body_x = gyro.y;     // Roll rate
                        float gyro_body_y = gyro.x;     // Pitch rate
                        float gyro_body_z = -gyro.z;    // Yaw rate

                        gyro_sum_x += gyro_body_x;
                        gyro_sum_y += gyro_body_y;
                        gyro_sum_z += gyro_body_z;
                        valid_samples++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2));  // ~500Hz
                }

                if (valid_samples > 0) {
                    stampfly::math::Vector3 gyro_bias(
                        gyro_sum_x / valid_samples,
                        gyro_sum_y / valid_samples,
                        gyro_sum_z / valid_samples
                    );
                    g_fusion.setGyroBias(gyro_bias);
                    g_initial_gyro_bias = gyro_bias;  // binlog reset後に復元するため保存
                    ESP_LOGI(TAG, "Gyro bias set: [%.5f, %.5f, %.5f] rad/s",
                             gyro_bias.x, gyro_bias.y, gyro_bias.z);
                }
            }

            // 地磁気リファレンス取得（Magキャリブレーション済みの場合）
            if (g_mag_cal.isCalibrated() && g_mag.isInitialized()) {
                ESP_LOGI(TAG, "Acquiring mag reference (keep device still)...");
                constexpr int MAG_CALIB_SAMPLES = 100;  // 100サンプル @ 100Hz = 1秒
                stampfly::math::Vector3 mag_sum = stampfly::math::Vector3::zero();
                int mag_valid_samples = 0;

                for (int i = 0; i < MAG_CALIB_SAMPLES; i++) {
                    stampfly::MagData raw_mag;
                    if (g_mag.read(raw_mag) == ESP_OK) {
                        // センサ座標系 → 機体座標系変換
                        float mag_body_x = raw_mag.y;
                        float mag_body_y = raw_mag.x;
                        float mag_body_z = -raw_mag.z;
                        // キャリブレーション適用
                        float cal_x, cal_y, cal_z;
                        g_mag_cal.applyCalibration(mag_body_x, mag_body_y, mag_body_z, cal_x, cal_y, cal_z);
                        mag_sum.x += cal_x;
                        mag_sum.y += cal_y;
                        mag_sum.z += cal_z;
                        mag_valid_samples++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));  // ~100Hz
                }

                if (mag_valid_samples > 0) {
                    stampfly::math::Vector3 mag_ref(
                        mag_sum.x / mag_valid_samples,
                        mag_sum.y / mag_valid_samples,
                        mag_sum.z / mag_valid_samples
                    );
                    g_fusion.setMagReference(mag_ref);
                    g_mag_ref_set = true;
                    ESP_LOGI(TAG, "Mag reference set: [%.1f, %.1f, %.1f] uT",
                             mag_ref.x, mag_ref.y, mag_ref.z);
                }
            }
        }
    }

    // Simple Attitude Estimator (backup/complementary)
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

esp_err_t communication()
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

        // Set global pointer for CLI access
        g_comm_ptr = &g_comm;

        ESP_LOGI(TAG, "ControllerComm initialized");
    }

    return ESP_OK;
}

esp_err_t cli()
{
    ESP_LOGI(TAG, "Initializing CLI...");

    esp_err_t ret = g_cli.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CLI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_cli.registerDefaultCommands();

    // binlog開始時にmag_refを設定するコールバックを登録 (後方互換性のため維持)
    g_cli.setBinlogStartCallback(onBinlogStart);

    ESP_LOGI(TAG, "CLI initialized");
    return ESP_OK;
}

esp_err_t logger()
{
    ESP_LOGI(TAG, "Initializing Logger...");

    // 400Hz logging (matches ESKF rate)
    esp_err_t ret = g_logger.init(400);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Logger init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set start callback (ESKF reset + mag_ref setting)
    g_logger.setStartCallback(onBinlogStart);

    // Set global pointer for CLI access
    g_logger_ptr = &g_logger;

    ESP_LOGI(TAG, "Logger initialized at 400Hz");
    return ESP_OK;
}

esp_err_t telemetry()
{
    ESP_LOGI(TAG, "Initializing Telemetry...");

    auto& telem = stampfly::Telemetry::getInstance();
    stampfly::Telemetry::Config cfg;
    cfg.port = 80;
    cfg.rate_hz = 50;

    esp_err_t ret = telem.init(cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Telemetry init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Telemetry initialized - Connect to WiFi 'StampFly', open http://192.168.4.1");
    return ESP_OK;
}

} // namespace init
