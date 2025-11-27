/**
 * @file stampfly_state.hpp
 * @brief StampFly State Management Class
 *
 * Central state management for the StampFly drone system.
 * Provides thread-safe access to all sensor data, estimated state,
 * control inputs, and system status.
 */

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

namespace stampfly {

// ============================================================================
// Data Types
// ============================================================================

struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3() = default;
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Vector2 {
    float x = 0.0f;
    float y = 0.0f;

    Vector2() = default;
    Vector2(float x_, float y_) : x(x_), y(y_) {}
};

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};

struct ControlInput {
    uint16_t throttle = 0;      // 0-1000
    int16_t roll = 0;           // -500 to +500
    int16_t pitch = 0;          // -500 to +500
    int16_t yaw = 0;            // -500 to +500
    bool arm = false;
    bool flip = false;
    bool mode = false;          // Flight mode switch
    bool alt_mode = false;      // Altitude hold mode
    int64_t timestamp_us = 0;
};

// Calibration data structure
struct CalibrationData {
    Vector3 accel_bias;
    Vector3 gyro_bias;
    Vector3 mag_hard_iron;
    float mag_soft_iron[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float baro_offset = 0.0f;
    bool valid = false;
};

class StampFlyState {
public:
    // Singleton access
    static StampFlyState& getInstance();

    // Flight states
    enum class FlightState {
        INIT,           // Initializing
        CALIBRATING,    // Calibration in progress
        IDLE,           // Standby (Disarmed)
        ARMED,          // Armed
        FLYING,         // In flight
        LANDING,        // Landing
        ERROR           // Error state
    };

    // Error codes
    enum class ErrorCode {
        NONE = 0,
        IMU_FAILURE,
        MAG_FAILURE,
        BARO_FAILURE,
        TOF_FAILURE,
        FLOW_FAILURE,
        COMM_TIMEOUT,
        LOW_BATTERY,
        ESTIMATOR_DIVERGED
    };

    // Warning flags
    enum WarningFlags : uint32_t {
        WARNING_NONE = 0,
        WARNING_LOW_BATTERY = (1 << 0),
        WARNING_COMM_WEAK = (1 << 1),
        WARNING_SENSOR_DEGRADED = (1 << 2)
    };

    // Pairing states
    enum class PairingState {
        IDLE,       // Normal operation (not connected)
        WAITING,    // Waiting for pairing
        PAIRED      // Paired
    };

    // State getters (thread-safe)
    FlightState getFlightState() const;
    PairingState getPairingState() const;
    ErrorCode getErrorCode() const;
    bool hasWarning(WarningFlags flag) const;
    bool hasError() const;

    // Sensor data getters (thread-safe)
    void getIMUData(Vector3& accel, Vector3& gyro) const;
    void getMagData(Vector3& mag) const;
    void getBaroData(float& altitude, float& pressure) const;
    void getToFData(float& front, float& bottom) const;
    void getFlowData(Vector2& velocity) const;

    // Estimated state getters
    void getPosition(Vector3& pos) const;
    void getVelocity(Vector3& vel) const;
    void getAttitude(Quaternion& quat) const;
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;

    // Controller input
    void getControlInput(ControlInput& input) const;
    uint16_t getThrottleInput() const;

    // Sensor data setters (called from sensor tasks)
    void updateIMU(const Vector3& accel, const Vector3& gyro);
    void updateMag(const Vector3& mag);
    void updateBaro(float altitude, float pressure);
    void updateToF(float front, float bottom);
    void updateFlow(const Vector2& velocity);

    // Battery state
    void updateBattery(float voltage, float current);
    float getBatteryVoltage() const;
    float getBatteryCurrent() const;
    float getBatteryPercent() const;

    // Warning flag operations
    void setWarning(WarningFlags flag);
    void clearWarning(WarningFlags flag);

    // State transitions
    void setFlightState(FlightState state);
    void setPairingState(PairingState state);
    bool requestArm();
    bool requestDisarm();
    void setError(ErrorCode code);
    void clearError();

    // NVS persistence
    esp_err_t saveToNVS();
    esp_err_t loadFromNVS();

    // Prevent copying
    StampFlyState(const StampFlyState&) = delete;
    StampFlyState& operator=(const StampFlyState&) = delete;

private:
    StampFlyState();
    ~StampFlyState();

    mutable SemaphoreHandle_t mutex_;

    // State data
    FlightState flight_state_ = FlightState::INIT;
    PairingState pairing_state_ = PairingState::IDLE;
    ErrorCode error_code_ = ErrorCode::NONE;
    uint32_t warning_flags_ = WARNING_NONE;

    // Sensor data
    struct SensorData {
        Vector3 accel;
        Vector3 gyro;
        Vector3 mag;
        float baro_altitude = 0.0f;
        float baro_pressure = 0.0f;
        float tof_front = 0.0f;
        float tof_bottom = 0.0f;
        Vector2 flow_velocity;
    } sensor_data_;

    // Battery data
    struct BatteryData {
        float voltage = 0.0f;
        float current = 0.0f;
    } battery_data_;

    // Estimated state
    struct EstimatedState {
        Vector3 position;
        Vector3 velocity;
        Quaternion attitude;
    } estimated_state_;

    // Controller input
    ControlInput control_input_;
};

// ============================================================================
// System Manager - Central initialization and management
// ============================================================================

class SystemManager {
public:
    // Event bits for system state
    static constexpr uint32_t EVENT_IMU_READY      = (1 << 0);
    static constexpr uint32_t EVENT_MAG_READY      = (1 << 1);
    static constexpr uint32_t EVENT_BARO_READY     = (1 << 2);
    static constexpr uint32_t EVENT_TOF_READY      = (1 << 3);
    static constexpr uint32_t EVENT_FLOW_READY     = (1 << 4);
    static constexpr uint32_t EVENT_POWER_READY    = (1 << 5);
    static constexpr uint32_t EVENT_COMM_READY     = (1 << 6);
    static constexpr uint32_t EVENT_CALIBRATED     = (1 << 7);
    static constexpr uint32_t EVENT_ALL_READY      = 0xFF;

    struct Config {
        // I2C configuration
        int i2c_sda;
        int i2c_scl;
        int i2c_freq;

        // SPI configuration (IMU/Flow)
        int spi_mosi;
        int spi_miso;
        int spi_sck;
        int imu_cs;
        int flow_cs;

        // Sensor specific
        int tof_bottom_xshut;
        int tof_front_xshut;

        // Peripherals
        int led_pin;
        int buzzer_pin;
        int button_pin;

        // Motors
        int motor_pins[4];

        // Timing
        uint32_t imu_rate_hz;
        uint32_t control_rate_hz;
        uint32_t sensor_rate_hz;

        Config() :
            i2c_sda(3), i2c_scl(4), i2c_freq(400000),
            spi_mosi(14), spi_miso(43), spi_sck(44), imu_cs(46), flow_cs(12),
            tof_bottom_xshut(7), tof_front_xshut(9),
            led_pin(39), buzzer_pin(40), button_pin(0),
            motor_pins{42, 41, 10, 5},
            imu_rate_hz(400), control_rate_hz(400), sensor_rate_hz(100)
        {}
    };

    static SystemManager& getInstance();

    /**
     * @brief Initialize the entire system
     * @param config System configuration
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config = Config{});

    /**
     * @brief Start all system tasks
     */
    esp_err_t start();

    /**
     * @brief Stop all system tasks
     */
    esp_err_t stop();

    /**
     * @brief Wait for specific events
     * @param events Event bits to wait for
     * @param timeout_ms Timeout in milliseconds
     * @return true if all events occurred
     */
    bool waitForEvents(uint32_t events, uint32_t timeout_ms);

    /**
     * @brief Set event bit
     */
    void setEvent(uint32_t event);

    /**
     * @brief Clear event bit
     */
    void clearEvent(uint32_t event);

    /**
     * @brief Get current event state
     */
    uint32_t getEvents() const;

    /**
     * @brief Check if system is fully initialized
     */
    bool isReady() const;

    /**
     * @brief Get calibration data
     */
    CalibrationData& getCalibration() { return calibration_; }

    /**
     * @brief Run sensor calibration
     * @return ESP_OK on success
     */
    esp_err_t runCalibration();

    /**
     * @brief Get system uptime in milliseconds
     */
    uint32_t getUptimeMs() const;

    // Prevent copying
    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

private:
    SystemManager() = default;
    ~SystemManager();

    Config config_;
    bool initialized_ = false;
    bool running_ = false;

    EventGroupHandle_t event_group_ = nullptr;
    CalibrationData calibration_;

    int64_t start_time_us_ = 0;
};

}  // namespace stampfly
