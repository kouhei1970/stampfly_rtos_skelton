/**
 * @file stampfly_state.hpp
 * @brief StampFly State Management Class
 */

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

namespace stampfly {

// Forward declarations
struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Vector2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct ControlInput {
    uint16_t throttle = 0;
    uint16_t roll = 1500;
    uint16_t pitch = 1500;
    uint16_t yaw = 1500;
    bool arm = false;
    bool flip = false;
    bool mode = false;
    bool alt_mode = false;
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

}  // namespace stampfly
