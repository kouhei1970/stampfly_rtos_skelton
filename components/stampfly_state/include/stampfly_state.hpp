/**
 * @file stampfly_state.hpp
 * @brief StampFly State Manager
 *
 * Thread-safe, sensor data management, state transitions
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "filter.hpp"

namespace stampfly {

enum class FlightState {
    INIT,
    CALIBRATING,
    IDLE,
    ARMED,
    FLYING,
    LANDING,
    ERROR
};

enum class PairingState {
    NOT_PAIRED,
    PAIRING,
    PAIRED
};

enum class ErrorCode {
    NONE,
    SENSOR_IMU,
    SENSOR_MAG,
    SENSOR_BARO,
    SENSOR_TOF,
    SENSOR_FLOW,
    SENSOR_POWER,
    LOW_BATTERY,
    COMM_LOST,
    CALIBRATION_FAILED
};

/**
 * @brief Centralized state manager (Singleton)
 */
class StampFlyState {
public:
    static StampFlyState& getInstance();

    // Delete copy/move
    StampFlyState(const StampFlyState&) = delete;
    StampFlyState& operator=(const StampFlyState&) = delete;

    esp_err_t init();

    // State access
    FlightState getFlightState() const;
    PairingState getPairingState() const;
    ErrorCode getErrorCode() const;

    // State transitions
    bool requestArm();
    bool requestDisarm();
    void setFlightState(FlightState state);
    void setPairingState(PairingState state);
    void setError(ErrorCode code);
    void clearError();

    // Sensor data access (thread-safe)
    void getIMUData(Vec3& accel, Vec3& gyro) const;
    void getMagData(Vec3& mag) const;
    void getBaroData(float& altitude, float& pressure) const;
    void getToFData(float& bottom, float& front) const;
    void getFlowData(float& vx, float& vy) const;
    void getPowerData(float& voltage, float& current) const;

    // Sensor data update (thread-safe)
    void updateIMU(const Vec3& accel, const Vec3& gyro);
    void updateMag(const Vec3& mag);
    void updateBaro(float altitude, float pressure);
    void updateToF(float bottom, float front);
    void updateFlow(float vx, float vy);
    void updatePower(float voltage, float current);

    // Attitude data
    void getAttitude(float& roll, float& pitch, float& yaw) const;
    void updateAttitude(float roll, float pitch, float yaw);

    // Control input
    void getControlInput(float& throttle, float& roll, float& pitch, float& yaw) const;
    void updateControlInput(float throttle, float roll, float pitch, float yaw);

private:
    StampFlyState() = default;

    mutable SemaphoreHandle_t mutex_ = nullptr;

    FlightState flight_state_ = FlightState::INIT;
    PairingState pairing_state_ = PairingState::NOT_PAIRED;
    ErrorCode error_code_ = ErrorCode::NONE;

    // Sensor data
    Vec3 accel_ = {};
    Vec3 gyro_ = {};
    Vec3 mag_ = {};
    float baro_altitude_ = 0;
    float baro_pressure_ = 101325.0f;
    float tof_bottom_ = 0;
    float tof_front_ = 0;
    float flow_vx_ = 0;
    float flow_vy_ = 0;
    float power_voltage_ = 0;
    float power_current_ = 0;

    // Attitude
    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;

    // Control input
    float ctrl_throttle_ = 0;
    float ctrl_roll_ = 0;
    float ctrl_pitch_ = 0;
    float ctrl_yaw_ = 0;
};

}  // namespace stampfly
