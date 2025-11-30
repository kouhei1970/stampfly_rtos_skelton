/**
 * @file stampfly_state.cpp
 * @brief StampFly State Manager Implementation
 */

#include "stampfly_state.hpp"
#include "esp_log.h"

static const char* TAG = "StampFlyState";

namespace stampfly {

StampFlyState& StampFlyState::getInstance()
{
    static StampFlyState instance;
    return instance;
}

esp_err_t StampFlyState::init()
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "StampFlyState initialized");
    return ESP_OK;
}

FlightState StampFlyState::getFlightState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    FlightState state = flight_state_;
    xSemaphoreGive(mutex_);
    return state;
}

PairingState StampFlyState::getPairingState() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    PairingState state = pairing_state_;
    xSemaphoreGive(mutex_);
    return state;
}

ErrorCode StampFlyState::getErrorCode() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ErrorCode code = error_code_;
    xSemaphoreGive(mutex_);
    return code;
}

bool StampFlyState::requestArm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool success = false;
    if (flight_state_ == FlightState::IDLE && error_code_ == ErrorCode::NONE) {
        flight_state_ = FlightState::ARMED;
        success = true;
        ESP_LOGI(TAG, "Armed");
    }
    xSemaphoreGive(mutex_);
    return success;
}

bool StampFlyState::requestDisarm()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool success = false;
    if (flight_state_ == FlightState::ARMED || flight_state_ == FlightState::FLYING) {
        flight_state_ = FlightState::IDLE;
        success = true;
        ESP_LOGI(TAG, "Disarmed");
    }
    xSemaphoreGive(mutex_);
    return success;
}

void StampFlyState::setFlightState(FlightState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    flight_state_ = state;
    xSemaphoreGive(mutex_);
}

void StampFlyState::setPairingState(PairingState state)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    pairing_state_ = state;
    xSemaphoreGive(mutex_);
}

void StampFlyState::setError(ErrorCode code)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = code;
    if (code != ErrorCode::NONE) {
        flight_state_ = FlightState::ERROR;
    }
    xSemaphoreGive(mutex_);
}

void StampFlyState::clearError()
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_code_ = ErrorCode::NONE;
    xSemaphoreGive(mutex_);
}

// Sensor data getters
void StampFlyState::getIMUData(Vec3& accel, Vec3& gyro) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    accel.x = accel_.x;
    accel.y = accel_.y;
    accel.z = accel_.z;
    gyro.x = gyro_.x;
    gyro.y = gyro_.y;
    gyro.z = gyro_.z;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getMagData(Vec3& mag) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    mag.x = mag_.x;
    mag.y = mag_.y;
    mag.z = mag_.z;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getBaroData(float& altitude, float& pressure) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    altitude = baro_altitude_;
    pressure = baro_pressure_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getToFData(float& bottom, float& front) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bottom = tof_bottom_;
    front = tof_front_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getFlowData(float& vx, float& vy) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    vx = static_cast<float>(flow_delta_x_);
    vy = static_cast<float>(flow_delta_y_);
    xSemaphoreGive(mutex_);
}

void StampFlyState::getFlowRawData(int16_t& dx, int16_t& dy, uint8_t& squal) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    dx = flow_delta_x_;
    dy = flow_delta_y_;
    squal = flow_squal_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getPowerData(float& voltage, float& current) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    voltage = power_voltage_;
    current = power_current_;
    xSemaphoreGive(mutex_);
}

bool StampFlyState::isFrontToFAvailable() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool available = front_tof_available_;
    xSemaphoreGive(mutex_);
    return available;
}

void StampFlyState::setFrontToFAvailable(bool available)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    front_tof_available_ = available;
    xSemaphoreGive(mutex_);
}

// Simple getters
float StampFlyState::getAltitude() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    float alt = baro_altitude_;
    xSemaphoreGive(mutex_);
    return alt;
}

float StampFlyState::getVoltage() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    float v = power_voltage_;
    xSemaphoreGive(mutex_);
    return v;
}

StateVector3 StampFlyState::getVelocity() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    StateVector3 vel = velocity_;
    xSemaphoreGive(mutex_);
    return vel;
}

StateVector3 StampFlyState::getAttitude() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    StateVector3 att(roll_, pitch_, yaw_);
    xSemaphoreGive(mutex_);
    return att;
}

StateVector3 StampFlyState::getPosition() const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    StateVector3 pos = position_;
    xSemaphoreGive(mutex_);
    return pos;
}

// Sensor data setters
void StampFlyState::updateIMU(const StateVector3& accel, const StateVector3& gyro)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    accel_ = accel;
    gyro_ = gyro;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateMag(float x, float y, float z)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    mag_.x = x;
    mag_.y = y;
    mag_.z = z;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateBaro(float pressure, float temperature, float altitude)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    baro_pressure_ = pressure;
    baro_temperature_ = temperature;
    baro_altitude_ = altitude;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateToF(ToFPosition position, float distance, uint8_t status)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (position == ToFPosition::BOTTOM) {
        tof_bottom_ = distance;
        tof_bottom_status_ = status;
    } else {
        tof_front_ = distance;
        tof_front_status_ = status;
    }
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateOpticalFlow(int16_t delta_x, int16_t delta_y, uint8_t squal)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    flow_delta_x_ = delta_x;
    flow_delta_y_ = delta_y;
    flow_squal_ = squal;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updatePower(float voltage, float current)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    power_voltage_ = voltage;
    power_current_ = current;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getAttitudeEuler(float& roll, float& pitch, float& yaw) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateAttitude(float roll, float pitch, float yaw)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateEstimatedPosition(float x, float y, float z)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    position_.x = x;
    position_.y = y;
    position_.z = z;
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateEstimatedVelocity(float vx, float vy, float vz)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    velocity_.x = vx;
    velocity_.y = vy;
    velocity_.z = vz;
    xSemaphoreGive(mutex_);
}

void StampFlyState::getControlInput(float& throttle, float& roll, float& pitch, float& yaw) const
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    // Convert from 0-1000 to normalized values
    throttle = ctrl_throttle_ / 1000.0f;
    roll = (ctrl_roll_ - 500) / 500.0f;      // -1 to +1
    pitch = (ctrl_pitch_ - 500) / 500.0f;    // -1 to +1
    yaw = (ctrl_yaw_ - 500) / 500.0f;        // -1 to +1
    xSemaphoreGive(mutex_);
}

void StampFlyState::updateControlInput(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw)
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ctrl_throttle_ = throttle;
    ctrl_roll_ = roll;
    ctrl_pitch_ = pitch;
    ctrl_yaw_ = yaw;
    xSemaphoreGive(mutex_);
}

}  // namespace stampfly
