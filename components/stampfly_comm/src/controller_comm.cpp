/**
 * @file controller_comm.cpp
 * @brief Controller Communication Implementation (Placeholder)
 */
#include "controller_comm.hpp"
#include "esp_log.h"

static const char* TAG = "comm";

namespace stampfly {

ControllerComm& ControllerComm::getInstance() {
    static ControllerComm instance;
    return instance;
}

esp_err_t ControllerComm::init() {
    ESP_LOGI(TAG, "ControllerComm init (placeholder)");
    // TODO: Implement ESP-NOW communication
    return ESP_OK;
}

void ControllerComm::setControlCallback(std::function<void(const ControlPacket&)> cb) {
    // TODO: Implement
}

esp_err_t ControllerComm::sendTelemetry(const TelemetryPacket& packet) {
    return ESP_OK;
}

bool ControllerComm::isConnected() const {
    return false;
}

esp_err_t ControllerComm::clearPairingFromNVS() {
    ESP_LOGI(TAG, "Clearing pairing info (placeholder)");
    return ESP_OK;
}

}  // namespace stampfly
