/**
 * @file controller_comm.hpp
 * @brief Controller Communication via ESP-NOW (Placeholder)
 */
#pragma once
#include <cstdint>
#include <functional>
#include "esp_err.h"

namespace stampfly {

class ControllerComm {
public:
    struct ControlPacket {
        uint8_t drone_mac[3];
        uint16_t throttle;
        uint16_t roll;
        uint16_t pitch;
        uint16_t yaw;
        uint8_t flags;
        uint8_t reserved;
        uint8_t checksum;
    } __attribute__((packed));

    struct TelemetryPacket {
        uint8_t packet_type;
        uint8_t sequence;
        uint16_t battery_mv;
        int16_t altitude_cm;
        int16_t velocity_x;
        int16_t velocity_y;
        int16_t velocity_z;
        int16_t roll_deg10;
        int16_t pitch_deg10;
        int16_t yaw_deg10;
        uint8_t state;
        uint8_t error;
        uint8_t checksum;
    } __attribute__((packed));

    struct PairingPacket {
        uint8_t channel;
        uint8_t drone_mac[6];
        uint8_t magic[4];
        uint8_t reserved[3];
    } __attribute__((packed));

    static ControllerComm& getInstance();

    esp_err_t init();
    void setControlCallback(std::function<void(const ControlPacket&)> cb);
    esp_err_t sendTelemetry(const TelemetryPacket& packet);
    bool isConnected() const;
    esp_err_t clearPairingFromNVS();

private:
    ControllerComm() = default;
};

}  // namespace stampfly
