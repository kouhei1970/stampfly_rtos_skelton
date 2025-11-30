/**
 * @file replay.cpp
 * @brief ESKF Replay Tool - Run ESKF on recorded sensor data
 *
 * Usage:
 *   ./eskf_replay <input.bin> <output.csv> [--params config.json]
 *
 * Reads binary log file from StampFly, runs ESKF on each packet,
 * and outputs state estimates to CSV.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include "eskf.hpp"

using namespace stampfly;
using namespace stampfly::math;

// Binary log packet structure (must match cli.hpp)
#pragma pack(push, 1)
struct BinaryLogPacket {
    uint8_t header[2];      // 0xAA, 0x55
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float pressure;
    float baro_alt;
    float tof_bottom;
    float tof_front;
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    uint8_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacket) == 64, "Packet size mismatch");

bool verify_checksum(const BinaryLogPacket& pkt)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 63; i++) {
        checksum ^= data[i];
    }
    return checksum == pkt.checksum;
}

std::vector<BinaryLogPacket> load_log_file(const char* filename)
{
    std::vector<BinaryLogPacket> packets;

    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Error: Cannot open file %s\n", filename);
        return packets;
    }

    BinaryLogPacket pkt;
    while (fread(&pkt, sizeof(pkt), 1, f) == 1) {
        if (pkt.header[0] == 0xAA && pkt.header[1] == 0x55) {
            if (verify_checksum(pkt)) {
                packets.push_back(pkt);
            } else {
                fprintf(stderr, "Warning: Checksum error at packet %zu\n", packets.size());
            }
        }
    }

    fclose(f);
    printf("Loaded %zu packets from %s\n", packets.size(), filename);

    return packets;
}

void print_usage(const char* prog)
{
    printf("Usage: %s <input.bin> <output.csv> [options]\n", prog);
    printf("\nOptions:\n");
    printf("  --baro-rate N    Baro update rate divisor (default: 2 = 50Hz)\n");
    printf("  --tof-rate N     ToF update rate divisor (default: 3 = 33Hz)\n");
    printf("  --mag-rate N     Mag update rate divisor (default: 10 = 10Hz)\n");
    printf("  --flow-rate N    Flow update rate divisor (default: 5 = 20Hz)\n");
    printf("  --flow-squal N   Min flow surface quality (default: 30)\n");
    printf("  --verbose        Print progress\n");
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char* input_file = argv[1];
    const char* output_file = argv[2];

    // Default update rates (as divisor of 100Hz base rate)
    int baro_rate = 2;    // 50Hz
    int tof_rate = 3;     // 33Hz
    int mag_rate = 10;    // 10Hz
    int flow_rate = 5;    // 20Hz
    int flow_squal_min = 30;
    bool verbose = false;

    // Parse options
    for (int i = 3; i < argc; i++) {
        if (strcmp(argv[i], "--baro-rate") == 0 && i + 1 < argc) {
            baro_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--tof-rate") == 0 && i + 1 < argc) {
            tof_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--mag-rate") == 0 && i + 1 < argc) {
            mag_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--flow-rate") == 0 && i + 1 < argc) {
            flow_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--flow-squal") == 0 && i + 1 < argc) {
            flow_squal_min = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        }
    }

    printf("=== ESKF Replay Tool ===\n");
    printf("Input: %s\n", input_file);
    printf("Output: %s\n", output_file);
    printf("Update rates: baro=%dHz, tof=%dHz, mag=%dHz, flow=%dHz\n",
           100 / baro_rate, 100 / tof_rate, 100 / mag_rate, 100 / flow_rate);

    // Load log file
    auto packets = load_log_file(input_file);
    if (packets.empty()) {
        fprintf(stderr, "No valid packets found\n");
        return 1;
    }

    // Initialize ESKF
    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    eskf.init(config);

    // Open output CSV
    FILE* out = fopen(output_file, "w");
    if (!out) {
        fprintf(stderr, "Error: Cannot create output file %s\n", output_file);
        return 1;
    }

    // Write CSV header
    fprintf(out, "timestamp_ms,dt_ms,");
    fprintf(out, "pos_x,pos_y,pos_z,");
    fprintf(out, "vel_x,vel_y,vel_z,");
    fprintf(out, "roll_deg,pitch_deg,yaw_deg,");
    fprintf(out, "gyro_bias_x,gyro_bias_y,gyro_bias_z,");
    fprintf(out, "accel_bias_x,accel_bias_y,accel_bias_z,");
    fprintf(out, "raw_accel_x,raw_accel_y,raw_accel_z,");
    fprintf(out, "raw_gyro_x,raw_gyro_y,raw_gyro_z,");
    fprintf(out, "raw_baro_alt,raw_tof,raw_flow_squal\n");

    // Process packets
    uint32_t last_timestamp = packets[0].timestamp_ms;
    int packet_count = 0;

    for (size_t i = 0; i < packets.size(); i++) {
        const auto& pkt = packets[i];

        // Calculate dt
        float dt = (pkt.timestamp_ms - last_timestamp) / 1000.0f;
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.01f;  // Default to 10ms if invalid
        }
        last_timestamp = pkt.timestamp_ms;

        // IMU data
        Vector3 accel(pkt.accel_x, pkt.accel_y, pkt.accel_z);
        Vector3 gyro(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);

        // ESKF predict step (every packet = 100Hz)
        eskf.predict(accel, gyro, dt);

        // Measurement updates at lower rates
        if (i % baro_rate == 0) {
            eskf.updateBaro(pkt.baro_alt);
        }

        if (i % tof_rate == 0 && pkt.tof_bottom > 0.01f && pkt.tof_bottom < 4.0f) {
            eskf.updateToF(pkt.tof_bottom);
        }

        if (i % mag_rate == 0) {
            Vector3 mag(pkt.mag_x, pkt.mag_y, pkt.mag_z);
            eskf.updateMag(mag);
        }

        if (i % flow_rate == 0 && pkt.flow_squal >= flow_squal_min) {
            // Convert raw flow to rad/s (approximate, sensor dependent)
            float flow_scale = 0.001f;  // TODO: calibrate
            float flow_x = pkt.flow_dx * flow_scale;
            float flow_y = pkt.flow_dy * flow_scale;
            float height = std::max(pkt.tof_bottom, 0.1f);
            eskf.updateFlow(flow_x, flow_y, height);
        }

        // Get state
        auto state = eskf.getState();

        // Write to CSV
        fprintf(out, "%u,%.1f,",
                pkt.timestamp_ms, dt * 1000.0f);
        fprintf(out, "%.6f,%.6f,%.6f,",
                state.position.x, state.position.y, state.position.z);
        fprintf(out, "%.6f,%.6f,%.6f,",
                state.velocity.x, state.velocity.y, state.velocity.z);
        fprintf(out, "%.4f,%.4f,%.4f,",
                state.roll * 180.0f / M_PI,
                state.pitch * 180.0f / M_PI,
                state.yaw * 180.0f / M_PI);
        fprintf(out, "%.8f,%.8f,%.8f,",
                state.gyro_bias.x, state.gyro_bias.y, state.gyro_bias.z);
        fprintf(out, "%.8f,%.8f,%.8f,",
                state.accel_bias.x, state.accel_bias.y, state.accel_bias.z);
        fprintf(out, "%.4f,%.4f,%.4f,",
                pkt.accel_x, pkt.accel_y, pkt.accel_z);
        fprintf(out, "%.6f,%.6f,%.6f,",
                pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);
        fprintf(out, "%.4f,%.4f,%d\n",
                pkt.baro_alt, pkt.tof_bottom, pkt.flow_squal);

        packet_count++;

        if (verbose && packet_count % 1000 == 0) {
            printf("Processed %d packets (t=%.2fs)\n",
                   packet_count, pkt.timestamp_ms / 1000.0f);
        }
    }

    fclose(out);

    // Final summary
    auto final_state = eskf.getState();
    printf("\n=== Replay Complete ===\n");
    printf("Processed %d packets\n", packet_count);
    printf("Duration: %.2f seconds\n",
           (packets.back().timestamp_ms - packets.front().timestamp_ms) / 1000.0f);
    printf("\nFinal state:\n");
    printf("  Position: [%.3f, %.3f, %.3f] m\n",
           final_state.position.x, final_state.position.y, final_state.position.z);
    printf("  Velocity: [%.3f, %.3f, %.3f] m/s\n",
           final_state.velocity.x, final_state.velocity.y, final_state.velocity.z);
    printf("  Attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           final_state.roll * 180.0f / M_PI,
           final_state.pitch * 180.0f / M_PI,
           final_state.yaw * 180.0f / M_PI);
    printf("  Gyro bias: [%.6f, %.6f, %.6f] rad/s\n",
           final_state.gyro_bias.x, final_state.gyro_bias.y, final_state.gyro_bias.z);
    printf("\nOutput written to: %s\n", output_file);

    return 0;
}
