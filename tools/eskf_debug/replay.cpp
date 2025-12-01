/**
 * @file replay.cpp
 * @brief ESKF Replay Tool - Run ESKF on recorded sensor data
 *
 * Usage:
 *   ./eskf_replay <input.bin> <output.csv> [options]
 *
 * Reads binary log file from StampFly, runs ESKF on each packet,
 * and outputs state estimates to CSV for comparison with device.
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

// Binary log packet structure V1 (must match cli.hpp)
#pragma pack(push, 1)
struct BinaryLogPacketV1 {
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

static_assert(sizeof(BinaryLogPacketV1) == 64, "V1 Packet size mismatch");

// Binary log packet structure V2 (must match cli.hpp)
#pragma pack(push, 1)
struct BinaryLogPacketV2 {
    uint8_t header[2];      // 0xAA, 0x56 (V2)
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
    // ESKF estimates from device
    float pos_x, pos_y, pos_z;
    float vel_x, vel_y, vel_z;
    float roll, pitch, yaw;
    float gyro_bias_z;
    float accel_bias_x, accel_bias_y;
    // Status + metadata
    uint8_t eskf_status;
    float baro_ref_alt;
    uint8_t reserved[11];
    uint8_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacketV2) == 128, "V2 Packet size mismatch");

// Common packet data for ESKF processing
struct SensorData {
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float pressure;
    float baro_alt;
    float baro_ref_alt;
    float tof_bottom;
    float tof_front;
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    // Device ESKF estimates (V2 only)
    float dev_pos_x, dev_pos_y, dev_pos_z;
    float dev_vel_x, dev_vel_y, dev_vel_z;
    float dev_roll, dev_pitch, dev_yaw;
    float gyro_bias_z;
    float accel_bias_x, accel_bias_y;
    bool has_device_eskf;
};

bool verify_checksum_v1(const BinaryLogPacketV1& pkt)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 63; i++) {
        checksum ^= data[i];
    }
    return checksum == pkt.checksum;
}

bool verify_checksum_v2(const BinaryLogPacketV2& pkt)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 127; i++) {
        checksum ^= data[i];
    }
    return checksum == pkt.checksum;
}

int detect_log_format(const char* filename)
{
    FILE* f = fopen(filename, "rb");
    if (!f) return 0;

    uint8_t header[2];
    if (fread(header, 1, 2, f) != 2) {
        fclose(f);
        return 0;
    }
    fclose(f);

    if (header[0] == 0xAA && header[1] == 0x55) return 1;
    if (header[0] == 0xAA && header[1] == 0x56) return 2;
    return 0;
}

std::vector<SensorData> load_log_file(const char* filename, int& version)
{
    std::vector<SensorData> packets;

    version = detect_log_format(filename);
    if (version == 0) {
        fprintf(stderr, "Error: Unknown log format in %s\n", filename);
        return packets;
    }

    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Error: Cannot open file %s\n", filename);
        return packets;
    }

    printf("Detected log format: V%d (%d bytes/packet)\n", version, version == 1 ? 64 : 128);

    int checksum_errors = 0;

    if (version == 1) {
        BinaryLogPacketV1 pkt;
        while (fread(&pkt, sizeof(pkt), 1, f) == 1) {
            if (pkt.header[0] == 0xAA && pkt.header[1] == 0x55) {
                if (verify_checksum_v1(pkt)) {
                    SensorData data = {};
                    data.timestamp_ms = pkt.timestamp_ms;
                    data.accel_x = pkt.accel_x;
                    data.accel_y = pkt.accel_y;
                    data.accel_z = pkt.accel_z;
                    data.gyro_x = pkt.gyro_x;
                    data.gyro_y = pkt.gyro_y;
                    data.gyro_z = pkt.gyro_z;
                    data.mag_x = pkt.mag_x;
                    data.mag_y = pkt.mag_y;
                    data.mag_z = pkt.mag_z;
                    data.pressure = pkt.pressure;
                    data.baro_alt = pkt.baro_alt;
                    data.baro_ref_alt = 0.0f;
                    data.tof_bottom = pkt.tof_bottom;
                    data.tof_front = pkt.tof_front;
                    data.flow_dx = pkt.flow_dx;
                    data.flow_dy = pkt.flow_dy;
                    data.flow_squal = pkt.flow_squal;
                    data.gyro_bias_z = 0.0f;
                    data.has_device_eskf = false;
                    packets.push_back(data);
                } else {
                    checksum_errors++;
                }
            }
        }
    } else {
        BinaryLogPacketV2 pkt;
        while (fread(&pkt, sizeof(pkt), 1, f) == 1) {
            if (pkt.header[0] == 0xAA && pkt.header[1] == 0x56) {
                if (verify_checksum_v2(pkt)) {
                    SensorData data = {};
                    data.timestamp_ms = pkt.timestamp_ms;
                    data.accel_x = pkt.accel_x;
                    data.accel_y = pkt.accel_y;
                    data.accel_z = pkt.accel_z;
                    data.gyro_x = pkt.gyro_x;
                    data.gyro_y = pkt.gyro_y;
                    data.gyro_z = pkt.gyro_z;
                    data.mag_x = pkt.mag_x;
                    data.mag_y = pkt.mag_y;
                    data.mag_z = pkt.mag_z;
                    data.pressure = pkt.pressure;
                    data.baro_alt = pkt.baro_alt;
                    data.baro_ref_alt = pkt.baro_ref_alt;
                    data.tof_bottom = pkt.tof_bottom;
                    data.tof_front = pkt.tof_front;
                    data.flow_dx = pkt.flow_dx;
                    data.flow_dy = pkt.flow_dy;
                    data.flow_squal = pkt.flow_squal;
                    data.dev_pos_x = pkt.pos_x;
                    data.dev_pos_y = pkt.pos_y;
                    data.dev_pos_z = pkt.pos_z;
                    data.dev_vel_x = pkt.vel_x;
                    data.dev_vel_y = pkt.vel_y;
                    data.dev_vel_z = pkt.vel_z;
                    data.dev_roll = pkt.roll;
                    data.dev_pitch = pkt.pitch;
                    data.dev_yaw = pkt.yaw;
                    data.gyro_bias_z = pkt.gyro_bias_z;
                    data.accel_bias_x = pkt.accel_bias_x;
                    data.accel_bias_y = pkt.accel_bias_y;
                    data.has_device_eskf = (pkt.eskf_status != 0);
                    packets.push_back(data);
                } else {
                    checksum_errors++;
                }
            }
        }
    }

    fclose(f);
    printf("Loaded %zu packets from %s", packets.size(), filename);
    if (checksum_errors > 0) {
        printf(" (%d checksum errors)\n", checksum_errors);
    } else {
        printf("\n");
    }

    return packets;
}

void print_usage(const char* prog)
{
    printf("Usage: %s <input.bin> <output.csv> [options]\n", prog);
    printf("\nOptions:\n");
    printf("  --baro-rate N    Baro update rate divisor (default: 2 = 50Hz)\n");
    printf("  --tof-rate N     ToF update rate divisor (default: 3 = 33Hz)\n");
    printf("  --mag-rate N     Mag update rate divisor (default: 10 = 10Hz)\n");
    printf("  --flow-rate N    Flow update rate divisor (default: 1 = 100Hz)\n");
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

    // Default update rates (divisor of 100Hz base rate)
    int baro_rate = 2;    // 50Hz
    int tof_rate = 3;     // 33Hz
    int mag_rate = 10;    // 10Hz
    int flow_rate = 1;    // 100Hz
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
    int log_version = 0;
    auto packets = load_log_file(input_file, log_version);
    if (packets.empty()) {
        fprintf(stderr, "No valid packets found\n");
        return 1;
    }
    bool has_device_eskf = (log_version == 2);

    // Initialize ESKF with device-matched parameters
    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    config.flow_min_height = 0.02f;
    config.flow_noise = 0.1f;  // Lower = trust flow more
    config.tof_tilt_threshold = 0.35f;
    eskf.init(config);
    eskf.reset();

    // Restore biases from device log (V2 only)
    if (has_device_eskf && packets.size() > 0) {
        Vector3 initial_gyro_bias(0.0f, 0.0f, packets[0].gyro_bias_z);
        eskf.setGyroBias(initial_gyro_bias);
        printf("Gyro bias Z restored: %.6f rad/s\n", packets[0].gyro_bias_z);

        Vector3 initial_accel_bias(packets[0].accel_bias_x, packets[0].accel_bias_y, 0.0f);
        eskf.setAccelBias(initial_accel_bias);
        printf("Accel bias restored: (%.6f, %.6f) m/s²\n",
               packets[0].accel_bias_x, packets[0].accel_bias_y);
    }

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
    fprintf(out, "raw_baro_alt,raw_tof,raw_flow_squal,");
    fprintf(out, "raw_flow_dx,raw_flow_dy");
    if (has_device_eskf) {
        fprintf(out, ",dev_pos_x,dev_pos_y,dev_pos_z,");
        fprintf(out, "dev_vel_x,dev_vel_y,dev_vel_z,");
        fprintf(out, "dev_roll_deg,dev_pitch_deg,dev_yaw_deg");
    }
    fprintf(out, "\n");

    // Process packets
    uint32_t last_timestamp = packets[0].timestamp_ms;
    int packet_count = 0;

    // No IMU averaging (device does 400Hz->4avg->100Hz, but log is already 100Hz)
    constexpr int IMU_HISTORY_SIZE = 1;
    Vector3 accel_history[IMU_HISTORY_SIZE];
    Vector3 gyro_history[IMU_HISTORY_SIZE];
    int imu_history_idx = 0;
    for (int j = 0; j < IMU_HISTORY_SIZE; j++) {
        accel_history[j] = Vector3::zero();
        gyro_history[j] = Vector3::zero();
    }

    // Baro reference altitude
    float baro_alt_reference = packets[0].baro_ref_alt;
    if (std::abs(baro_alt_reference) < 0.001f) {
        baro_alt_reference = packets[0].baro_alt;
        if (std::abs(baro_alt_reference) < 0.001f && packets[0].pressure > 80000.0f) {
            constexpr float P0 = 101325.0f;
            baro_alt_reference = 44330.0f * (1.0f - std::pow(packets[0].pressure / P0, 0.1903f));
        }
        printf("Baro reference (from first packet): %.3f m\n", baro_alt_reference);
    } else {
        printf("Baro reference (from device): %.3f m\n", baro_alt_reference);
    }

    // Mag reference (accumulated from first 10 samples)
    constexpr int MAG_REF_SAMPLES = 10;
    Vector3 mag_sum = Vector3::zero();
    int mag_sample_count = 0;
    bool mag_ref_initialized = false;

    for (size_t i = 0; i < packets.size(); i++) {
        const auto& pkt = packets[i];

        // Calculate dt
        float dt = (pkt.timestamp_ms - last_timestamp) / 1000.0f;
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.01f;
        }
        last_timestamp = pkt.timestamp_ms;

        // IMU data (convert [g] to [m/s²] if needed)
        constexpr float GRAVITY = 9.81f;
        float accel_scale = (std::abs(pkt.accel_z) < 2.0f) ? GRAVITY : 1.0f;
        Vector3 accel_raw(pkt.accel_x * accel_scale,
                          pkt.accel_y * accel_scale,
                          pkt.accel_z * accel_scale);
        Vector3 gyro_raw(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);

        // Update IMU history
        accel_history[imu_history_idx] = accel_raw;
        gyro_history[imu_history_idx] = gyro_raw;
        imu_history_idx = (imu_history_idx + 1) % IMU_HISTORY_SIZE;

        // Calculate smoothed IMU
        Vector3 accel = Vector3::zero();
        Vector3 gyro = Vector3::zero();
        for (int j = 0; j < IMU_HISTORY_SIZE; j++) {
            accel += accel_history[j];
            gyro += gyro_history[j];
        }
        accel = accel * (1.0f / IMU_HISTORY_SIZE);
        gyro = gyro * (1.0f / IMU_HISTORY_SIZE);

        // ESKF predict (100Hz)
        eskf.predict(accel, gyro, dt);

        // Accel attitude update (50Hz)
        static int accel_att_counter = 0;
        if (++accel_att_counter >= 2) {
            accel_att_counter = 0;
            eskf.updateAccelAttitude(accel);
        }

        // Baro update
        float baro_alt = pkt.baro_alt;
        if (std::abs(baro_alt) < 0.001f && pkt.pressure > 80000.0f) {
            constexpr float P0 = 101325.0f;
            baro_alt = 44330.0f * (1.0f - std::pow(pkt.pressure / P0, 0.1903f));
        }
        float baro_alt_relative = baro_alt - baro_alt_reference;
        if (i % baro_rate == 0) {
            eskf.updateBaro(baro_alt_relative);
        }

        // ToF update
        if (i % tof_rate == 0 && pkt.tof_bottom > 0.01f && pkt.tof_bottom < 4.0f) {
            eskf.updateToF(pkt.tof_bottom);
        }

        // Mag update (device uses mag when calibration data exists in NVS)
        if (i % mag_rate == 0) {
            Vector3 mag(pkt.mag_x, pkt.mag_y, pkt.mag_z);
            float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
            if (mag_norm > 10.0f) {
                if (!mag_ref_initialized) {
                    mag_sum += mag;
                    mag_sample_count++;
                    if (mag_sample_count >= MAG_REF_SAMPLES) {
                        Vector3 mag_avg = mag_sum * (1.0f / mag_sample_count);
                        eskf.setMagReference(mag_avg);
                        mag_ref_initialized = true;
                        printf("Mag reference set from %d samples: (%.1f, %.1f, %.1f) uT\n",
                               mag_sample_count, mag_avg.x, mag_avg.y, mag_avg.z);
                    }
                }
                if (mag_ref_initialized) {
                    eskf.updateMag(mag);
                }
            }
        }

        // Flow update (100Hz)
        if (i % flow_rate == 0 && pkt.flow_squal >= flow_squal_min) {
            float flow_scale = 0.08f;
            float flow_x = pkt.flow_dx * flow_scale;
            float flow_y = pkt.flow_dy * flow_scale;
            float height = std::max(pkt.tof_bottom, 0.02f);
            eskf.updateFlowWithGyro(flow_x, flow_y, height, gyro.x, gyro.y);
        }

        // Get state and write to CSV
        auto state = eskf.getState();

        fprintf(out, "%u,%.1f,", pkt.timestamp_ms, dt * 1000.0f);
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
        fprintf(out, "%.4f,%.4f,%d,",
                pkt.baro_alt, pkt.tof_bottom, pkt.flow_squal);
        fprintf(out, "%d,%d", pkt.flow_dx, pkt.flow_dy);
        if (has_device_eskf) {
            fprintf(out, ",%.6f,%.6f,%.6f,",
                    pkt.dev_pos_x, pkt.dev_pos_y, pkt.dev_pos_z);
            fprintf(out, "%.6f,%.6f,%.6f,",
                    pkt.dev_vel_x, pkt.dev_vel_y, pkt.dev_vel_z);
            fprintf(out, "%.4f,%.4f,%.4f",
                    pkt.dev_roll * 180.0f / M_PI,
                    pkt.dev_pitch * 180.0f / M_PI,
                    pkt.dev_yaw * 180.0f / M_PI);
        }
        fprintf(out, "\n");

        packet_count++;

        if (verbose && packet_count % 1000 == 0) {
            printf("Processed %d packets (t=%.2fs)\n",
                   packet_count, pkt.timestamp_ms / 1000.0f);
        }
    }

    fclose(out);

    // Final summary
    auto final_state = eskf.getState();
    const auto& last_pkt = packets.back();
    printf("\n=== Replay Complete ===\n");
    printf("Log format: V%d\n", log_version);
    printf("Processed %d packets\n", packet_count);
    printf("Duration: %.2f seconds\n",
           (packets.back().timestamp_ms - packets.front().timestamp_ms) / 1000.0f);
    printf("\nPC ESKF Final state:\n");
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

    if (has_device_eskf) {
        printf("\nDevice ESKF Final state (from log):\n");
        printf("  Position: [%.3f, %.3f, %.3f] m\n",
               last_pkt.dev_pos_x, last_pkt.dev_pos_y, last_pkt.dev_pos_z);
        printf("  Velocity: [%.3f, %.3f, %.3f] m/s\n",
               last_pkt.dev_vel_x, last_pkt.dev_vel_y, last_pkt.dev_vel_z);
        printf("  Attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
               last_pkt.dev_roll * 180.0f / M_PI,
               last_pkt.dev_pitch * 180.0f / M_PI,
               last_pkt.dev_yaw * 180.0f / M_PI);

        float pos_err = std::sqrt(
            std::pow(final_state.position.x - last_pkt.dev_pos_x, 2) +
            std::pow(final_state.position.y - last_pkt.dev_pos_y, 2) +
            std::pow(final_state.position.z - last_pkt.dev_pos_z, 2));
        printf("\nPC vs Device position error: %.3f m\n", pos_err);
    }

    printf("\nOutput written to: %s\n", output_file);

    return 0;
}
