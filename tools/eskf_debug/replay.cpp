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
    // Header (2 bytes)
    uint8_t header[2];      // 0xAA, 0x56 (V2)

    // Timestamp (4 bytes)
    uint32_t timestamp_ms;

    // IMU data (24 bytes)
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    // Magnetometer (12 bytes)
    float mag_x, mag_y, mag_z;

    // Barometer (8 bytes)
    float pressure;
    float baro_alt;

    // ToF (8 bytes)
    float tof_bottom;
    float tof_front;

    // Optical Flow (5 bytes)
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;

    // === ESKF Estimates from device (52 bytes) ===
    float pos_x, pos_y, pos_z;
    float vel_x, vel_y, vel_z;
    float roll, pitch, yaw;
    float gyro_bias_z;
    float accel_bias_x, accel_bias_y;

    // Status + metadata (17 bytes)
    uint8_t eskf_status;
    float baro_ref_alt;     // [m] barometer reference altitude
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
    float baro_ref_alt;     // Reference altitude from device
    float tof_bottom;
    float tof_front;
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    // Device ESKF estimates (for comparison, V2 only)
    float dev_pos_x, dev_pos_y, dev_pos_z;
    float dev_vel_x, dev_vel_y, dev_vel_z;
    float dev_roll, dev_pitch, dev_yaw;
    float gyro_bias_z;  // Device gyro bias Z (for PC initialization)
    float accel_bias_x, accel_bias_y;  // Device accel bias X/Y (for PC initialization)
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

// Detect log format and return version (1 or 2, 0 if unknown)
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
                    data.baro_ref_alt = 0.0f;  // V1 has no reference, will use first packet
                    data.tof_bottom = pkt.tof_bottom;
                    data.tof_front = pkt.tof_front;
                    data.flow_dx = pkt.flow_dx;
                    data.flow_dy = pkt.flow_dy;
                    data.flow_squal = pkt.flow_squal;
                    data.gyro_bias_z = 0.0f;  // V1 has no gyro bias
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
                    // Device ESKF estimates
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
    // Match device rates for accurate comparison
    int baro_rate = 2;    // 50Hz
    int tof_rate = 3;     // 33Hz
    int mag_rate = 10;    // 10Hz
    int flow_rate = 1;    // 100Hz (device: 100Hz)
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

    // Initialize ESKF
    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();
    config.flow_min_height = 0.02f;  // Allow flow updates at low height (desk test)
    // flow_noise: use default 0.5 m/s (same as device)
    // Previous value 0.01 was too small, causing velocity to be pulled to observation too strongly
    config.tof_tilt_threshold = 0.35f;  // 20° - stricter threshold for ToF reliability

    // センサデータから推定したQ/Rパラメータ (estimate_qr.py参照)
    // 結論: プロセスノイズ(Q)はデフォルト値を維持し、観測ノイズ(R)のみ調整
    //
    // 理由: 推定したQ値が小さすぎるとバイアス推定が不安定になる
    //       特にgyro_bias_noiseを小さくするとYawバイアスが発散する
    //
    // 推定値 vs デフォルト:
    //   gyro_noise:       0.0002 vs 0.001 (デフォルト維持)
    //   accel_noise:      0.001  vs 0.1   (デフォルト維持)
    //   gyro_bias_noise:  0.00002 vs 0.00005 (デフォルト維持)
    //   accel_bias_noise: 0.0001 vs 0.001 (デフォルト維持)
    //
    // 観測ノイズ(R)は推定値を参考に調整:
    //   baro_noise: 0.1m (推定0.099m)
    //   tof_noise:  0.002m (推定0.0013m) - 既にtof_tilt_thresholdで調整済み

    eskf.init(config);

    // Reset ESKF at start (to match device behavior at binlog start)
    // Device does reset() + setGyroBias() at binlog start
    eskf.reset();

    // Restore biases from device log (V2 only)
    // Device preserves initial biases after reset
    if (has_device_eskf && packets.size() > 0) {
        // Restore gyro bias Z from first packet
        Vector3 initial_gyro_bias(0.0f, 0.0f, packets[0].gyro_bias_z);
        eskf.setGyroBias(initial_gyro_bias);
        printf("Gyro bias Z restored from device: %.6f rad/s\n", packets[0].gyro_bias_z);

        // Restore accel bias X/Y from first packet
        Vector3 initial_accel_bias(packets[0].accel_bias_x, packets[0].accel_bias_y, 0.0f);
        eskf.setAccelBias(initial_accel_bias);
        printf("Accel bias restored from device: (%.6f, %.6f) m/s²\n",
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
        // Add device ESKF columns for comparison
        fprintf(out, ",dev_pos_x,dev_pos_y,dev_pos_z,");
        fprintf(out, "dev_vel_x,dev_vel_y,dev_vel_z,");
        fprintf(out, "dev_roll_deg,dev_pitch_deg,dev_yaw_deg");
    }
    fprintf(out, "\n");

    // Process packets
    uint32_t last_timestamp = packets[0].timestamp_ms;
    int packet_count = 0;

    // IMU smoothing to match device behavior
    // Device: 400Hz IMU -> LPF -> 4-sample average -> 100Hz ESKF
    // Binary log: 100Hz samples of LPF output (no 4-sample average)
    // We need to apply 4-sample moving average to match device
    constexpr int IMU_HISTORY_SIZE = 4;
    Vector3 accel_history[IMU_HISTORY_SIZE];
    Vector3 gyro_history[IMU_HISTORY_SIZE];
    int imu_history_idx = 0;
    for (int j = 0; j < IMU_HISTORY_SIZE; j++) {
        accel_history[j] = Vector3::zero();
        gyro_history[j] = Vector3::zero();
    }

    // Get baro reference altitude from device (V2) or use first packet (V1)
    float baro_alt_reference = packets[0].baro_ref_alt;
    if (std::abs(baro_alt_reference) < 0.001f) {
        // V1 or invalid reference: use first packet's baro_alt as reference
        baro_alt_reference = packets[0].baro_alt;
        if (std::abs(baro_alt_reference) < 0.001f && packets[0].pressure > 80000.0f) {
            constexpr float P0 = 101325.0f;
            baro_alt_reference = 44330.0f * (1.0f - std::pow(packets[0].pressure / P0, 0.1903f));
        }
        printf("Baro reference (from first packet): %.3f m\n", baro_alt_reference);
    } else {
        printf("Baro reference (from device): %.3f m\n", baro_alt_reference);
    }

    for (size_t i = 0; i < packets.size(); i++) {
        const auto& pkt = packets[i];

        // Calculate dt
        float dt = (pkt.timestamp_ms - last_timestamp) / 1000.0f;
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.01f;  // Default to 10ms if invalid
        }
        last_timestamp = pkt.timestamp_ms;

        // IMU data - convert [g] to [m/s²] if needed
        // Detect unit: if |az| < 2, assume [g] units; if |az| > 5, assume [m/s²]
        constexpr float GRAVITY = 9.81f;
        float accel_scale = (std::abs(pkt.accel_z) < 2.0f) ? GRAVITY : 1.0f;
        Vector3 accel_raw(pkt.accel_x * accel_scale,
                          pkt.accel_y * accel_scale,
                          pkt.accel_z * accel_scale);
        Vector3 gyro_raw(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);

        // Update IMU history for smoothing
        accel_history[imu_history_idx] = accel_raw;
        gyro_history[imu_history_idx] = gyro_raw;
        imu_history_idx = (imu_history_idx + 1) % IMU_HISTORY_SIZE;

        // Calculate smoothed IMU (4-sample moving average to match device)
        // Device does: 400Hz -> 4-sample avg -> 100Hz ESKF
        // We approximate with 4-sample moving average at 100Hz
        Vector3 accel = Vector3::zero();
        Vector3 gyro = Vector3::zero();
        for (int j = 0; j < IMU_HISTORY_SIZE; j++) {
            accel += accel_history[j];
            gyro += gyro_history[j];
        }
        accel = accel * (1.0f / IMU_HISTORY_SIZE);
        gyro = gyro * (1.0f / IMU_HISTORY_SIZE);

        // ESKF predict step (every packet = 100Hz)
        eskf.predict(accel, gyro, dt);

        // Accel attitude update (uses gravity vector for roll/pitch correction)
        // Run at 50Hz to match device (device: 100Hz/2 = 50Hz)
        static int accel_att_counter = 0;
        if (++accel_att_counter >= 2) {
            accel_att_counter = 0;
            eskf.updateAccelAttitude(accel);
        }

        // Measurement updates
        #if 1
        // Measurement updates at lower rates
        // Note: If baro_alt is 0, calculate from pressure using barometric formula
        // Use relative altitude from initial value
        float baro_alt = pkt.baro_alt;
        if (std::abs(baro_alt) < 0.001f && pkt.pressure > 80000.0f) {
            // Calculate altitude from pressure: h = 44330 * (1 - (P/P0)^0.1903)
            constexpr float P0 = 101325.0f;  // Sea level pressure [Pa]
            baro_alt = 44330.0f * (1.0f - std::pow(pkt.pressure / P0, 0.1903f));
        }
        // Calculate relative altitude (up is positive)
        // NED conversion is done inside ESKF::updateBaro()
        float baro_alt_relative = baro_alt - baro_alt_reference;
        if (i % baro_rate == 0) {
            eskf.updateBaro(baro_alt_relative);
        }

        if (i % tof_rate == 0 && pkt.tof_bottom > 0.01f && pkt.tof_bottom < 4.0f) {
            eskf.updateToF(pkt.tof_bottom);
        }

        if (i % mag_rate == 0) {
            Vector3 mag(pkt.mag_x, pkt.mag_y, pkt.mag_z);
            // Only update if mag data is valid (magnitude > 10 uT)
            float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
            if (mag_norm > 10.0f) {
                // 最初の100サンプル（1秒）でmag_refを平均化して設定
                static constexpr int MAG_REF_SAMPLES = 100;
                static Vector3 mag_sum = Vector3::zero();
                static int mag_sample_count = 0;
                static bool mag_ref_initialized = false;

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
                // mag_ref設定後のみupdateMagを呼ぶ
                if (mag_ref_initialized) {
                    eskf.updateMag(mag);
                }
            }
        }

        // Flow update: every packet (same as device behavior)
        // Device calls updateFlowWithGyro every 100Hz cycle from IMUTask
        if (i % flow_rate == 0 && pkt.flow_squal >= flow_squal_min) {
            float flow_scale = 0.08f;
            float flow_x = pkt.flow_dx * flow_scale;
            float flow_y = pkt.flow_dy * flow_scale;
            float height = std::max(pkt.tof_bottom, 0.02f);

            eskf.updateFlowWithGyro(flow_x, flow_y, height, gyro.x, gyro.y);
        }
        #endif

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
        fprintf(out, "%.4f,%.4f,%d,",
                pkt.baro_alt, pkt.tof_bottom, pkt.flow_squal);
        fprintf(out, "%d,%d",
                pkt.flow_dx, pkt.flow_dy);
        if (has_device_eskf) {
            // Output device ESKF estimates for comparison
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

        // Calculate position error
        float pos_err = std::sqrt(
            std::pow(final_state.position.x - last_pkt.dev_pos_x, 2) +
            std::pow(final_state.position.y - last_pkt.dev_pos_y, 2) +
            std::pow(final_state.position.z - last_pkt.dev_pos_z, 2));
        printf("\nPC vs Device position error: %.3f m\n", pos_err);
    }

    printf("\nOutput written to: %s\n", output_file);

    return 0;
}
