/**
 * @file cli.cpp
 * @brief CLI Console Implementation (USB CDC)
 *
 * USB Serial経由のコマンドラインインターフェース
 * - センサ表示
 * - キャリブレーション
 * - モーターテスト
 * - ペアリング制御
 */

#include "cli.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/usb_serial_jtag.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>

static const char* TAG = "CLI";

namespace stampfly {

// USB Serial input buffer
static constexpr size_t USB_BUF_SIZE = 256;

esp_err_t CLI::init()
{
    ESP_LOGI(TAG, "Initializing CLI");

    // USB Serial JTAG設定
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = USB_BUF_SIZE,
        .rx_buffer_size = USB_BUF_SIZE,
    };

    esp_err_t ret = usb_serial_jtag_driver_install(&usb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Serial driver: %s", esp_err_to_name(ret));
        // USB未接続でも動作可能にするためエラーを返さない
    }

    input_pos_ = 0;
    memset(input_buffer_, 0, sizeof(input_buffer_));

    initialized_ = true;
    ESP_LOGI(TAG, "CLI initialized");

    return ESP_OK;
}

void CLI::registerCommand(const char* name, CommandHandler handler, const char* help)
{
    commands_[name] = {handler, help ? help : ""};
}

void CLI::processInput()
{
    if (!initialized_) return;

    // USB Serialからデータ読み込み
    uint8_t data[64];
    int len = usb_serial_jtag_read_bytes(data, sizeof(data), 0);

    if (len <= 0) return;

    for (int i = 0; i < len; i++) {
        char c = (char)data[i];

        // エコーバック
        usb_serial_jtag_write_bytes(&c, 1, 0);

        if (c == '\r' || c == '\n') {
            if (input_pos_ > 0) {
                // 改行を出力
                const char* newline = "\r\n";
                usb_serial_jtag_write_bytes((const uint8_t*)newline, 2, 0);

                input_buffer_[input_pos_] = '\0';
                parseAndExecute(input_buffer_);
                input_pos_ = 0;
                memset(input_buffer_, 0, sizeof(input_buffer_));

                // プロンプト表示
                print("> ");
            }
        }
        else if (c == '\b' || c == 0x7F) {  // Backspace or Delete
            if (input_pos_ > 0) {
                input_pos_--;
                // 画面上のバックスペース処理
                const char* bs = "\b \b";
                usb_serial_jtag_write_bytes((const uint8_t*)bs, 3, 0);
            }
        }
        else if (c >= 0x20 && c < 0x7F) {  // Printable ASCII
            if (input_pos_ < MAX_CMD_LEN - 1) {
                input_buffer_[input_pos_++] = c;
            }
        }
    }
}

void CLI::print(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        usb_serial_jtag_write_bytes((const uint8_t*)buffer, len, portMAX_DELAY);
    }
}

void CLI::parseAndExecute(const char* line)
{
    char buffer[MAX_CMD_LEN];
    strncpy(buffer, line, MAX_CMD_LEN - 1);
    buffer[MAX_CMD_LEN - 1] = '\0';

    char* argv[MAX_ARGS];
    int argc = 0;

    char* token = strtok(buffer, " \t\n");
    while (token != nullptr && argc < static_cast<int>(MAX_ARGS)) {
        argv[argc++] = token;
        token = strtok(nullptr, " \t\n");
    }

    if (argc == 0) return;

    auto it = commands_.find(argv[0]);
    if (it != commands_.end()) {
        it->second.handler(argc, argv);
    } else {
        print("Unknown command: %s\r\n", argv[0]);
        print("Type 'help' for available commands\r\n");
    }
}

void CLI::printHelp()
{
    print("\r\n=== StampFly CLI ===\r\n");
    print("Available commands:\r\n");
    for (const auto& cmd : commands_) {
        print("  %-12s %s\r\n", cmd.first.c_str(), cmd.second.help.c_str());
    }
    print("\r\n");
}

void CLI::registerDefaultCommands()
{
    registerCommand("help", [this](int, char**) {
        printHelp();
    }, "Show available commands");

    registerCommand("status", [this](int, char**) {
        print("=== System Status ===\r\n");
        print("Flight State: INIT (stub)\r\n");
        print("Pairing: NOT_PAIRED (stub)\r\n");
        print("Battery: 4.2V (stub)\r\n");
        print("Connected: No (stub)\r\n");
    }, "Show system status");

    registerCommand("sensor", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: sensor [imu|mag|baro|tof|flow|power]\r\n");
            return;
        }

        const char* sensor = argv[1];
        if (strcmp(sensor, "imu") == 0) {
            print("IMU Data (stub):\r\n");
            print("  Accel: X=0.00, Y=0.00, Z=9.81 [m/s^2]\r\n");
            print("  Gyro:  X=0.00, Y=0.00, Z=0.00 [rad/s]\r\n");
        }
        else if (strcmp(sensor, "mag") == 0) {
            print("Magnetometer Data (stub):\r\n");
            print("  Mag: X=20.0, Y=0.0, Z=40.0 [uT]\r\n");
        }
        else if (strcmp(sensor, "baro") == 0) {
            print("Barometer Data (stub):\r\n");
            print("  Pressure: 101325 [Pa]\r\n");
            print("  Altitude: 0.00 [m]\r\n");
        }
        else if (strcmp(sensor, "tof") == 0) {
            print("ToF Data (stub):\r\n");
            print("  Bottom: 0.50 [m]\r\n");
            print("  Front:  2.00 [m]\r\n");
        }
        else if (strcmp(sensor, "flow") == 0) {
            print("Optical Flow Data (stub):\r\n");
            print("  Vx: 0.00, Vy: 0.00 [m/s]\r\n");
            print("  Quality: 80\r\n");
        }
        else if (strcmp(sensor, "power") == 0) {
            print("Power Data (stub):\r\n");
            print("  Voltage: 4.20 [V]\r\n");
            print("  Current: 0.50 [A]\r\n");
        }
        else {
            print("Unknown sensor: %s\r\n", sensor);
            print("Available: imu, mag, baro, tof, flow, power\r\n");
        }
    }, "Show sensor data");

    registerCommand("calib", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: calib [gyro|accel|mag]\r\n");
            return;
        }

        const char* type = argv[1];
        if (strcmp(type, "gyro") == 0) {
            print("Starting gyro calibration...\r\n");
            print("Keep device still for 3 seconds.\r\n");
            // TODO: SystemManager::runGyroCalibration()
            print("Gyro calibration complete (stub)\r\n");
        }
        else if (strcmp(type, "accel") == 0) {
            print("Starting accelerometer calibration...\r\n");
            print("Place device on flat surface.\r\n");
            // TODO: SystemManager::runAccelCalibration()
            print("Accel calibration complete (stub)\r\n");
        }
        else if (strcmp(type, "mag") == 0) {
            print("Starting magnetometer calibration...\r\n");
            print("Rotate device in all directions.\r\n");
            // TODO: SystemManager::runMagCalibration()
            print("Mag calibration complete (stub)\r\n");
        }
        else {
            print("Unknown calibration type: %s\r\n", type);
        }
    }, "Run calibration");

    registerCommand("motor", [this](int argc, char** argv) {
        if (argc < 2) {
            print("Usage: motor test <id> <throttle> | arm | disarm\r\n");
            print("  test <id> <throttle> - Test motor (id:1-4, throttle:0-100)\r\n");
            print("  arm                  - Arm motors\r\n");
            print("  disarm               - Disarm motors\r\n");
            return;
        }

        const char* cmd = argv[1];
        if (strcmp(cmd, "arm") == 0) {
            print("Motors armed (stub)\r\n");
            // TODO: StampFlyState::requestArm()
        }
        else if (strcmp(cmd, "disarm") == 0) {
            print("Motors disarmed (stub)\r\n");
            // TODO: StampFlyState::requestDisarm()
        }
        else if (strcmp(cmd, "test") == 0) {
            if (argc < 4) {
                print("Usage: motor test <id> <throttle>\r\n");
                return;
            }
            int id = atoi(argv[2]);
            int throttle = atoi(argv[3]);
            if (id < 1 || id > 4) {
                print("Invalid motor ID. Use 1-4.\r\n");
                return;
            }
            if (throttle < 0 || throttle > 100) {
                print("Invalid throttle. Use 0-100.\r\n");
                return;
            }
            print("Testing motor %d at %d%% (stub)\r\n", id, throttle);
            // TODO: MotorDriver::testMotor(id-1, throttle)
        }
        else {
            print("Unknown motor command: %s\r\n", cmd);
        }
    }, "Motor control");

    registerCommand("pair", [this](int, char**) {
        print("Entering pairing mode...\r\n");
        print("Press the pairing button on the controller.\r\n");
        // TODO: ControllerComm::enterPairingMode()
    }, "Enter pairing mode");

    registerCommand("unpair", [this](int, char**) {
        print("Clearing pairing information...\r\n");
        // TODO: ControllerComm::clearPairingFromNVS()
        print("Pairing cleared. Restart to apply.\r\n");
    }, "Clear pairing");

    registerCommand("reset", [this](int, char**) {
        print("Resetting system...\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }, "Reset system");

    registerCommand("gain", [this](int argc, char** argv) {
        if (argc < 3) {
            print("Usage: gain <name> <value>\r\n");
            print("Gains: kp_roll, kd_roll, kp_pitch, kd_pitch, kp_yaw, kd_yaw\r\n");
            return;
        }
        const char* name = argv[1];
        float value = atof(argv[2]);
        print("Setting gain %s = %.4f (stub)\r\n", name, value);
        // TODO: Save gain to control task
    }, "Set control gain");

    registerCommand("attitude", [this](int, char**) {
        print("Attitude (stub):\r\n");
        print("  Roll:  0.00 [deg]\r\n");
        print("  Pitch: 0.00 [deg]\r\n");
        print("  Yaw:   0.00 [deg]\r\n");
        // TODO: Get from StampFlyState
    }, "Show attitude");

    registerCommand("version", [this](int, char**) {
        print("StampFly RTOS Skeleton\r\n");
        print("  ESP-IDF: %s\r\n", esp_get_idf_version());
        print("  Chip: ESP32-S3\r\n");
    }, "Show version info");
}

}  // namespace stampfly
