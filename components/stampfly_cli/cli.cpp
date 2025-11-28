/**
 * @file cli.cpp
 * @brief CLI Console Implementation (USB CDC)
 *
 * USB Serial経由のコマンドラインインターフェース
 * - センサ表示
 * - キャリブレーション
 * - モーターテスト
 * - ペアリング制御
 *
 * Note: Uses static arrays and function pointers instead of
 * std::map/std::function to avoid dynamic memory allocation.
 */

#include "cli.hpp"
#include "stampfly_state.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <fcntl.h>
#include <unistd.h>

static const char* TAG = "CLI";

namespace stampfly {

// Forward declarations for command handlers
static void cmd_help(int argc, char** argv, void* context);
static void cmd_status(int argc, char** argv, void* context);
static void cmd_sensor(int argc, char** argv, void* context);
static void cmd_teleplot(int argc, char** argv, void* context);
static void cmd_calib(int argc, char** argv, void* context);
static void cmd_motor(int argc, char** argv, void* context);
static void cmd_pair(int argc, char** argv, void* context);
static void cmd_unpair(int argc, char** argv, void* context);
static void cmd_reset(int argc, char** argv, void* context);
static void cmd_gain(int argc, char** argv, void* context);
static void cmd_attitude(int argc, char** argv, void* context);
static void cmd_version(int argc, char** argv, void* context);

esp_err_t CLI::init()
{
    ESP_LOGI(TAG, "Initializing CLI");

    // Disable buffering on stdin and stdout for immediate I/O
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // Set stdin to non-blocking mode
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    input_pos_ = 0;
    memset(input_buffer_, 0, sizeof(input_buffer_));
    command_count_ = 0;

    initialized_ = true;
    ESP_LOGI(TAG, "CLI initialized");

    return ESP_OK;
}

void CLI::registerCommand(const char* name, CommandHandlerFn handler,
                          const char* help, void* context)
{
    if (command_count_ >= MAX_COMMANDS) {
        ESP_LOGW(TAG, "Max commands reached, cannot register: %s", name);
        return;
    }

    CommandEntry& entry = commands_[command_count_];
    strncpy(entry.name, name, MAX_CMD_NAME_LEN - 1);
    entry.name[MAX_CMD_NAME_LEN - 1] = '\0';
    strncpy(entry.help, help ? help : "", MAX_HELP_LEN - 1);
    entry.help[MAX_HELP_LEN - 1] = '\0';
    entry.handler = handler;
    entry.context = context;
    command_count_++;
}

void CLI::processInput()
{
    if (!initialized_) return;

    // Non-blocking read from stdin
    int c = getchar();
    if (c == EOF) return;

    // エコーバック - use write() to bypass stdio buffering
    char ch = static_cast<char>(c);
    write(fileno(stdout), &ch, 1);
    fsync(fileno(stdout));

    if (c == '\r' || c == '\n') {
        if (input_pos_ > 0) {
            // 改行を出力
            const char* newline = "\r\n";
            write(fileno(stdout), newline, 2);

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
            write(fileno(stdout), bs, 3);
        }
    }
    else if (c >= 0x20 && c < 0x7F) {  // Printable ASCII
        if (input_pos_ < MAX_CMD_LEN - 1) {
            input_buffer_[input_pos_++] = ch;
        }
    }
}

void CLI::print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
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

    // Search for command in array
    for (size_t i = 0; i < command_count_; i++) {
        if (strcmp(commands_[i].name, argv[0]) == 0) {
            commands_[i].handler(argc, argv, commands_[i].context);
            return;
        }
    }

    print("Unknown command: %s\r\n", argv[0]);
    print("Type 'help' for available commands\r\n");
}

void CLI::printHelp()
{
    print("\r\n=== StampFly CLI ===\r\n");
    print("Available commands:\r\n");
    for (size_t i = 0; i < command_count_; i++) {
        print("  %-12s %s\r\n", commands_[i].name, commands_[i].help);
    }
    print("\r\n");
}

void CLI::registerDefaultCommands()
{
    registerCommand("help", cmd_help, "Show available commands", this);
    registerCommand("status", cmd_status, "Show system status", this);
    registerCommand("sensor", cmd_sensor, "Show sensor data", this);
    registerCommand("teleplot", cmd_teleplot, "Teleplot stream [on|off]", this);
    registerCommand("calib", cmd_calib, "Run calibration", this);
    registerCommand("motor", cmd_motor, "Motor control", this);
    registerCommand("pair", cmd_pair, "Enter pairing mode", this);
    registerCommand("unpair", cmd_unpair, "Clear pairing", this);
    registerCommand("reset", cmd_reset, "Reset system", this);
    registerCommand("gain", cmd_gain, "Set control gain", this);
    registerCommand("attitude", cmd_attitude, "Show attitude", this);
    registerCommand("version", cmd_version, "Show version info", this);
}

// ========== Command Handlers ==========

static void cmd_help(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->printHelp();
}

static void cmd_status(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("=== System Status ===\r\n");
    cli->print("Flight State: INIT (stub)\r\n");
    cli->print("Pairing: NOT_PAIRED (stub)\r\n");
    cli->print("Battery: 4.2V (stub)\r\n");
    cli->print("Connected: No (stub)\r\n");
}

static void cmd_sensor(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    auto& state = StampFlyState::getInstance();

    if (argc < 2) {
        cli->print("Usage: sensor [imu|mag|baro|tof|flow|power|all]\r\n");
        return;
    }

    const char* sensor = argv[1];
    if (strcmp(sensor, "imu") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 accel, gyro;
        state.getIMUData(accel, gyro);
        cli->print("IMU:\r\n");
        cli->print("  Accel: X=%.2f, Y=%.2f, Z=%.2f [m/s^2]\r\n", accel.x, accel.y, accel.z);
        cli->print("  Gyro:  X=%.3f, Y=%.3f, Z=%.3f [rad/s]\r\n", gyro.x, gyro.y, gyro.z);
    }
    if (strcmp(sensor, "mag") == 0 || strcmp(sensor, "all") == 0) {
        Vec3 mag;
        state.getMagData(mag);
        cli->print("Mag: X=%.1f, Y=%.1f, Z=%.1f [uT]\r\n", mag.x, mag.y, mag.z);
    }
    if (strcmp(sensor, "baro") == 0 || strcmp(sensor, "all") == 0) {
        float altitude, pressure;
        state.getBaroData(altitude, pressure);
        cli->print("Baro: Pressure=%.0f [Pa], Alt=%.2f [m]\r\n", pressure, altitude);
    }
    if (strcmp(sensor, "tof") == 0 || strcmp(sensor, "all") == 0) {
        float bottom, front;
        state.getToFData(bottom, front);
        cli->print("ToF: Bottom=%.3f [m], Front=%.3f [m]\r\n", bottom, front);
    }
    if (strcmp(sensor, "flow") == 0 || strcmp(sensor, "all") == 0) {
        float vx, vy;
        state.getFlowData(vx, vy);
        cli->print("OptFlow: Vx=%.3f, Vy=%.3f [m/s]\r\n", vx, vy);
    }
    if (strcmp(sensor, "power") == 0 || strcmp(sensor, "all") == 0) {
        float voltage, current;
        state.getPowerData(voltage, current);
        cli->print("Power: %.2f [V], %.1f [mA]\r\n", voltage, current * 1000.0f);
    }
    if (strcmp(sensor, "imu") != 0 && strcmp(sensor, "mag") != 0 &&
        strcmp(sensor, "baro") != 0 && strcmp(sensor, "tof") != 0 &&
        strcmp(sensor, "flow") != 0 && strcmp(sensor, "power") != 0 &&
        strcmp(sensor, "all") != 0) {
        cli->print("Unknown sensor: %s\r\n", sensor);
        cli->print("Available: imu, mag, baro, tof, flow, power, all\r\n");
    }
}

static void cmd_teleplot(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: teleplot [on|off]\r\n");
        cli->print("Current: %s\r\n", cli->isTeleplotEnabled() ? "on" : "off");
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        cli->setTeleplotEnabled(true);
        cli->print("Teleplot streaming ON\r\n");
    } else if (strcmp(argv[1], "off") == 0) {
        cli->setTeleplotEnabled(false);
        cli->print("Teleplot streaming OFF\r\n");
    } else {
        cli->print("Usage: teleplot [on|off]\r\n");
    }
}

static void cmd_calib(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: calib [gyro|accel|mag]\r\n");
        return;
    }

    const char* type = argv[1];
    if (strcmp(type, "gyro") == 0) {
        cli->print("Starting gyro calibration...\r\n");
        cli->print("Keep device still for 3 seconds.\r\n");
        // TODO: SystemManager::runGyroCalibration()
        cli->print("Gyro calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "accel") == 0) {
        cli->print("Starting accelerometer calibration...\r\n");
        cli->print("Place device on flat surface.\r\n");
        // TODO: SystemManager::runAccelCalibration()
        cli->print("Accel calibration complete (stub)\r\n");
    }
    else if (strcmp(type, "mag") == 0) {
        cli->print("Starting magnetometer calibration...\r\n");
        cli->print("Rotate device in all directions.\r\n");
        // TODO: SystemManager::runMagCalibration()
        cli->print("Mag calibration complete (stub)\r\n");
    }
    else {
        cli->print("Unknown calibration type: %s\r\n", type);
    }
}

static void cmd_motor(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 2) {
        cli->print("Usage: motor test <id> <throttle> | arm | disarm\r\n");
        cli->print("  test <id> <throttle> - Test motor (id:1-4, throttle:0-100)\r\n");
        cli->print("  arm                  - Arm motors\r\n");
        cli->print("  disarm               - Disarm motors\r\n");
        return;
    }

    const char* cmd = argv[1];
    if (strcmp(cmd, "arm") == 0) {
        cli->print("Motors armed (stub)\r\n");
        // TODO: StampFlyState::requestArm()
    }
    else if (strcmp(cmd, "disarm") == 0) {
        cli->print("Motors disarmed (stub)\r\n");
        // TODO: StampFlyState::requestDisarm()
    }
    else if (strcmp(cmd, "test") == 0) {
        if (argc < 4) {
            cli->print("Usage: motor test <id> <throttle>\r\n");
            return;
        }
        int id = atoi(argv[2]);
        int throttle = atoi(argv[3]);
        if (id < 1 || id > 4) {
            cli->print("Invalid motor ID. Use 1-4.\r\n");
            return;
        }
        if (throttle < 0 || throttle > 100) {
            cli->print("Invalid throttle. Use 0-100.\r\n");
            return;
        }
        cli->print("Testing motor %d at %d%% (stub)\r\n", id, throttle);
        // TODO: MotorDriver::testMotor(id-1, throttle)
    }
    else {
        cli->print("Unknown motor command: %s\r\n", cmd);
    }
}

static void cmd_pair(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Entering pairing mode...\r\n");
    cli->print("Press the pairing button on the controller.\r\n");
    // TODO: ControllerComm::enterPairingMode()
}

static void cmd_unpair(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Clearing pairing information...\r\n");
    // TODO: ControllerComm::clearPairingFromNVS()
    cli->print("Pairing cleared. Restart to apply.\r\n");
}

static void cmd_reset(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Resetting system...\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

static void cmd_gain(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    if (argc < 3) {
        cli->print("Usage: gain <name> <value>\r\n");
        cli->print("Gains: kp_roll, kd_roll, kp_pitch, kd_pitch, kp_yaw, kd_yaw\r\n");
        return;
    }
    const char* name = argv[1];
    float value = atof(argv[2]);
    cli->print("Setting gain %s = %.4f (stub)\r\n", name, value);
    // TODO: Save gain to control task
}

static void cmd_attitude(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("Attitude (stub):\r\n");
    cli->print("  Roll:  0.00 [deg]\r\n");
    cli->print("  Pitch: 0.00 [deg]\r\n");
    cli->print("  Yaw:   0.00 [deg]\r\n");
    // TODO: Get from StampFlyState
}

static void cmd_version(int, char**, void* context)
{
    CLI* cli = static_cast<CLI*>(context);
    cli->print("StampFly RTOS Skeleton\r\n");
    cli->print("  ESP-IDF: %s\r\n", esp_get_idf_version());
    cli->print("  Chip: ESP32-S3\r\n");
}

void CLI::outputTeleplot()
{
    if (!teleplot_enabled_) return;

    auto& state = StampFlyState::getInstance();

    // IMU data
    Vec3 accel, gyro;
    state.getIMUData(accel, gyro);
    print(">accel_x:%.3f\r\n", accel.x);
    print(">accel_y:%.3f\r\n", accel.y);
    print(">accel_z:%.3f\r\n", accel.z);
    print(">gyro_x:%.4f\r\n", gyro.x);
    print(">gyro_y:%.4f\r\n", gyro.y);
    print(">gyro_z:%.4f\r\n", gyro.z);

    // Mag data
    Vec3 mag;
    state.getMagData(mag);
    print(">mag_x:%.1f\r\n", mag.x);
    print(">mag_y:%.1f\r\n", mag.y);
    print(">mag_z:%.1f\r\n", mag.z);

    // Baro data
    float altitude, pressure;
    state.getBaroData(altitude, pressure);
    print(">pressure:%.0f\r\n", pressure);
    print(">altitude:%.3f\r\n", altitude);

    // ToF data
    float tof_bottom, tof_front;
    state.getToFData(tof_bottom, tof_front);
    print(">tof_bottom:%.3f\r\n", tof_bottom);
    print(">tof_front:%.3f\r\n", tof_front);

    // OptFlow data
    float flow_vx, flow_vy;
    state.getFlowData(flow_vx, flow_vy);
    print(">flow_vx:%.4f\r\n", flow_vx);
    print(">flow_vy:%.4f\r\n", flow_vy);

    // Power data
    float voltage, current;
    state.getPowerData(voltage, current);
    print(">voltage:%.3f\r\n", voltage);
    print(">current:%.1f\r\n", current * 1000.0f);
}

}  // namespace stampfly
