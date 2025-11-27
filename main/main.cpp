/**
 * @file main.cpp
 * @brief StampFly RTOS Skeleton - Main Entry Point
 */

#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "hardware_config.hpp"

static const char* TAG = "main";

// Forward declarations
static esp_err_t init_nvs();
static esp_err_t init_gpio();
static esp_err_t init_spi();
static esp_err_t init_i2c();

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  StampFly RTOS Skeleton");
    ESP_LOGI(TAG, "  Version: 0.1.0");
    ESP_LOGI(TAG, "===========================================");

    // 1. NVS initialization
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "NVS initialized");

    // 2. Hardware initialization
    ESP_ERROR_CHECK(init_gpio());
    ESP_LOGI(TAG, "GPIO initialized");

    ESP_ERROR_CHECK(init_spi());
    ESP_LOGI(TAG, "SPI initialized");

    ESP_ERROR_CHECK(init_i2c());
    ESP_LOGI(TAG, "I2C initialized");

    // TODO: Phase 2 - Sensor driver initialization
    // imu.init();
    // mag.init();
    // baro.init();
    // tof_front.init();
    // tof_bottom.init();
    // flow.init();

    // TODO: Phase 3 - Peripheral initialization
    // power.init();
    // buzzer.init();
    // led.init();
    // button.init();

    // TODO: Phase 3.5 - Motor initialization
    // motor.init();

    // TODO: Phase 4 - State management initialization
    // StampFlyState::getInstance().loadFromNVS();

    // TODO: Phase 5 - Communication initialization
    // comm.init();

    // TODO: Phase 6 - CLI initialization
    // cli.init();

    // TODO: Phase 7 - Task creation
    // Sensor tasks
    // xTaskCreate(IMUTask, "IMU", hw::task::STACK_IMU, NULL, hw::task::PRIORITY_IMU, NULL);
    // xTaskCreate(MagTask, "Mag", hw::task::STACK_MAG, NULL, hw::task::PRIORITY_MAG, NULL);
    // xTaskCreate(BaroTask, "Baro", hw::task::STACK_BARO, NULL, hw::task::PRIORITY_BARO, NULL);
    // xTaskCreate(ToFTask, "ToF", hw::task::STACK_TOF, NULL, hw::task::PRIORITY_TOF, NULL);
    // xTaskCreate(OptFlowTask, "Flow", hw::task::STACK_FLOW, NULL, hw::task::PRIORITY_FLOW, NULL);

    // Peripheral tasks
    // xTaskCreate(PowerTask, "Power", hw::task::STACK_POWER, NULL, hw::task::PRIORITY_POWER, NULL);
    // xTaskCreate(LEDTask, "LED", hw::task::STACK_LED, NULL, hw::task::PRIORITY_LED, NULL);
    // xTaskCreate(ButtonTask, "Button", hw::task::STACK_BUTTON, NULL, hw::task::PRIORITY_BUTTON, NULL);

    // Communication task
    // xTaskCreate(CommTask, "Comm", hw::task::STACK_COMM, NULL, hw::task::PRIORITY_COMM, NULL);

    // Main control task
    // xTaskCreate(MainTask, "Main", hw::task::STACK_MAIN, NULL, hw::task::PRIORITY_MAIN, NULL);

    // CLI task
    // xTaskCreate(CLITask, "CLI", hw::task::STACK_CLI, NULL, hw::task::PRIORITY_CLI, NULL);

    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Main loop (placeholder)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

static esp_err_t init_gpio()
{
    // Configure ToF XSHUT pins as outputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << stampfly::hw::tof::front::XSHUT) |
                           (1ULL << stampfly::hw::tof::bottom::XSHUT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Keep ToF sensors in reset initially
    gpio_set_level(stampfly::hw::tof::front::XSHUT, 0);
    gpio_set_level(stampfly::hw::tof::bottom::XSHUT, 0);

    // Configure button input with pullup
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << stampfly::hw::button::PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    return ESP_OK;
}

static esp_err_t init_spi()
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = stampfly::hw::spi::MOSI;
    bus_cfg.miso_io_num = stampfly::hw::spi::MISO;
    bus_cfg.sclk_io_num = stampfly::hw::spi::SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = stampfly::hw::spi::MAX_TRANSFER_SIZE;

    return spi_bus_initialize(stampfly::hw::spi::HOST, &bus_cfg, stampfly::hw::spi::DMA_CHANNEL);
}

static esp_err_t init_i2c()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = stampfly::hw::i2c::SDA;
    conf.scl_io_num = stampfly::hw::i2c::SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = stampfly::hw::i2c::FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(stampfly::hw::i2c::PORT, &conf));
    return i2c_driver_install(stampfly::hw::i2c::PORT, conf.mode, 0, 0, 0);
}
