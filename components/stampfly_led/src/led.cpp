/**
 * @file led.cpp
 * @brief RGB LED (WS2812) Implementation using ESP-IDF RMT driver
 */
#include "led.hpp"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include <cstring>

static const char* TAG = "LED";

// WS2812 timing (in nanoseconds)
static constexpr uint32_t WS2812_T0H_NS = 350;
static constexpr uint32_t WS2812_T0L_NS = 900;
static constexpr uint32_t WS2812_T1H_NS = 900;
static constexpr uint32_t WS2812_T1L_NS = 350;
static constexpr uint32_t WS2812_RESET_US = 280;

namespace stampfly {

// RMT encoder for WS2812
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t* bytes_encoder;
    rmt_encoder_t* copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t* encoder, rmt_channel_handle_t channel,
                            const void* primary_data, size_t data_size,
                            rmt_encode_state_t* ret_state) {
    ws2812_encoder_t* ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state) {
    case 0:  // Send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data,
                                                  data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = static_cast<rmt_encode_state_t>(session_state);
            return encoded_symbols;
        }
        // Fall through
    case 1:  // Send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel,
                                                 &ws2812_encoder->reset_code,
                                                 sizeof(ws2812_encoder->reset_code),
                                                 &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 0;
            *ret_state = RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = static_cast<rmt_encode_state_t>(session_state);
        }
        break;
    }
    return encoded_symbols;
}

static esp_err_t ws2812_del(rmt_encoder_t* encoder) {
    ws2812_encoder_t* ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t ws2812_reset(rmt_encoder_t* encoder) {
    ws2812_encoder_t* ws2812_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_new(uint32_t resolution, rmt_encoder_handle_t* ret_encoder) {
    ws2812_encoder_t* ws2812_encoder = static_cast<ws2812_encoder_t*>(
        calloc(1, sizeof(ws2812_encoder_t)));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }

    ws2812_encoder->base.encode = ws2812_encode;
    ws2812_encoder->base.del = ws2812_del;
    ws2812_encoder->base.reset = ws2812_reset;

    // Bytes encoder for LED data
    rmt_bytes_encoder_config_t bytes_config = {};
    bytes_config.bit0.level0 = 1;
    bytes_config.bit0.duration0 = WS2812_T0H_NS * resolution / 1000000000;
    bytes_config.bit0.level1 = 0;
    bytes_config.bit0.duration1 = WS2812_T0L_NS * resolution / 1000000000;
    bytes_config.bit1.level0 = 1;
    bytes_config.bit1.duration0 = WS2812_T1H_NS * resolution / 1000000000;
    bytes_config.bit1.level1 = 0;
    bytes_config.bit1.duration1 = WS2812_T1L_NS * resolution / 1000000000;
    bytes_config.flags.msb_first = 1;

    esp_err_t ret = rmt_new_bytes_encoder(&bytes_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(ws2812_encoder);
        return ret;
    }

    // Copy encoder for reset code
    rmt_copy_encoder_config_t copy_config = {};
    ret = rmt_new_copy_encoder(&copy_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(ws2812_encoder->bytes_encoder);
        free(ws2812_encoder);
        return ret;
    }

    // Reset code (low for >280us)
    ws2812_encoder->reset_code.level0 = 0;
    ws2812_encoder->reset_code.duration0 = WS2812_RESET_US * resolution / 1000000;
    ws2812_encoder->reset_code.level1 = 0;
    ws2812_encoder->reset_code.duration1 = WS2812_RESET_US * resolution / 1000000;

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;
}

LED& LED::getInstance() {
    static LED instance;
    return instance;
}

LED::~LED() {
    if (pattern_timer_) {
        xTimerStop(pattern_timer_, 0);
        xTimerDelete(pattern_timer_, 0);
    }
    if (led_colors_) {
        free(led_colors_);
    }
    if (rmt_encoder_) {
        rmt_del_encoder(static_cast<rmt_encoder_handle_t>(rmt_encoder_));
    }
    if (rmt_channel_) {
        rmt_disable(static_cast<rmt_channel_handle_t>(rmt_channel_));
        rmt_del_channel(static_cast<rmt_channel_handle_t>(rmt_channel_));
    }
}

esp_err_t LED::init(const Config& config) {
    config_ = config;
    brightness_ = config.brightness;

    // Allocate LED color buffer
    led_colors_ = static_cast<uint32_t*>(calloc(config_.num_leds, sizeof(uint32_t)));
    if (!led_colors_) {
        ESP_LOGE(TAG, "Failed to allocate LED buffer");
        return ESP_ERR_NO_MEM;
    }

    // Initialize RMT
    esp_err_t ret = initRMT();
    if (ret != ESP_OK) {
        free(led_colors_);
        led_colors_ = nullptr;
        return ret;
    }

    // Create pattern update timer (50ms period = 20Hz)
    pattern_timer_ = xTimerCreate("led_pattern", pdMS_TO_TICKS(50), pdTRUE,
                                   this, timerCallback);
    if (!pattern_timer_) {
        ESP_LOGE(TAG, "Failed to create pattern timer");
        return ESP_ERR_NO_MEM;
    }
    xTimerStart(pattern_timer_, 0);

    initialized_ = true;
    ESP_LOGI(TAG, "LED initialized: GPIO %d, %d LEDs", config_.gpio_pin, config_.num_leds);

    // Initial display
    setAllColor(OFF);
    refresh();

    return ESP_OK;
}

esp_err_t LED::initRMT() {
    // RMT TX channel config
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = static_cast<gpio_num_t>(config_.gpio_pin);
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = 10000000;  // 10MHz, 0.1us resolution
    tx_config.mem_block_symbols = 64;
    tx_config.trans_queue_depth = 4;
    tx_config.flags.invert_out = false;
    tx_config.flags.with_dma = false;

    rmt_channel_handle_t channel = nullptr;
    esp_err_t ret = rmt_new_tx_channel(&tx_config, &channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    rmt_channel_ = channel;

    // Create WS2812 encoder
    rmt_encoder_handle_t encoder = nullptr;
    ret = ws2812_encoder_new(tx_config.resolution_hz, &encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(channel);
        return ret;
    }
    rmt_encoder_ = encoder;

    // Enable RMT channel
    ret = rmt_enable(channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(encoder);
        rmt_del_channel(channel);
        return ret;
    }

    return ESP_OK;
}

void LED::setColor(uint8_t index, uint32_t color) {
    if (!initialized_ || index >= config_.num_leds) return;
    led_colors_[index] = color;
}

void LED::setAllColor(uint32_t color) {
    if (!initialized_) return;
    for (uint8_t i = 0; i < config_.num_leds; i++) {
        led_colors_[i] = color;
    }
}

void LED::setPattern(Pattern pattern, uint32_t color) {
    current_pattern_ = pattern;
    pattern_color_ = color;
    pattern_tick_ = 0;
    blink_state_ = false;
    breathe_value_ = 0;
    breathe_direction_ = true;
}

void LED::setBrightness(uint8_t brightness) {
    brightness_ = brightness;
}

uint32_t LED::applyBrightness(uint32_t color) const {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = (r * brightness_) / 255;
    g = (g * brightness_) / 255;
    b = (b * brightness_) / 255;

    return (r << 16) | (g << 8) | b;
}

uint32_t LED::wheel(uint8_t pos) const {
    pos = 255 - pos;
    if (pos < 85) {
        return ((255 - pos * 3) << 16) | (pos * 3);
    }
    if (pos < 170) {
        pos -= 85;
        return (pos * 3) | ((255 - pos * 3) << 8);
    }
    pos -= 170;
    return ((pos * 3) << 8) | (255 - pos * 3);
}

void LED::update() {
    if (!initialized_) return;

    switch (current_pattern_) {
    case Pattern::OFF:
        setAllColor(OFF);
        break;

    case Pattern::SOLID:
        setAllColor(pattern_color_);
        break;

    case Pattern::BLINK_SLOW:
        // 1Hz blink (toggle every 500ms = 10 ticks at 20Hz)
        if (pattern_tick_ % 10 == 0) {
            blink_state_ = !blink_state_;
        }
        setAllColor(blink_state_ ? pattern_color_ : OFF);
        break;

    case Pattern::BLINK_FAST:
        // 4Hz blink (toggle every 125ms = 2-3 ticks at 20Hz)
        if (pattern_tick_ % 3 == 0) {
            blink_state_ = !blink_state_;
        }
        setAllColor(blink_state_ ? pattern_color_ : OFF);
        break;

    case Pattern::BREATHE: {
        // Smooth fade in/out
        if (breathe_direction_) {
            breathe_value_ += 5;
            if (breathe_value_ >= 250) {
                breathe_direction_ = false;
            }
        } else {
            breathe_value_ -= 5;
            if (breathe_value_ <= 5) {
                breathe_direction_ = true;
            }
        }

        uint8_t r = ((pattern_color_ >> 16) & 0xFF) * breathe_value_ / 255;
        uint8_t g = ((pattern_color_ >> 8) & 0xFF) * breathe_value_ / 255;
        uint8_t b = (pattern_color_ & 0xFF) * breathe_value_ / 255;
        setAllColor((r << 16) | (g << 8) | b);
        break;
    }

    case Pattern::RAINBOW:
        for (uint8_t i = 0; i < config_.num_leds; i++) {
            led_colors_[i] = wheel((i * 256 / config_.num_leds + rainbow_offset_) & 0xFF);
        }
        rainbow_offset_++;
        break;
    }

    pattern_tick_++;
    refresh();
}

void LED::refresh() {
    if (!initialized_ || !rmt_channel_ || !rmt_encoder_) return;

    // Prepare GRB data for WS2812
    uint8_t* grb_data = static_cast<uint8_t*>(malloc(config_.num_leds * 3));
    if (!grb_data) return;

    for (uint8_t i = 0; i < config_.num_leds; i++) {
        uint32_t color = applyBrightness(led_colors_[i]);
        // WS2812 uses GRB order
        grb_data[i * 3 + 0] = (color >> 8) & 0xFF;   // G
        grb_data[i * 3 + 1] = (color >> 16) & 0xFF;  // R
        grb_data[i * 3 + 2] = color & 0xFF;          // B
    }

    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;

    rmt_transmit(static_cast<rmt_channel_handle_t>(rmt_channel_),
                 static_cast<rmt_encoder_handle_t>(rmt_encoder_),
                 grb_data, config_.num_leds * 3, &tx_config);

    rmt_tx_wait_all_done(static_cast<rmt_channel_handle_t>(rmt_channel_), pdMS_TO_TICKS(100));

    free(grb_data);
}

void LED::timerCallback(TimerHandle_t timer) {
    LED* led = static_cast<LED*>(pvTimerGetTimerID(timer));
    if (led) {
        led->handleTimer();
    }
}

void LED::handleTimer() {
    update();
}

// Predefined state displays
void LED::showInit() {
    setPattern(Pattern::BLINK_SLOW, WHITE);
}

void LED::showCalibrating() {
    setPattern(Pattern::SOLID, PURPLE);
}

void LED::showIdle() {
    setPattern(Pattern::SOLID, GREEN);
}

void LED::showArmed() {
    setPattern(Pattern::SOLID, YELLOW);
}

void LED::showFlying() {
    setPattern(Pattern::BLINK_FAST, YELLOW);
}

void LED::showPairing() {
    setPattern(Pattern::BLINK_SLOW, BLUE);
}

void LED::showConnected() {
    setPattern(Pattern::BLINK_SLOW, GREEN);
}

void LED::showDisconnected() {
    setPattern(Pattern::BLINK_SLOW, RED);
}

void LED::showLowBattery() {
    setPattern(Pattern::BLINK_FAST, RED);
}

void LED::showError() {
    setPattern(Pattern::SOLID, RED);
}

}  // namespace stampfly
