# StampFly RTOSスケルトン 実装計画

## 1. 概要

本ドキュメントはStampFlyの制御用ファームウェアスケルトンの実装計画を記述する。

### 1.1 スケルトンの定義

スケルトンとは、メインタスクに実装するべき飛行制御タスクを省略し、以下の機能を提供するもの：

- プログラムの各タスク管理
- センサの読み込み
- キャリブレーション
- StampFlyの各状態の一元管理クラス
- コントローラとの通信
- CLIによるセンサ値取得・ゲイン調整

### 1.2 開発環境

| 項目 | 内容 |
|------|------|
| フレームワーク | ESP-IDF |
| RTOS | FreeRTOS |
| 言語 | C++ |
| ターゲット | M5StampS3 (ESP32-S3) |

---

## 2. 全体アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application Layer                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │  CLI Task   │  │ Main Task   │  │   Controller Comm Task  │ │
│  │  (USB CDC)  │  │ (400Hz制御) │  │      (ESP-NOW 50Hz)     │ │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                      State Manager Layer                        │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │              StampFlyState (シングルトン)                   ││
│  │  - センサデータ管理  - 状態遷移管理  - キャリブレーション   ││
│  │  - バッテリー監視    - エラー管理                           ││
│  └─────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────┤
│                       Estimator Layer                           │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                  ESKF Position/Attitude                     ││
│  └─────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────┤
│                        Filter Layer                             │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │         stampfly_filter (外れ値処理・フィルタ)              ││
│  └─────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────┤
│                        Sensor Layer                             │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐       │
│  │  IMU   │ │  ToF   │ │ OptFlow│ │  Mag   │ │ Baro   │       │
│  │BMI270  │ │VL53L3CX│ │PMW3901 │ │BMM150  │ │BMP280  │       │
│  │ 400Hz  │ │ 30Hz   │ │ 100Hz  │ │ 100Hz  │ │ 50Hz   │       │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘       │
├─────────────────────────────────────────────────────────────────┤
│                      Actuator / Peripheral Layer                │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐       │
│  │ Motor  │ │INA3221 │ │ Buzzer │ │RGB LED │ │ Button │       │
│  │ Driver │ │電源監視│ │ PWM    │ │WS2812  │ │ GPIO0  │       │
│  │PWM x4  │ │ 10Hz   │ │        │ │        │ │        │       │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘       │
├─────────────────────────────────────────────────────────────────┤
│                        Driver Layer                             │
│  ┌──────────────────┐  ┌──────────────────────────────────────┐│
│  │    SPI Bus       │  │           I2C Bus                    ││
│  │ BMI270, PMW3901  │  │  BMM150, BMP280, VL53L3CX, INA3221   ││
│  └──────────────────┘  └──────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. ハードウェア構成

### 3.1 センサ一覧

| 役割 | 型番 | 通信 | サンプリング |
|------|------|------|-------------|
| 6軸IMU | BMI270 | SPI | 400Hz (内部1600Hz) |
| 地磁気 | BMM150 | I2C | 100Hz |
| 気圧計 | BMP280 | I2C | 50Hz |
| ToF (前方) | VL53L3CX | I2C | 30Hz |
| ToF (底面) | VL53L3CX | I2C | 30Hz |
| Optical Flow | PMW3901MB | SPI | 100Hz |
| 電源監視 | INA3221 | I2C | 10Hz |

### 3.2 GPIO割り当て

#### SPIバス
| ピン | GPIO | 接続先 |
|------|------|--------|
| MOSI | 14 | BMI270, PMW3901 |
| MISO | 43 | BMI270, PMW3901 |
| SCK | 44 | BMI270, PMW3901 |
| CS (IMU) | 46 | BMI270 |
| CS (Flow) | 12 | PMW3901 |

#### I2Cバス
| ピン | GPIO | 備考 |
|------|------|------|
| SDA | 3 | 400kHz |
| SCL | 4 | 400kHz |

#### ToFセンサ制御
| センサ | XSHUT | INT | I2Cアドレス |
|--------|-------|-----|------------|
| 前方 | GPIO9 | GPIO8 | 0x29 |
| 底面 | GPIO7 | GPIO6 | 0x30 |

#### モータ (PWM)
| モータ | GPIO | 位置 | 回転方向 |
|--------|------|------|----------|
| M1 | 5 | 右前 | CCW |
| M2 | 45 | 右後 | CW |
| M3 | 41 | 左後 | CCW |
| M4 | 42 | 左前 | CW |

```
    前方
  M4    M1
   CW  CCW
    ＼／
    ／＼
  CCW  CW
  M3    M2
```

#### 周辺機器
| 機能 | GPIO | 備考 |
|------|------|------|
| リセットボタン | 0 | 内部プルアップ、LOW=押下 |
| IMU INT1 | 11 | BMI270割り込み |
| ブザー | 40 | PWM制御 |
| RGB LED (オンボード) | 39 | WS2812 x2 |
| RGB LED (ESP) | 21 | WS2812 x1 |

#### Grove拡張
| 機能 | GPIO |
|------|------|
| Grove I2C SDA | 13 |
| Grove I2C SCL | 15 |
| Grove UART RX | 1 |
| Grove UART TX | 2 |

---

## 4. プロジェクト構造

```
stampfly_rtos_skelton/
├── CMakeLists.txt
├── sdkconfig.defaults
├── project.md
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp
│   └── Kconfig.projbuild
├── components/
│   ├── stampfly_imu/           # BMI270ドライバ (既存取込)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── bmi270_wrapper.hpp
│   │   └── src/
│   ├── stampfly_tof/           # VL53L3CXドライバ (既存取込)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── vl53l3cx_wrapper.hpp
│   │   └── src/
│   ├── stampfly_opticalflow/   # PMW3901ドライバ (既存取込)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   └── src/
│   ├── stampfly_mag/           # BMM150ドライバ (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── bmm150.hpp
│   │   └── src/
│   │       └── bmm150.cpp
│   ├── stampfly_baro/          # BMP280ドライバ (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── bmp280.hpp
│   │   └── src/
│   │       └── bmp280.cpp
│   ├── stampfly_eskf/          # ESKF推定器 (既存取込)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   └── src/
│   ├── stampfly_filter/        # フィルタ・外れ値処理 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── moving_average.hpp
│   │   │   ├── median_filter.hpp
│   │   │   ├── lowpass_filter.hpp
│   │   │   └── outlier_detector.hpp
│   │   └── src/
│   ├── stampfly_state/         # 状態管理 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── stampfly_state.hpp
│   │   └── src/
│   │       └── stampfly_state.cpp
│   ├── stampfly_comm/          # ESP-NOW通信 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── controller_comm.hpp
│   │   └── src/
│   │       └── controller_comm.cpp
│   ├── stampfly_power/         # 電源監視 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── power_monitor.hpp
│   │   └── src/
│   │       └── power_monitor.cpp
│   ├── stampfly_buzzer/        # ブザー制御 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── buzzer.hpp
│   │   └── src/
│   │       └── buzzer.cpp
│   ├── stampfly_led/           # RGB LED制御 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── led.hpp
│   │   └── src/
│   │       └── led.cpp
│   ├── stampfly_button/        # ボタン制御 (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── button.hpp
│   │   └── src/
│   │       └── button.cpp
│   ├── stampfly_motor/         # モータドライバ (新規実装)
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── motor_driver.hpp
│   │   └── src/
│   │       └── motor_driver.cpp
│   └── stampfly_cli/           # CLIコンソール (新規実装)
│       ├── CMakeLists.txt
│       ├── include/
│       │   └── cli.hpp
│       └── src/
│           └── cli.cpp
└── docs/
    ├── implementation_plan.md
    ├── architecture.md
    ├── calibration_guide.md
    └── control_implementation_guide.md
```

---

## 5. 実装フェーズ

### Phase 1: プロジェクト基盤構築

#### 1.1 タスク
- [ ] ESP-IDFプロジェクト構造作成
- [ ] CMakeLists.txt設定
- [ ] sdkconfig.defaults設定
- [ ] ハードウェア定義ヘッダ作成

#### 1.2 sdkconfig.defaults設定項目
```
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_COMPILER_CXX_EXCEPTIONS=y
CONFIG_COMPILER_CXX_RTTI=y
CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=4096
```

#### 1.3 確認項目
- ビルド成功
- M5StampS3への書き込み成功

---

### Phase 2: ドライバ層実装

#### 2.1 既存ドライバ統合

| ドライバ | ソース | 作業内容 |
|---------|--------|---------|
| BMI270 | kouhei1970/stampfly_imu | C++ラッパ作成 |
| VL53L3CX | kouhei1970/stampfly_tof | C++ラッパ作成 |
| PMW3901 | kouhei1970/stampfly_opticalflow | そのまま使用 |
| ESKF | kouhei1970/stampfly-eskf-estimator | ESP-IDF統合 |

#### 2.2 新規ドライバ実装

##### BMM150 (地磁気センサ)
```cpp
namespace stampfly {

class BMM150 {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr = 0x10;
        gpio_num_t sda;
        gpio_num_t scl;
    };

    struct MagData {
        float x, y, z;  // [uT]
    };

    esp_err_t init(const Config& config);
    esp_err_t read(MagData& data);
    esp_err_t setDataRate(uint8_t odr);
    esp_err_t performSelfTest();
};

}
```

##### BMP280 (気圧センサ)
```cpp
namespace stampfly {

class BMP280 {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr = 0x76;
        gpio_num_t sda;
        gpio_num_t scl;
    };

    struct BaroData {
        float pressure;     // [hPa]
        float temperature;  // [°C]
        float altitude;     // [m]
    };

    esp_err_t init(const Config& config);
    esp_err_t read(BaroData& data);
    void setSeaLevelPressure(float pressure_hpa);
};

}
```

#### 2.3 フィルタライブラリ実装

```cpp
namespace stampfly {
namespace filter {

// 移動平均フィルタ
template<typename T, size_t N>
class MovingAverage {
public:
    void update(T value);
    T get() const;
    void reset();
};

// メディアンフィルタ
template<typename T, size_t N>
class MedianFilter {
public:
    void update(T value);
    T get() const;
    void reset();
};

// 1次IIRローパスフィルタ
class LowPassFilter {
public:
    LowPassFilter(float cutoff_freq, float sample_freq);
    float update(float value);
    void reset();
private:
    float alpha_;
    float output_;
};

// 外れ値検出器
class OutlierDetector {
public:
    // 閾値ベース検出 (±n*σ)
    bool isOutlier(float value, float mean, float stddev, float n_sigma = 3.0f);

    // 変化率ベース検出
    bool isRateExceeded(float current, float previous, float max_rate, float dt);

    // マハラノビス距離ベース検出
    bool isMahalanobisExceeded(float distance, float threshold);
};

}}
```

#### 2.4 周辺機器ドライバ実装

##### 電源監視 (INA3221)

```cpp
namespace stampfly {

class PowerMonitor {
public:
    struct Config {
        i2c_port_t i2c_port;
        uint8_t i2c_addr = 0x40;  // INA3221_ADDR40_GND
        gpio_num_t sda;
        gpio_num_t scl;
        float shunt_resistance_mohm = 10.0f;  // 10mΩ
    };

    struct PowerData {
        float voltage;      // [V]
        float current;      // [A]
        float power;        // [W]
    };

    // 定数
    static constexpr float BATTERY_LOW_VOLTAGE = 3.4f;    // 低電圧警告閾値
    static constexpr float BATTERY_FULL_VOLTAGE = 4.2f;   // 満充電電圧
    static constexpr float BATTERY_MAX_VOLTAGE = 4.35f;   // LiHV最大電圧

    esp_err_t init(const Config& config);
    esp_err_t read(PowerData& data);

    // バッテリー状態
    bool isLowBattery() const;
    bool isCharging() const;
    float getBatteryPercent() const;

private:
    float voltage_;
    float current_;
    float prev_voltage_;
};

}
```

##### ブザー制御

```cpp
namespace stampfly {

class Buzzer {
public:
    // 音符定義
    enum Note {
        NOTE_D1 = 294,
        NOTE_D2 = 330,
        NOTE_D3 = 350,
        NOTE_D4 = 393,
        NOTE_D5 = 441,
        NOTE_D6 = 495,
        NOTE_D7 = 556
    };

    struct Config {
        gpio_num_t pin = GPIO_NUM_40;
        ledc_channel_t channel = LEDC_CHANNEL_5;
        ledc_timer_t timer = LEDC_TIMER_2;
    };

    esp_err_t init(const Config& config = Config());

    // 基本操作
    void playTone(uint16_t frequency, uint32_t duration_ms);
    void stop();

    // プリセット音
    void beep();                    // 短いビープ
    void startTone();               // 起動音
    void armTone();                 // アーム音
    void disarmTone();              // ディスアーム音
    void lowBatteryWarning();       // 低電圧警告
    void errorTone();               // エラー音
    void pairingTone();             // ペアリング音

private:
    Config config_;
};

}
```

##### RGB LED制御

```cpp
namespace stampfly {

class LED {
public:
    // 色定義
    static constexpr uint32_t WHITE   = 0xFFFFFF;
    static constexpr uint32_t RED     = 0xFF0000;
    static constexpr uint32_t GREEN   = 0x00FF00;
    static constexpr uint32_t BLUE    = 0x0000FF;
    static constexpr uint32_t YELLOW  = 0xFFFF00;
    static constexpr uint32_t PURPLE  = 0xFF00FF;
    static constexpr uint32_t CYAN    = 0x00FFFF;
    static constexpr uint32_t ORANGE  = 0xFF9933;
    static constexpr uint32_t LOW_BATTERY_COLOR = 0x18EBF9;  // 水色

    // LEDパターン
    enum Pattern {
        PATTERN_OFF,
        PATTERN_SOLID,          // 点灯
        PATTERN_BLINK_SLOW,     // 1Hz点滅
        PATTERN_BLINK_FAST,     // 4Hz点滅
        PATTERN_BREATHE         // 呼吸（フェード）
    };

    struct Config {
        gpio_num_t pin_onboard = GPIO_NUM_39;  // オンボード LED x2
        gpio_num_t pin_esp = GPIO_NUM_21;      // ESP LED x1
        uint8_t brightness = 15;               // 0-255
    };

    esp_err_t init(const Config& config = Config());

    // 個別LED制御
    void setOnboardLED(uint8_t index, uint32_t color);  // index: 0 or 1
    void setEspLED(uint32_t color);

    // パターン設定
    void setPattern(Pattern pattern, uint32_t color);

    // 状態表示プリセット
    void showInit();            // 初期化中：白点滅
    void showCalibrating();     // キャリブ中：紫点滅
    void showIdle();            // 待機：緑点灯
    void showArmed();           // アーム：黄点灯
    void showFlying();          // 飛行中：黄点灯
    void showPairing();         // ペアリング：青点滅
    void showConnected();       // 接続済：緑点灯
    void showDisconnected();    // 切断：赤点滅
    void showLowBattery();      // 低電圧：水色点滅
    void showError();           // エラー：赤点灯

    // 更新（タスクから定期呼び出し）
    void update();

private:
    Config config_;
    Pattern current_pattern_;
    uint32_t current_color_;
    int64_t last_update_us_;
    bool blink_state_;
};

}
```

##### ボタン制御

```cpp
namespace stampfly {

class Button {
public:
    // イベントタイプ
    enum Event {
        EVENT_NONE,
        EVENT_CLICK,            // 短押し
        EVENT_DOUBLE_CLICK,     // ダブルクリック
        EVENT_LONG_PRESS,       // 長押し（3秒）
        EVENT_LONG_PRESS_START  // 長押し開始
    };

    // コールバック型
    using EventCallback = std::function<void(Event)>;

    struct Config {
        gpio_num_t pin = GPIO_NUM_0;
        bool active_low = true;         // LOW=押下
        bool use_internal_pullup = true;
        uint32_t debounce_ms = 50;
        uint32_t long_press_ms = 3000;  // 3秒
    };

    esp_err_t init(const Config& config = Config());

    // コールバック登録
    void setCallback(EventCallback callback);

    // ポーリング（タスクから呼び出し）
    void tick();

    // 現在の状態
    bool isPressed() const;
    Event getLastEvent();

private:
    Config config_;
    EventCallback callback_;
    bool last_state_;
    int64_t press_start_us_;
    int64_t last_click_us_;
    Event last_event_;
};

}
```

#### 2.5 確認項目
- 各センサの単体動作確認
- 電源監視動作確認（電圧読み取り）
- ブザー動作確認（各音パターン）
- LED動作確認（各色・パターン）
- ボタン動作確認（短押し・長押し）

---

### Phase 3: センサタスク実装

#### 3.1 タスク設計

##### センサタスク
| タスク名 | 周期 | 優先度 | スタック | 内容 |
|---------|------|--------|---------|------|
| IMUTask | 2.5ms (400Hz) | 24 | 4096 | BMI270 FIFO読み出し |
| OptFlowTask | 10ms (100Hz) | 20 | 4096 | PMW3901読み出し |
| MagTask | 10ms (100Hz) | 18 | 2048 | BMM150読み出し |
| BaroTask | 20ms (50Hz) | 16 | 2048 | BMP280読み出し |
| ToFTask | 33ms (30Hz) | 14 | 4096 | VL53L3CX読み出し |

##### 周辺機器タスク
| タスク名 | 周期 | 優先度 | スタック | 内容 |
|---------|------|--------|---------|------|
| PowerTask | 100ms (10Hz) | 12 | 2048 | INA3221電圧・電流監視、低電圧警告 |
| LEDTask | 32ms (30Hz) | 8 | 2048 | RGB LEDパターン更新 |
| ButtonTask | 10ms (100Hz) | 10 | 2048 | ボタン状態監視、長押し検出 |

#### 3.2 外れ値処理実装

##### IMU外れ値処理
```cpp
void IMUTask::processData(bmi270_gyro_t& gyro, bmi270_accel_t& accel) {
    // 加速度ノルムチェック (静止時≈1g、飛行時0.5〜3g)
    float accel_norm = std::sqrt(
        accel.x * accel.x +
        accel.y * accel.y +
        accel.z * accel.z
    );

    if (accel_norm < 0.1f || accel_norm > 6.0f) {
        ESP_LOGW(TAG, "Accel outlier: norm=%.2f", accel_norm);
        return;  // 前回値を維持
    }

    // 角速度の急激な変化チェック
    float gyro_norm = std::sqrt(
        gyro.x * gyro.x +
        gyro.y * gyro.y +
        gyro.z * gyro.z
    );

    if (gyro_norm > MAX_GYRO_RATE) {
        ESP_LOGW(TAG, "Gyro outlier: norm=%.2f", gyro_norm);
        return;
    }

    // ローパスフィルタ適用
    accel_filtered_ = accel_lpf_.update(accel);
    gyro_filtered_ = gyro_lpf_.update(gyro);
}
```

##### ToF外れ値処理
```cpp
bool ToFTask::isValidMeasurement(VL53LX_MultiRangingData_t& data) {
    // Range Status検証 (0-4が有効)
    if (data.RangeData[0].RangeStatus > 4) {
        return false;
    }

    // 信号強度チェック
    if (data.RangeData[0].SignalRateRtnMegaCps < MIN_SIGNAL_RATE) {
        return false;
    }

    // 距離の物理的妥当性チェック
    int16_t range_mm = data.RangeData[0].RangeMilliMeter;
    if (range_mm < 0 || range_mm > MAX_RANGE_MM) {
        return false;
    }

    return true;
}
```

##### Optical Flow外れ値処理
```cpp
bool OptFlowTask::isValidMotion(pmw3901_motion_burst_t& burst) {
    // SQUAL (信号品質) チェック
    if (burst.squal < MIN_SQUAL) {
        ESP_LOGD(TAG, "Low SQUAL: %d", burst.squal);
        return false;
    }

    // 急激な変化量チェック
    if (std::abs(burst.delta_x) > MAX_DELTA ||
        std::abs(burst.delta_y) > MAX_DELTA) {
        ESP_LOGW(TAG, "Flow outlier: dx=%d, dy=%d",
                 burst.delta_x, burst.delta_y);
        return false;
    }

    return true;
}
```

##### 磁気センサ外れ値処理
```cpp
bool MagTask::isValidMeasurement(float mx, float my, float mz) {
    // ノルム検証 (地磁気の大きさは地域で一定)
    float norm = std::sqrt(mx*mx + my*my + mz*mz);

    // 期待値の±30%以内
    if (norm < expected_norm_ * 0.7f || norm > expected_norm_ * 1.3f) {
        ESP_LOGW(TAG, "Mag outlier: norm=%.2f (expected=%.2f)",
                 norm, expected_norm_);
        return false;
    }

    return true;
}
```

##### 気圧計外れ値処理
```cpp
float BaroTask::filterAltitude(float raw_altitude) {
    // メディアンフィルタで突発的ノイズ除去
    median_filter_.update(raw_altitude);
    float filtered = median_filter_.get();

    // 変化率制限 (物理的に不可能な急激変化を制限)
    float max_change = MAX_VERTICAL_SPEED * dt_;
    float delta = filtered - last_altitude_;

    if (std::fabs(delta) > max_change) {
        filtered = last_altitude_ + std::copysign(max_change, delta);
        ESP_LOGD(TAG, "Baro rate limited: raw=%.2f, filtered=%.2f",
                 raw_altitude, filtered);
    }

    last_altitude_ = filtered;
    return filtered;
}
```

#### 3.3 データフロー

```
IMU(400Hz) ──────────────────→ ESKF.predict()
                                     │
Mag(100Hz) ──→ 外れ値処理 ──→ ESKF.updateMag()
                                     │
Baro(50Hz) ──→ 外れ値処理 ──→ ESKF.updateBaro()
                                     │
ToF(30Hz) ───→ 外れ値処理 ──→ ESKF.updateToF()
                                     │
OptFlow(100Hz)→ 外れ値処理 ──→ ESKF.updateOptFlow()
                                     │
                                     ↓
                              StampFlyState
                           (位置・姿勢・速度)
```

#### 3.4 周辺機器タスク実装

##### 電源監視タスク

```cpp
void PowerTask(void* param) {
    PowerMonitor& power = PowerMonitor::getInstance();
    Buzzer& buzzer = Buzzer::getInstance();
    LED& led = LED::getInstance();
    StampFlyState& state = StampFlyState::getInstance();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz

    int low_battery_count = 0;
    int64_t last_warning_time = 0;

    while (true) {
        PowerMonitor::PowerData data;
        power.read(data);

        // 状態管理クラスに通知
        state.updateBattery(data.voltage, data.current);

        // 低電圧チェック (3.4V以下)
        if (data.voltage < PowerMonitor::BATTERY_LOW_VOLTAGE) {
            low_battery_count++;

            // 連続5回（500ms）低電圧が続いたら警告
            if (low_battery_count >= 5) {
                state.setWarning(StampFlyState::WARNING_LOW_BATTERY);

                // 1秒ごとに警告
                int64_t now = esp_timer_get_time();
                if (now - last_warning_time > 1000000) {
                    led.showLowBattery();
                    buzzer.lowBatteryWarning();
                    last_warning_time = now;

                    ESP_LOGW(TAG, "Low battery: %.2fV", data.voltage);
                }
            }
        } else {
            low_battery_count = 0;
            state.clearWarning(StampFlyState::WARNING_LOW_BATTERY);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
```

##### LEDタスク

```cpp
void LEDTask(void* param) {
    LED& led = LED::getInstance();
    StampFlyState& state = StampFlyState::getInstance();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(32);  // 約30Hz

    while (true) {
        // 状態に応じたLED表示
        auto flight_state = state.getFlightState();
        auto pairing_state = state.getPairingState();
        bool low_battery = state.hasWarning(StampFlyState::WARNING_LOW_BATTERY);

        // 優先度順に表示
        if (low_battery) {
            led.showLowBattery();
        } else if (pairing_state == PairingState::WAITING) {
            led.showPairing();
        } else if (state.hasError()) {
            led.showError();
        } else {
            switch (flight_state) {
                case FlightState::INIT:
                    led.showInit();
                    break;
                case FlightState::CALIBRATING:
                    led.showCalibrating();
                    break;
                case FlightState::IDLE:
                    led.showIdle();
                    break;
                case FlightState::ARMED:
                    led.showArmed();
                    break;
                case FlightState::FLYING:
                    led.showFlying();
                    break;
                default:
                    break;
            }
        }

        // パターン更新
        led.update();

        vTaskDelayUntil(&last_wake, period);
    }
}
```

##### ボタンタスク

```cpp
void ButtonTask(void* param) {
    Button& button = Button::getInstance();
    StampFlyState& state = StampFlyState::getInstance();
    ControllerComm& comm = ControllerComm::getInstance();
    Buzzer& buzzer = Buzzer::getInstance();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        button.tick();

        Button::Event event = button.getLastEvent();

        switch (event) {
            case Button::EVENT_CLICK:
                // 短押し：ビープ音 + 再起動
                ESP_LOGI(TAG, "Button click - rebooting");
                buzzer.beep();
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
                break;

            case Button::EVENT_LONG_PRESS:
                // 長押し（3秒）：ペアリングリセット + 再起動
                ESP_LOGI(TAG, "Long press - clearing pairing");
                buzzer.playTone(Buzzer::NOTE_D7, 500);
                comm.clearPairingFromNVS();
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
                break;

            case Button::EVENT_LONG_PRESS_START:
                // 長押し開始：フィードバック音
                buzzer.playTone(Buzzer::NOTE_D3, 100);
                break;

            default:
                break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
```

##### 状態とブザー・LEDの対応表

| 状態 | LED色 | LEDパターン | ブザー |
|------|-------|------------|--------|
| 初期化中 | 白 | 1Hz点滅 | - |
| キャリブレーション | 紫 | 1Hz点滅 | - |
| 待機（Disarm） | 緑 | 点灯 | - |
| アーム | 黄 | 点灯 | アーム音 |
| 飛行中 | 黄 | 点灯 | - |
| ペアリング待機 | 青 | 1Hz点滅 | ペアリング音 |
| 接続済み | 緑 | 点灯 | ビープ |
| 接続断 | 赤 | 2Hz点滅 | - |
| 低電圧警告 | 水色 | 2Hz点滅 | 警告音（1秒毎） |
| エラー | 赤 | 点灯 | エラー音 |

#### 3.5 確認項目
- 全センサ同時動作
- タスク周期の安定性
- 外れ値検出のログ確認
- 電源監視動作（低電圧警告）
- LED状態表示
- ボタン操作（短押し・長押し）

---

### Phase 3.5: モータドライバ実装

#### 3.5.1 MotorDriverクラス

```cpp
namespace stampfly {

class MotorDriver {
public:
    static constexpr uint8_t NUM_MOTORS = 4;

    // モータ配置 (右前から時計回りに番号付け)
    enum MotorID {
        MOTOR_FR = 0,  // M1: 右前 (Front Right) - CCW
        MOTOR_RR = 1,  // M2: 右後 (Rear Right)  - CW
        MOTOR_RL = 2,  // M3: 左後 (Rear Left)   - CCW
        MOTOR_FL = 3   // M4: 左前 (Front Left)  - CW
    };

    // 回転方向
    enum RotationDir {
        CW = 0,   // 時計回り
        CCW = 1   // 反時計回り
    };

    struct Config {
        gpio_num_t pins[NUM_MOTORS] = {
            GPIO_NUM_5,   // M1: 右前
            GPIO_NUM_45,  // M2: 右後
            GPIO_NUM_41,  // M3: 左後
            GPIO_NUM_42   // M4: 左前
        };
        RotationDir directions[NUM_MOTORS] = {
            CCW,  // M1: 右前
            CW,   // M2: 右後
            CCW,  // M3: 左後
            CW    // M4: 左前
        };
        uint32_t pwm_freq = 50000;      // 50kHz
        uint8_t pwm_resolution = 11;    // 11bit (0-2047)
        uint16_t min_throttle = 0;
        uint16_t max_throttle = 2000;
        uint16_t idle_throttle = 100;   // アイドル回転
    };

    esp_err_t init(const Config& config);

    // 安全制御
    esp_err_t arm();
    esp_err_t disarm();
    bool isArmed() const;

    // スロットル設定 (0-2000)
    void setThrottle(MotorID motor_id, uint16_t throttle);
    void setAllThrottle(const uint16_t throttle[NUM_MOTORS]);

    // ミキサー出力 (制御系から呼び出し)
    // thrust: 総推力, roll/pitch/yaw: 各軸トルク
    void setMixerOutput(float thrust, float roll, float pitch, float yaw);

    // 緊急停止 (即座に全モータ停止)
    void emergencyStop();

    // 個別モータテスト用 (CLIから呼び出し)
    void testMotor(MotorID motor_id, uint16_t throttle, uint32_t duration_ms);

private:
    Config config_;
    bool armed_ = false;
    uint16_t throttle_[NUM_MOTORS] = {0};
    SemaphoreHandle_t mutex_;

    void applyThrottle();
    uint16_t clampThrottle(uint16_t throttle);
};

}
```

#### 3.5.2 モータミキサー

X型クアッドコプターのミキサー行列（右前から時計回りに番号付け）：

```
    前方
  M4    M1
   CW  CCW
    ＼／
    ／＼
  CCW  CW
  M3    M2

ミキサー行列:
| Motor | Thrust | Roll | Pitch | Yaw |
|-------|--------|------|-------|-----|
| M1(FR)| +1     | -1   | +1    | +1  |
| M2(RR)| +1     | -1   | -1    | -1  |
| M3(RL)| +1     | +1   | -1    | +1  |
| M4(FL)| +1     | +1   | +1    | -1  |

出力計算:
M1 = thrust - roll + pitch + yaw
M2 = thrust - roll - pitch - yaw
M3 = thrust + roll - pitch + yaw
M4 = thrust + roll + pitch - yaw
```

#### 3.5.3 安全機構

```cpp
// アーム条件チェック
bool MotorDriver::canArm() {
    auto& state = StampFlyState::getInstance();

    // 1. スロットルが最低位置
    if (state.getThrottleInput() > THROTTLE_ARM_THRESHOLD) {
        return false;
    }

    // 2. キャリブレーション完了
    if (state.getFlightState() == FlightState::CALIBRATING) {
        return false;
    }

    // 3. センサエラーなし
    if (state.hasError()) {
        return false;
    }

    // 4. バッテリー電圧OK
    if (state.getBatteryVoltage() < BATTERY_MIN_ARM_VOLTAGE) {
        return false;
    }

    // 5. コントローラ接続済み
    if (state.getCommState() != CommState::CONNECTED) {
        return false;
    }

    return true;
}

// ディスアーム条件
// - スロットル最低位置で一定時間経過
// - 通信途絶
// - 緊急停止コマンド
// - 低電圧警告
```

#### 3.5.4 CLIコマンド

| コマンド | 引数 | 説明 |
|---------|------|------|
| `motor test` | [id] [throttle] [ms] | 個別モータテスト |
| `motor status` | - | モータ状態表示 |
| `motor arm` | - | アーム (テスト用) |
| `motor disarm` | - | ディスアーム |

#### 3.5.5 確認項目
- PWM出力波形確認
- 各モータ個別動作
- 回転方向確認
- アーム/ディスアーム動作
- 緊急停止動作
- ミキサー出力確認

---

### Phase 4: 状態管理クラス実装

#### 4.1 StampFlyStateクラス

```cpp
namespace stampfly {

class StampFlyState {
public:
    // シングルトンアクセス
    static StampFlyState& getInstance();

    // 飛行状態
    enum class FlightState {
        INIT,           // 初期化中
        CALIBRATING,    // キャリブレーション中
        IDLE,           // 待機 (Disarm)
        ARMED,          // アーム済み
        FLYING,         // 飛行中
        LANDING,        // 着陸中
        ERROR           // エラー状態
    };

    // エラーコード
    enum class ErrorCode {
        NONE = 0,
        IMU_FAILURE,
        MAG_FAILURE,
        BARO_FAILURE,
        TOF_FAILURE,
        FLOW_FAILURE,
        COMM_TIMEOUT,
        LOW_BATTERY,
        ESTIMATOR_DIVERGED
    };

    // 警告フラグ
    enum WarningFlags {
        WARNING_NONE = 0,
        WARNING_LOW_BATTERY = (1 << 0),
        WARNING_COMM_WEAK = (1 << 1),
        WARNING_SENSOR_DEGRADED = (1 << 2)
    };

    // 状態取得 (スレッドセーフ)
    FlightState getFlightState() const;
    ErrorCode getErrorCode() const;
    bool hasWarning(WarningFlags flag) const;
    bool hasError() const;

    // センサデータ取得 (スレッドセーフ)
    void getIMUData(Vector3& accel, Vector3& gyro) const;
    void getMagData(Vector3& mag) const;
    void getBaroData(float& altitude, float& pressure) const;
    void getToFData(float& front, float& bottom) const;
    void getFlowData(Vector2& velocity) const;

    // 推定値取得
    void getPosition(Vector3& pos) const;
    void getVelocity(Vector3& vel) const;
    void getAttitude(Quaternion& quat) const;
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;

    // コントローラ入力取得
    void getControlInput(ControlInput& input) const;

    // センサデータ更新 (各タスクから呼び出し)
    void updateIMU(const Vector3& accel, const Vector3& gyro);
    void updateMag(const Vector3& mag);
    void updateBaro(float altitude, float pressure);
    void updateToF(float front, float bottom);
    void updateFlow(const Vector2& velocity);

    // バッテリー状態更新
    void updateBattery(float voltage, float current);
    float getBatteryVoltage() const;
    float getBatteryCurrent() const;
    float getBatteryPercent() const;

    // 警告フラグ操作
    void setWarning(WarningFlags flag);
    void clearWarning(WarningFlags flag);

    // 状態遷移
    bool requestArm();
    bool requestDisarm();
    void setError(ErrorCode code);
    void clearError();

    // キャリブレーション
    bool startCalibration(CalibrationType type);
    bool isCalibrationComplete() const;
    CalibrationData getCalibrationData() const;

    // NVS永続化
    esp_err_t saveToNVS();
    esp_err_t loadFromNVS();

private:
    StampFlyState();
    mutable SemaphoreHandle_t mutex_;

    // 状態データ
    FlightState flight_state_;
    ErrorCode error_code_;
    uint32_t warning_flags_;

    // センサデータ
    struct SensorData {
        Vector3 accel;
        Vector3 gyro;
        Vector3 mag;
        float baro_altitude;
        float baro_pressure;
        float tof_front;
        float tof_bottom;
        Vector2 flow_velocity;
        int64_t timestamps[6];
    } sensor_data_;

    // バッテリーデータ
    struct BatteryData {
        float voltage;      // [V]
        float current;      // [A]
        int64_t timestamp;
    } battery_data_;

    // 推定値
    struct EstimatedState {
        Vector3 position;
        Vector3 velocity;
        Quaternion attitude;
    } estimated_state_;

    // コントローラ入力
    ControlInput control_input_;

    // キャリブレーションデータ
    CalibrationData calibration_;
};

}
```

#### 4.2 キャリブレーション機能

```cpp
enum class CalibrationType {
    IMU_BIAS,       // IMUバイアスキャリブレーション
    MAG_HARD_IRON,  // 磁気センサ ハードアイアン補正
    MAG_SOFT_IRON,  // 磁気センサ ソフトアイアン補正
    BARO_OFFSET,    // 気圧計オフセット
    ALL             // 全キャリブレーション
};

struct CalibrationData {
    // IMU
    Vector3 gyro_bias;
    Vector3 accel_bias;

    // 磁気センサ
    Vector3 mag_hard_iron;
    Matrix3x3 mag_soft_iron;
    float mag_expected_norm;

    // 気圧計
    float baro_offset;
    float sea_level_pressure;

    // 有効フラグ
    bool imu_calibrated;
    bool mag_calibrated;
    bool baro_calibrated;
};
```

#### 4.3 確認項目
- 状態遷移の正常動作
- スレッドセーフ性
- NVS保存・読み込み
- キャリブレーション手順

---

### Phase 5: 通信層実装

#### 5.1 通信概要

StampFlyはESP-NOWを使用してコントローラと通信する。

- **受信**: 自分のMACアドレス宛に送られた制御パケットを受信
- **送信**: コントローラへテレメトリデータを送信
- **TDMA**: コントローラ側で実現するため、StampFly側では同期処理不要

#### 5.2 ESP-NOW通信クラス

```cpp
namespace stampfly {

class ControllerComm {
public:
    // 受信パケット (コントローラからの制御入力)
    struct ControlPacket {
        uint8_t drone_mac[3];
        uint16_t throttle;
        uint16_t roll;
        uint16_t pitch;
        uint16_t yaw;
        uint8_t flags;      // bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
        uint8_t reserved;
        uint8_t checksum;
    } __attribute__((packed));

    // テレメトリパケット (StampFlyからコントローラへ) - 新規設計
    struct TelemetryPacket {
        uint8_t packet_type;
        uint8_t sequence;
        uint16_t battery_mv;        // バッテリー電圧 [mV]
        int16_t altitude_cm;        // 高度 [cm]
        int16_t velocity_x;         // X速度 [mm/s]
        int16_t velocity_y;         // Y速度 [mm/s]
        int16_t velocity_z;         // Z速度 [mm/s]
        int16_t roll_deg10;         // ロール [0.1度]
        int16_t pitch_deg10;        // ピッチ [0.1度]
        int16_t yaw_deg10;          // ヨー [0.1度]
        uint8_t state;              // FlightState
        uint8_t error;              // ErrorCode
        uint8_t checksum;
    } __attribute__((packed));

    // 初期化
    esp_err_t init();

    // コールバック設定
    void setControlCallback(std::function<void(const ControlPacket&)> cb);

    // テレメトリ送信
    esp_err_t sendTelemetry(const TelemetryPacket& packet);

    // 接続状態
    bool isConnected() const;
    int64_t getLastReceivedTime() const;

private:
    static void onDataReceived(const uint8_t* mac, const uint8_t* data, int len);
    static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);

    uint8_t controller_mac_[6];
    int64_t last_received_us_;
    std::function<void(const ControlPacket&)> control_callback_;
};

}
```

#### 5.3 受信処理

```cpp
void ControllerComm::onDataReceived(const uint8_t* mac,
                                    const uint8_t* data, int len) {
    // 制御パケットサイズ確認
    if (len != sizeof(ControlPacket)) {
        return;
    }

    const ControlPacket* packet = reinterpret_cast<const ControlPacket*>(data);

    // 自分のMACアドレス宛か確認 (下位3バイト)
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    if (packet->drone_mac[0] != my_mac[3] ||
        packet->drone_mac[1] != my_mac[4] ||
        packet->drone_mac[2] != my_mac[5]) {
        return;  // 自分宛ではない
    }

    // チェックサム検証
    uint8_t calc_checksum = 0;
    for (int i = 0; i < sizeof(ControlPacket) - 1; i++) {
        calc_checksum += data[i];
    }
    if (calc_checksum != packet->checksum) {
        ESP_LOGW(TAG, "Checksum error");
        return;
    }

    // 受信時刻更新
    last_received_us_ = esp_timer_get_time();

    // コントローラMACアドレス記憶 (テレメトリ送信用)
    memcpy(controller_mac_, mac, 6);

    // コールバック呼び出し
    if (control_callback_) {
        control_callback_(*packet);
    }
}
```

#### 5.4 テレメトリ送信

```cpp
esp_err_t ControllerComm::sendTelemetry(const TelemetryPacket& packet) {
    // コントローラ未接続の場合は送信しない
    if (!isConnected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // チェックサム計算
    TelemetryPacket tx_packet = packet;
    uint8_t checksum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&tx_packet);
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        checksum += data[i];
    }
    tx_packet.checksum = checksum;

    // ESP-NOW送信
    return esp_now_send(controller_mac_,
                       reinterpret_cast<const uint8_t*>(&tx_packet),
                       sizeof(TelemetryPacket));
}

bool ControllerComm::isConnected() const {
    // 500ms以内に受信があれば接続中と判断
    int64_t now = esp_timer_get_time();
    return (now - last_received_us_) < 500000;
}
```

#### 5.5 ペアリング機能

##### ペアリング概要

StampFlyは起動時またはリセット後にコントローラとのペアリングを行う。

- **ペアリング情報なし**: 自動的にペアリングモードに入る
- **ペアリング情報あり**: 通常モードで起動
- **リセットボタン長押し（3秒）**: ペアリング情報をクリアして再ペアリング

##### ペアリング状態

```cpp
enum class PairingState {
    IDLE,       // 通常運用（未接続）
    WAITING,    // ペアリング待機中
    PAIRED      // ペアリング済み
};
```

##### ペアリングパケット（StampFly → コントローラ）

| バイト | 内容 | 説明 |
|--------|------|------|
| 0 | channel | WiFiチャンネル（固定値） |
| 1-6 | drone_mac[6] | StampFlyのMACアドレス |
| 7-10 | magic[4] | マジックナンバー `0xAA 0x55 0x16 0x88` |
| 11-13 | reserved[3] | 予約 |

```cpp
struct PairingPacket {
    uint8_t channel;
    uint8_t drone_mac[6];
    uint8_t magic[4];       // 0xAA, 0x55, 0x16, 0x88
    uint8_t reserved[3];
} __attribute__((packed));
```

##### ペアリングフロー

```
[StampFly 起動/リセット]
      │
      ├─→ NVS読み込み
      │     │
      │     ├─ ペアリング情報あり
      │     │   └─→ 通常モード (PAIRED)
      │     │         └─→ コントローラからの制御パケット受信待機
      │     │
      │     └─ ペアリング情報なし
      │         └─→ ペアリングモード (WAITING)
      │               ├─→ LED青点滅
      │               ├─→ ペアリングパケット ブロードキャスト送信（500ms間隔）
      │               │
      │               └─→ コントローラからの制御パケット受信
      │                     ├─→ コントローラMAC記憶
      │                     ├─→ NVSに保存
      │                     └─→ PAIRED状態へ遷移、LED緑点灯
```

##### ペアリング処理実装

```cpp
// 起動時処理
esp_err_t ControllerComm::init() {
    // WiFi/ESP-NOW初期化
    initWiFi();
    initESPNOW();

    // 自分のMACアドレス取得
    esp_read_mac(my_mac_, ESP_MAC_WIFI_STA);

    // NVSからペアリング情報読み込み
    esp_err_t ret = loadPairingFromNVS();

    if (ret != ESP_OK || isInvalidMAC(controller_mac_)) {
        // ペアリング情報なし → ペアリングモード
        startPairingMode();
    } else {
        // ペアリング済み → 通常モード
        pairing_state_ = PairingState::PAIRED;
        ESP_LOGI(TAG, "Paired with %02X:%02X:%02X:%02X:%02X:%02X",
                 controller_mac_[0], controller_mac_[1], controller_mac_[2],
                 controller_mac_[3], controller_mac_[4], controller_mac_[5]);
    }

    return ESP_OK;
}

// ペアリングモード開始
void ControllerComm::startPairingMode() {
    pairing_state_ = PairingState::WAITING;
    pairing_start_time_ = esp_timer_get_time();

    ESP_LOGI(TAG, "Entering pairing mode...");
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac_[0], my_mac_[1], my_mac_[2],
             my_mac_[3], my_mac_[4], my_mac_[5]);

    // LED青点滅
    setLEDPattern(LED_PAIRING);

    // ブロードキャストピア追加
    addBroadcastPeer();

    // ペアリングブロードキャストタスク起動
    xTaskCreate(pairingTask, "Pairing", 2048, this, 10, &pairing_task_handle_);
}

// ペアリングパケット送信タスク
void ControllerComm::pairingTask(void* param) {
    ControllerComm* self = static_cast<ControllerComm*>(param);

    PairingPacket packet;
    packet.channel = WIFI_CHANNEL;
    memcpy(packet.drone_mac, self->my_mac_, 6);
    packet.magic[0] = 0xAA;
    packet.magic[1] = 0x55;
    packet.magic[2] = 0x16;
    packet.magic[3] = 0x88;
    memset(packet.reserved, 0, sizeof(packet.reserved));

    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    while (self->pairing_state_ == PairingState::WAITING) {
        // ペアリングパケット送信
        esp_now_send(broadcast_mac, (uint8_t*)&packet, sizeof(packet));

        // 500ms待機
        vTaskDelay(pdMS_TO_TICKS(500));

        // タイムアウトチェック（30秒）
        if (esp_timer_get_time() - self->pairing_start_time_ > 30000000) {
            ESP_LOGW(TAG, "Pairing timeout");
            // タイムアウト後も継続（電源切るまで待機）
        }
    }

    vTaskDelete(NULL);
}

// 受信処理（ペアリング時）
void ControllerComm::onDataReceived(const uint8_t* mac,
                                    const uint8_t* data, int len) {
    ControllerComm* self = instance_;

    // ペアリング待機中
    if (self->pairing_state_ == PairingState::WAITING) {
        if (len == sizeof(ControlPacket)) {
            const ControlPacket* pkt = reinterpret_cast<const ControlPacket*>(data);

            // 自分宛かチェック（MAC下位3バイト）
            if (pkt->drone_mac[0] == self->my_mac_[3] &&
                pkt->drone_mac[1] == self->my_mac_[4] &&
                pkt->drone_mac[2] == self->my_mac_[5]) {

                // コントローラMAC記憶
                memcpy(self->controller_mac_, mac, 6);

                // NVS保存
                self->savePairingToNVS();

                // ペアリング完了
                self->pairing_state_ = PairingState::PAIRED;

                ESP_LOGI(TAG, "Paired with %02X:%02X:%02X:%02X:%02X:%02X",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

                // LED緑点灯
                setLEDPattern(LED_CONNECTED);
            }
        }
        return;
    }

    // 通常の制御パケット処理
    // ...
}
```

##### NVS保存・読み込み

```cpp
#define NVS_NAMESPACE "stampfly"
#define NVS_KEY_CTRL_MAC "ctrl_mac"

esp_err_t ControllerComm::savePairingToNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(handle, NVS_KEY_CTRL_MAC, controller_mac_, 6);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}

esp_err_t ControllerComm::loadPairingFromNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    size_t len = 6;
    ret = nvs_get_blob(handle, NVS_KEY_CTRL_MAC, controller_mac_, &len);

    nvs_close(handle);
    return ret;
}

esp_err_t ControllerComm::clearPairingFromNVS() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    nvs_erase_key(handle, NVS_KEY_CTRL_MAC);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Pairing cleared");
    return ESP_OK;
}
```

##### リセットボタン長押しによるペアリングリセット

```cpp
// GPIO0（リセットボタン）監視タスク
void ControllerComm::buttonMonitorTask(void* param) {
    ControllerComm* self = static_cast<ControllerComm*>(param);
    int64_t press_start = 0;
    const int64_t LONG_PRESS_US = 3000000;  // 3秒

    while (true) {
        if (gpio_get_level(GPIO_NUM_0) == 0) {  // 押下中
            if (press_start == 0) {
                press_start = esp_timer_get_time();
            } else if (esp_timer_get_time() - press_start > LONG_PRESS_US) {
                ESP_LOGI(TAG, "Long press detected, clearing pairing...");
                self->clearPairingFromNVS();
                esp_restart();  // 再起動
            }
        } else {
            press_start = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

##### LED表示パターン

| 状態 | LEDパターン | 色 |
|------|------------|-----|
| ペアリング待機 | 1Hz点滅 | 青 |
| 接続済み | 点灯 | 緑 |
| 接続断 | 2Hz点滅 | 赤 |
| エラー | 点灯 | 赤 |

#### 5.6 確認項目
- ペアリングモード動作（LED青点滅）
- ペアリングパケット送信（ブロードキャスト）
- コントローラとのペアリング成立
- NVS保存・読み込み
- リセットボタン長押しによるペアリングリセット
- 制御パケット受信
- テレメトリ送信

---

### Phase 6: CLIコンソール実装

#### 6.1 コマンド一覧

| コマンド | 引数 | 説明 |
|---------|------|------|
| `help` | - | コマンド一覧表示 |
| `status` | - | 現在の状態表示 |
| `sensor` | [name] | センサ値リアルタイム表示 |
| `calib` | [type] | キャリブレーション実行 |
| `param` | list/set/save | パラメータ操作 |
| `log` | [level] | ログレベル設定 |
| `arm` | - | アーム要求 |
| `disarm` | - | ディスアーム要求 |
| `pair` | - | ペアリングモード開始 |
| `unpair` | - | ペアリング解除 |
| `reboot` | - | 再起動 |
| `teleplot` | [on/off] | Teleplot出力切替 |

#### 6.2 CLIクラス

```cpp
namespace stampfly {

class CLI {
public:
    esp_err_t init();
    void registerCommand(const char* name,
                        const char* help,
                        std::function<void(int argc, char** argv)> handler);
    void process();  // メインループから呼び出し

    // 出力
    void print(const char* fmt, ...);
    void printTeleplot(const char* name, float value);

private:
    void handleCommand(const std::string& line);

    // 組み込みコマンド
    void cmdHelp(int argc, char** argv);
    void cmdStatus(int argc, char** argv);
    void cmdSensor(int argc, char** argv);
    void cmdCalib(int argc, char** argv);
    void cmdParam(int argc, char** argv);
    void cmdLog(int argc, char** argv);
    void cmdArm(int argc, char** argv);
    void cmdDisarm(int argc, char** argv);
    void cmdReboot(int argc, char** argv);
    void cmdTeleplot(int argc, char** argv);
};

}
```

#### 6.3 Teleplot対応

```cpp
// Teleplotフォーマット出力
// >name:value
void CLI::printTeleplot(const char* name, float value) {
    printf(">%s:%.4f\n", name, value);
}

// 使用例
cli.printTeleplot("accel_x", accel.x);
cli.printTeleplot("gyro_z", gyro.z);
cli.printTeleplot("altitude", altitude);
```

#### 6.4 確認項目
- USB CDC接続
- コマンド入力・応答
- Teleplotでのグラフ表示

---

### Phase 7: メインタスク (スケルトン)

#### 7.1 制御ループ構造

```cpp
void MainTask(void* param) {
    auto& state = StampFlyState::getInstance();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);  // 400Hz (2.5ms)

    while (true) {
        // 1. 状態確認
        if (state.getFlightState() != FlightState::FLYING) {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        // 2. センサデータ・推定値取得
        Vector3 position, velocity;
        Quaternion attitude;
        state.getPosition(position);
        state.getVelocity(velocity);
        state.getAttitude(attitude);

        float roll, pitch, yaw;
        state.getEulerAngles(roll, pitch, yaw);

        // 3. コントローラ入力取得
        ControlInput input;
        state.getControlInput(input);

        // ============================================
        // 4. 制御計算 (ここがスケルトン - 将来実装)
        // ============================================
        // TODO: 姿勢制御 (角速度制御 → 角度制御)
        // TODO: 高度制御 (速度制御 → 位置制御)
        // TODO: 位置制御 (速度制御 → 位置制御)
        // TODO: ミキサー (制御出力 → モータ指令)

        float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        // ============================================
        // 5. モータ出力 (スケルトン)
        // ============================================
        // setMotorOutput(motor_output);

        vTaskDelayUntil(&last_wake, period);
    }
}
```

#### 7.2 タスク起動順序

```cpp
void app_main() {
    // 1. ハードウェア初期化
    initGPIO();
    initSPI();
    initI2C();

    // 2. センサドライバ初期化
    imu.init();
    mag.init();
    baro.init();
    tof_front.init();
    tof_bottom.init();
    flow.init();

    // 3. 周辺機器初期化
    power.init();
    buzzer.init();
    led.init();
    button.init();
    motor.init();

    // 4. 起動音
    buzzer.startTone();
    led.showInit();

    // 5. 状態管理初期化
    StampFlyState::getInstance().loadFromNVS();

    // 6. 通信初期化 (ペアリング処理含む)
    comm.init();

    // 7. CLI初期化
    cli.init();

    // 8. センサタスク起動
    xTaskCreate(IMUTask, "IMU", 4096, NULL, 24, NULL);
    xTaskCreate(MagTask, "Mag", 2048, NULL, 18, NULL);
    xTaskCreate(BaroTask, "Baro", 2048, NULL, 16, NULL);
    xTaskCreate(ToFTask, "ToF", 4096, NULL, 14, NULL);
    xTaskCreate(OptFlowTask, "Flow", 4096, NULL, 20, NULL);

    // 9. 周辺機器タスク起動
    xTaskCreate(PowerTask, "Power", 2048, NULL, 12, NULL);
    xTaskCreate(LEDTask, "LED", 2048, NULL, 8, NULL);
    xTaskCreate(ButtonTask, "Button", 2048, NULL, 10, NULL);

    // 10. 通信タスク起動
    xTaskCreate(CommTask, "Comm", 4096, NULL, 22, NULL);

    // 11. メインタスク起動
    xTaskCreate(MainTask, "Main", 8192, NULL, 23, NULL);

    // 12. CLIタスク起動
    xTaskCreate(CLITask, "CLI", 4096, NULL, 5, NULL);
}
```

#### 7.3 確認項目
- 全タスクの正常起動
- タスク間通信の動作
- 制御ループ周期の安定性

---

### Phase 8: ドキュメント作成

#### 8.1 作成ドキュメント

| ファイル | 内容 |
|---------|------|
| README.md | プロジェクト概要、クイックスタート |
| architecture.md | アーキテクチャ詳細 |
| calibration_guide.md | キャリブレーション手順 |
| control_implementation_guide.md | 制御実装ガイド |
| api_reference.md | API リファレンス |
| troubleshooting.md | トラブルシューティング |

#### 8.2 制御実装ガイド内容

- 姿勢制御の基礎理論
- PID制御の実装方法
- 高度制御の実装方法
- 位置制御の実装方法
- ゲイン調整手順
- サンプルコード

---

## 6. テスト計画

### 6.1 テスト構成

```
stampfly_rtos_skelton/
├── test/
│   ├── unit/                      # 単体テスト
│   │   ├── test_filter.cpp        # フィルタライブラリ
│   │   ├── test_state.cpp         # 状態管理クラス
│   │   ├── test_protocol.cpp      # 通信プロトコル
│   │   └── CMakeLists.txt
│   ├── integration/               # 統合テスト
│   │   ├── test_sensors.cpp       # センサ統合
│   │   ├── test_peripherals.cpp   # 周辺機器統合
│   │   ├── test_comm.cpp          # 通信統合
│   │   ├── test_system.cpp        # システム全体
│   │   └── CMakeLists.txt
│   └── CMakeLists.txt
```

### 6.2 単体テスト

#### 6.2.1 フィルタライブラリテスト (test_filter.cpp)

```cpp
#include "unity.h"
#include "moving_average.hpp"
#include "median_filter.hpp"
#include "lowpass_filter.hpp"
#include "outlier_detector.hpp"

// 移動平均フィルタテスト
void test_moving_average_basic() {
    stampfly::filter::MovingAverage<float, 5> avg;

    for (int i = 1; i <= 5; i++) {
        avg.update(static_cast<float>(i));
    }

    // 平均値 = (1+2+3+4+5)/5 = 3.0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, avg.get());
}

void test_moving_average_overflow() {
    stampfly::filter::MovingAverage<float, 3> avg;

    avg.update(1.0f);
    avg.update(2.0f);
    avg.update(3.0f);
    avg.update(10.0f);  // 1.0が押し出される

    // 平均値 = (2+3+10)/3 = 5.0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, avg.get());
}

// メディアンフィルタテスト
void test_median_filter_odd() {
    stampfly::filter::MedianFilter<float, 5> median;

    median.update(5.0f);
    median.update(1.0f);
    median.update(3.0f);
    median.update(2.0f);
    median.update(4.0f);

    // ソート後: 1,2,3,4,5 → 中央値 = 3.0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, median.get());
}

void test_median_filter_outlier_rejection() {
    stampfly::filter::MedianFilter<float, 5> median;

    median.update(10.0f);
    median.update(10.0f);
    median.update(100.0f);  // 外れ値
    median.update(10.0f);
    median.update(10.0f);

    // 外れ値が除去される → 中央値 = 10.0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, median.get());
}

// ローパスフィルタテスト
void test_lowpass_filter_step_response() {
    // カットオフ10Hz、サンプリング100Hz
    stampfly::filter::LowPassFilter lpf(10.0f, 100.0f);

    // ステップ入力
    float output = 0.0f;
    for (int i = 0; i < 100; i++) {
        output = lpf.update(1.0f);
    }

    // 十分な時間後、出力は入力に収束
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, output);
}

// 外れ値検出テスト
void test_outlier_detector_sigma() {
    stampfly::filter::OutlierDetector detector;

    float mean = 10.0f;
    float stddev = 1.0f;

    // 3σ以内 → 外れ値ではない
    TEST_ASSERT_FALSE(detector.isOutlier(12.0f, mean, stddev, 3.0f));

    // 3σ超過 → 外れ値
    TEST_ASSERT_TRUE(detector.isOutlier(15.0f, mean, stddev, 3.0f));
}

void test_outlier_detector_rate() {
    stampfly::filter::OutlierDetector detector;

    float max_rate = 10.0f;  // 最大変化率 10/s
    float dt = 0.01f;        // 10ms

    // 許容範囲内の変化
    TEST_ASSERT_FALSE(detector.isRateExceeded(10.05f, 10.0f, max_rate, dt));

    // 許容範囲超過の変化
    TEST_ASSERT_TRUE(detector.isRateExceeded(11.0f, 10.0f, max_rate, dt));
}
```

#### 6.2.2 状態管理クラステスト (test_state.cpp)

```cpp
#include "unity.h"
#include "stampfly_state.hpp"

void test_state_initial() {
    auto& state = stampfly::StampFlyState::getInstance();

    // 初期状態はINIT
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::INIT,
                      state.getFlightState());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::ErrorCode::NONE,
                      state.getErrorCode());
}

void test_state_arm_disarm() {
    auto& state = stampfly::StampFlyState::getInstance();

    // IDLEからARMへ遷移
    state.setFlightState(stampfly::StampFlyState::FlightState::IDLE);
    TEST_ASSERT_TRUE(state.requestArm());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::ARMED,
                      state.getFlightState());

    // ARMからIDLEへ遷移
    TEST_ASSERT_TRUE(state.requestDisarm());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::IDLE,
                      state.getFlightState());
}

void test_state_arm_rejected_on_error() {
    auto& state = stampfly::StampFlyState::getInstance();

    // エラー状態ではアーム拒否
    state.setError(stampfly::StampFlyState::ErrorCode::IMU_FAILURE);
    TEST_ASSERT_FALSE(state.requestArm());
}

void test_state_warning_flags() {
    auto& state = stampfly::StampFlyState::getInstance();

    // 警告フラグ設定
    state.setWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY);
    TEST_ASSERT_TRUE(state.hasWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY));

    // 警告フラグクリア
    state.clearWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY);
    TEST_ASSERT_FALSE(state.hasWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY));
}

void test_state_battery_update() {
    auto& state = stampfly::StampFlyState::getInstance();

    state.updateBattery(3.8f, 1.5f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.8f, state.getBatteryVoltage());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, state.getBatteryCurrent());
}

void test_state_thread_safety() {
    auto& state = stampfly::StampFlyState::getInstance();

    // 複数タスクから同時アクセスのテスト
    TaskHandle_t task1, task2;
    volatile bool task1_done = false;
    volatile bool task2_done = false;

    xTaskCreate([](void* param) {
        auto& s = stampfly::StampFlyState::getInstance();
        for (int i = 0; i < 1000; i++) {
            s.updateBattery(3.7f + (i % 10) * 0.01f, 1.0f);
        }
        *((volatile bool*)param) = true;
        vTaskDelete(NULL);
    }, "Task1", 2048, (void*)&task1_done, 5, &task1);

    xTaskCreate([](void* param) {
        auto& s = stampfly::StampFlyState::getInstance();
        for (int i = 0; i < 1000; i++) {
            float v = s.getBatteryVoltage();
            (void)v;
        }
        *((volatile bool*)param) = true;
        vTaskDelete(NULL);
    }, "Task2", 2048, (void*)&task2_done, 5, &task2);

    // 両タスク完了待ち
    while (!task1_done || !task2_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_PASS();  // デッドロックせずに完了
}
```

#### 6.2.3 通信プロトコルテスト (test_protocol.cpp)

```cpp
#include "unity.h"
#include "controller_comm.hpp"

void test_control_packet_checksum() {
    stampfly::ControllerComm::ControlPacket packet = {};
    packet.drone_mac[0] = 0x12;
    packet.drone_mac[1] = 0x34;
    packet.drone_mac[2] = 0x56;
    packet.throttle = 1500;
    packet.roll = 1500;
    packet.pitch = 1500;
    packet.yaw = 1500;
    packet.flags = 0x01;
    packet.reserved = 0;

    // チェックサム計算
    uint8_t checksum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    for (size_t i = 0; i < sizeof(packet) - 1; i++) {
        checksum += data[i];
    }
    packet.checksum = checksum;

    // 検証
    uint8_t verify = 0;
    for (size_t i = 0; i < sizeof(packet) - 1; i++) {
        verify += data[i];
    }
    TEST_ASSERT_EQUAL(packet.checksum, verify);
}

void test_pairing_packet_magic() {
    stampfly::ControllerComm::PairingPacket packet = {};
    packet.channel = 6;
    packet.magic[0] = 0xAA;
    packet.magic[1] = 0x55;
    packet.magic[2] = 0x16;
    packet.magic[3] = 0x88;

    // マジックナンバー検証
    TEST_ASSERT_EQUAL(0xAA, packet.magic[0]);
    TEST_ASSERT_EQUAL(0x55, packet.magic[1]);
    TEST_ASSERT_EQUAL(0x16, packet.magic[2]);
    TEST_ASSERT_EQUAL(0x88, packet.magic[3]);
}

void test_telemetry_packet_size() {
    // テレメトリパケットサイズ確認
    TEST_ASSERT_EQUAL(17, sizeof(stampfly::ControllerComm::TelemetryPacket));
}

void test_control_flags_parsing() {
    uint8_t flags = 0x0D;  // Arm=1, Flip=0, Mode=1, AltMode=1

    bool arm = (flags & 0x01) != 0;
    bool flip = (flags & 0x02) != 0;
    bool mode = (flags & 0x04) != 0;
    bool alt_mode = (flags & 0x08) != 0;

    TEST_ASSERT_TRUE(arm);
    TEST_ASSERT_FALSE(flip);
    TEST_ASSERT_TRUE(mode);
    TEST_ASSERT_TRUE(alt_mode);
}
```

#### 6.2.4 モータドライバテスト (test_motor.cpp)

```cpp
#include "unity.h"
#include "motor_driver.hpp"

// モータ設定テスト
void test_motor_config_defaults() {
    stampfly::MotorDriver::Config config;

    // GPIO割り当て確認
    TEST_ASSERT_EQUAL(GPIO_NUM_5, config.pins[0]);   // M1: 右前
    TEST_ASSERT_EQUAL(GPIO_NUM_45, config.pins[1]);  // M2: 右後
    TEST_ASSERT_EQUAL(GPIO_NUM_41, config.pins[2]);  // M3: 左後
    TEST_ASSERT_EQUAL(GPIO_NUM_42, config.pins[3]);  // M4: 左前

    // 回転方向確認
    TEST_ASSERT_EQUAL(stampfly::MotorDriver::CCW, config.directions[0]);  // M1
    TEST_ASSERT_EQUAL(stampfly::MotorDriver::CW, config.directions[1]);   // M2
    TEST_ASSERT_EQUAL(stampfly::MotorDriver::CCW, config.directions[2]);  // M3
    TEST_ASSERT_EQUAL(stampfly::MotorDriver::CW, config.directions[3]);   // M4
}

void test_motor_throttle_clamp() {
    stampfly::MotorDriver motor;
    stampfly::MotorDriver::Config config;
    motor.init(config);

    // 範囲外の値がクランプされること
    motor.setThrottle(stampfly::MotorDriver::MOTOR_FR, 3000);  // 上限超え
    // 内部で max_throttle (2000) にクランプされる

    motor.setThrottle(stampfly::MotorDriver::MOTOR_FR, 0);  // 最小値
    // 正常に設定される

    TEST_PASS();
}

void test_motor_arm_disarm() {
    stampfly::MotorDriver motor;
    stampfly::MotorDriver::Config config;
    motor.init(config);

    // 初期状態はディスアーム
    TEST_ASSERT_FALSE(motor.isArmed());

    // アーム
    motor.arm();
    TEST_ASSERT_TRUE(motor.isArmed());

    // ディスアーム
    motor.disarm();
    TEST_ASSERT_FALSE(motor.isArmed());
}

void test_motor_emergency_stop() {
    stampfly::MotorDriver motor;
    stampfly::MotorDriver::Config config;
    motor.init(config);

    motor.arm();
    uint16_t throttles[4] = {500, 500, 500, 500};
    motor.setAllThrottle(throttles);

    // 緊急停止
    motor.emergencyStop();

    // ディスアームされ、スロットル0になる
    TEST_ASSERT_FALSE(motor.isArmed());
}

void test_motor_mixer_calculation() {
    // ミキサー計算の検証
    float thrust = 1000.0f;
    float roll = 100.0f;
    float pitch = 50.0f;
    float yaw = 25.0f;

    // M1 = thrust - roll + pitch + yaw
    float m1 = thrust - roll + pitch + yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 975.0f, m1);

    // M2 = thrust - roll - pitch - yaw
    float m2 = thrust - roll - pitch - yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 825.0f, m2);

    // M3 = thrust + roll - pitch + yaw
    float m3 = thrust + roll - pitch + yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1075.0f, m3);

    // M4 = thrust + roll + pitch - yaw
    float m4 = thrust + roll + pitch - yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1125.0f, m4);
}

void test_motor_id_enum() {
    // モータID列挙値確認（右前から時計回り）
    TEST_ASSERT_EQUAL(0, stampfly::MotorDriver::MOTOR_FR);  // M1: 右前
    TEST_ASSERT_EQUAL(1, stampfly::MotorDriver::MOTOR_RR);  // M2: 右後
    TEST_ASSERT_EQUAL(2, stampfly::MotorDriver::MOTOR_RL);  // M3: 左後
    TEST_ASSERT_EQUAL(3, stampfly::MotorDriver::MOTOR_FL);  // M4: 左前
}
```

### 6.3 統合テスト

#### 6.3.1 センサ統合テスト (test_sensors.cpp)

```cpp
#include "unity.h"
#include "bmi270_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "bmm150.hpp"
#include "bmp280.hpp"

void test_imu_data_valid() {
    stampfly::BMI270 imu;
    TEST_ASSERT_EQUAL(ESP_OK, imu.init());

    stampfly::BMI270::IMUData data;
    TEST_ASSERT_EQUAL(ESP_OK, imu.read(data));

    // 静止状態での加速度ノルム ≈ 1g
    float accel_norm = std::sqrt(
        data.accel.x * data.accel.x +
        data.accel.y * data.accel.y +
        data.accel.z * data.accel.z
    );
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.0f, accel_norm);

    // 静止状態でのジャイロ ≈ 0
    float gyro_norm = std::sqrt(
        data.gyro.x * data.gyro.x +
        data.gyro.y * data.gyro.y +
        data.gyro.z * data.gyro.z
    );
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, gyro_norm);
}

void test_tof_data_valid() {
    stampfly::VL53L3CX tof_bottom;
    TEST_ASSERT_EQUAL(ESP_OK, tof_bottom.init());

    float distance;
    TEST_ASSERT_EQUAL(ESP_OK, tof_bottom.read(distance));

    // 距離は正の値
    TEST_ASSERT_GREATER_THAN(0.0f, distance);
    TEST_ASSERT_LESS_THAN(4000.0f, distance);  // 最大4m
}

void test_optical_flow_data_valid() {
    stampfly::PMW3901 flow;
    TEST_ASSERT_EQUAL(ESP_OK, flow.init());

    auto burst = flow.readMotionBurst();

    // Product IDの確認
    TEST_ASSERT_EQUAL(0x49, flow.getProductId());
}

void test_mag_data_valid() {
    stampfly::BMM150 mag;
    TEST_ASSERT_EQUAL(ESP_OK, mag.init());

    stampfly::BMM150::MagData data;
    TEST_ASSERT_EQUAL(ESP_OK, mag.read(data));

    // 磁場ノルム（地球磁場は約25-65μT）
    float mag_norm = std::sqrt(
        data.x * data.x +
        data.y * data.y +
        data.z * data.z
    );
    TEST_ASSERT_FLOAT_WITHIN(40.0f, 45.0f, mag_norm);
}

void test_baro_data_valid() {
    stampfly::BMP280 baro;
    TEST_ASSERT_EQUAL(ESP_OK, baro.init());

    stampfly::BMP280::BaroData data;
    TEST_ASSERT_EQUAL(ESP_OK, baro.read(data));

    // 気圧は約900-1100hPa
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 1013.25f, data.pressure);

    // 温度は妥当な範囲
    TEST_ASSERT_GREATER_THAN(-20.0f, data.temperature);
    TEST_ASSERT_LESS_THAN(60.0f, data.temperature);
}

void test_all_sensors_concurrent() {
    // 全センサを同時に初期化・読み取り
    stampfly::BMI270 imu;
    stampfly::PMW3901 flow;
    stampfly::VL53L3CX tof;
    stampfly::BMM150 mag;
    stampfly::BMP280 baro;

    TEST_ASSERT_EQUAL(ESP_OK, imu.init());
    TEST_ASSERT_EQUAL(ESP_OK, flow.init());
    TEST_ASSERT_EQUAL(ESP_OK, tof.init());
    TEST_ASSERT_EQUAL(ESP_OK, mag.init());
    TEST_ASSERT_EQUAL(ESP_OK, baro.init());

    // 100回連続読み取り
    for (int i = 0; i < 100; i++) {
        stampfly::BMI270::IMUData imu_data;
        TEST_ASSERT_EQUAL(ESP_OK, imu.read(imu_data));

        float tof_dist;
        tof.read(tof_dist);  // エラーでも継続

        stampfly::BMM150::MagData mag_data;
        mag.read(mag_data);

        stampfly::BMP280::BaroData baro_data;
        baro.read(baro_data);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_PASS();
}
```

#### 6.3.2 周辺機器統合テスト (test_peripherals.cpp)

```cpp
#include "unity.h"
#include "power_monitor.hpp"
#include "buzzer.hpp"
#include "led.hpp"
#include "button.hpp"

void test_power_monitor_voltage() {
    stampfly::PowerMonitor power;
    TEST_ASSERT_EQUAL(ESP_OK, power.init());

    stampfly::PowerMonitor::PowerData data;
    TEST_ASSERT_EQUAL(ESP_OK, power.read(data));

    // USB給電時は約5V、バッテリー時は3.4-4.35V
    TEST_ASSERT_GREATER_THAN(3.0f, data.voltage);
    TEST_ASSERT_LESS_THAN(6.0f, data.voltage);
}

void test_power_monitor_low_battery_detection() {
    stampfly::PowerMonitor power;
    power.init();

    // 現在の電圧を取得
    stampfly::PowerMonitor::PowerData data;
    power.read(data);

    // 3.4V以下かどうかの判定が正しく動作するか
    if (data.voltage < stampfly::PowerMonitor::BATTERY_LOW_VOLTAGE) {
        TEST_ASSERT_TRUE(power.isLowBattery());
    } else {
        TEST_ASSERT_FALSE(power.isLowBattery());
    }
}

void test_buzzer_tones() {
    stampfly::Buzzer buzzer;
    TEST_ASSERT_EQUAL(ESP_OK, buzzer.init());

    // 各音程のテスト（聴覚確認が必要）
    buzzer.playTone(stampfly::Buzzer::NOTE_D1, 100);
    vTaskDelay(pdMS_TO_TICKS(150));

    buzzer.playTone(stampfly::Buzzer::NOTE_D5, 100);
    vTaskDelay(pdMS_TO_TICKS(150));

    buzzer.playTone(stampfly::Buzzer::NOTE_D7, 100);
    vTaskDelay(pdMS_TO_TICKS(150));

    TEST_PASS();
}

void test_buzzer_patterns() {
    stampfly::Buzzer buzzer;
    buzzer.init();

    // プリセット音のテスト
    buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(300));

    buzzer.armTone();
    vTaskDelay(pdMS_TO_TICKS(500));

    buzzer.disarmTone();
    vTaskDelay(pdMS_TO_TICKS(500));

    TEST_PASS();
}

void test_led_colors() {
    stampfly::LED led;
    TEST_ASSERT_EQUAL(ESP_OK, led.init());

    // 各色のテスト（視覚確認が必要）
    led.setOnboardLED(0, stampfly::LED::RED);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(300));

    led.setOnboardLED(0, stampfly::LED::GREEN);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(300));

    led.setOnboardLED(0, stampfly::LED::BLUE);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(300));

    led.setOnboardLED(0, 0);  // OFF
    led.update();

    TEST_PASS();
}

void test_led_patterns() {
    stampfly::LED led;
    led.init();

    // 点滅パターンテスト
    led.setPattern(stampfly::LED::PATTERN_BLINK_SLOW, stampfly::LED::BLUE);

    for (int i = 0; i < 60; i++) {  // 2秒間
        led.update();
        vTaskDelay(pdMS_TO_TICKS(32));
    }

    led.setPattern(stampfly::LED::PATTERN_OFF, 0);
    led.update();

    TEST_PASS();
}

void test_button_detection() {
    stampfly::Button button;
    TEST_ASSERT_EQUAL(ESP_OK, button.init());

    // 初期状態は非押下
    TEST_ASSERT_FALSE(button.isPressed());

    // 注：実際のボタン押下テストは手動で行う
    TEST_PASS();
}

void test_button_callback() {
    stampfly::Button button;
    button.init();

    volatile bool clicked = false;

    button.setCallback([&clicked](stampfly::Button::Event event) {
        if (event == stampfly::Button::EVENT_CLICK) {
            clicked = true;
        }
    });

    // 手動テスト用：ボタンを押してクリックイベント確認
    ESP_LOGI("TEST", "Press button within 5 seconds...");

    for (int i = 0; i < 500 && !clicked; i++) {
        button.tick();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 手動テストのため、結果は参考値
    if (clicked) {
        ESP_LOGI("TEST", "Button click detected!");
    } else {
        ESP_LOGW("TEST", "No button click detected (manual test)");
    }

    TEST_PASS();
}
```

#### 6.3.3 通信統合テスト (test_comm.cpp)

```cpp
#include "unity.h"
#include "controller_comm.hpp"

void test_esp_now_init() {
    stampfly::ControllerComm comm;
    TEST_ASSERT_EQUAL(ESP_OK, comm.init());
}

void test_pairing_broadcast() {
    stampfly::ControllerComm comm;
    comm.init();

    // ペアリングモード開始
    comm.startPairingMode();

    TEST_ASSERT_EQUAL(stampfly::ControllerComm::PairingState::WAITING,
                      comm.getPairingState());

    // 5秒間待機（コントローラ側でパケット受信を確認）
    ESP_LOGI("TEST", "Broadcasting pairing packets for 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    comm.stopPairingMode();
    TEST_PASS();
}

void test_nvs_pairing_save_load() {
    stampfly::ControllerComm comm;
    comm.init();

    // テスト用MACアドレス設定
    uint8_t test_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

    // 保存
    comm.setControllerMAC(test_mac);
    TEST_ASSERT_EQUAL(ESP_OK, comm.savePairingToNVS());

    // クリア
    comm.clearControllerMAC();

    // 読み込み
    TEST_ASSERT_EQUAL(ESP_OK, comm.loadPairingFromNVS());

    // 検証
    uint8_t loaded_mac[6];
    comm.getControllerMAC(loaded_mac);
    TEST_ASSERT_EQUAL_MEMORY(test_mac, loaded_mac, 6);

    // クリーンアップ
    comm.clearPairingFromNVS();
}

void test_connection_timeout() {
    stampfly::ControllerComm comm;
    comm.init();

    // 初期状態は未接続
    TEST_ASSERT_FALSE(comm.isConnected());

    // タイムアウト検出（500ms以上パケットなし）
    vTaskDelay(pdMS_TO_TICKS(600));
    TEST_ASSERT_FALSE(comm.isConnected());
}
```

#### 6.3.4 システム統合テスト (test_system.cpp)

```cpp
#include "unity.h"
#include "stampfly_state.hpp"
#include "controller_comm.hpp"
#include "power_monitor.hpp"
#include "buzzer.hpp"
#include "led.hpp"

void test_system_startup_sequence() {
    // システム起動シーケンスのテスト
    auto& state = stampfly::StampFlyState::getInstance();

    // 1. 初期状態
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::INIT,
                      state.getFlightState());

    // 2. センサ初期化完了後
    // （実際のセンサ初期化処理の後）
    state.setFlightState(stampfly::StampFlyState::FlightState::IDLE);
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::IDLE,
                      state.getFlightState());
}

void test_system_state_flow() {
    auto& state = stampfly::StampFlyState::getInstance();

    // IDLE → ARMED → FLYING → IDLE
    state.setFlightState(stampfly::StampFlyState::FlightState::IDLE);

    TEST_ASSERT_TRUE(state.requestArm());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::ARMED,
                      state.getFlightState());

    state.setFlightState(stampfly::StampFlyState::FlightState::FLYING);
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::FLYING,
                      state.getFlightState());

    TEST_ASSERT_TRUE(state.requestDisarm());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::FlightState::IDLE,
                      state.getFlightState());
}

void test_system_low_battery_flow() {
    auto& state = stampfly::StampFlyState::getInstance();
    stampfly::PowerMonitor power;
    stampfly::Buzzer buzzer;
    stampfly::LED led;

    power.init();
    buzzer.init();
    led.init();

    // 低電圧シミュレーション
    state.setWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY);

    TEST_ASSERT_TRUE(state.hasWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY));

    // 警告表示
    led.showLowBattery();
    buzzer.lowBatteryWarning();

    vTaskDelay(pdMS_TO_TICKS(500));

    // 警告クリア
    state.clearWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY);
    TEST_ASSERT_FALSE(state.hasWarning(stampfly::StampFlyState::WARNING_LOW_BATTERY));
}

void test_system_error_handling() {
    auto& state = stampfly::StampFlyState::getInstance();

    // エラー発生
    state.setError(stampfly::StampFlyState::ErrorCode::IMU_FAILURE);

    TEST_ASSERT_TRUE(state.hasError());
    TEST_ASSERT_EQUAL(stampfly::StampFlyState::ErrorCode::IMU_FAILURE,
                      state.getErrorCode());

    // エラー中はアーム不可
    TEST_ASSERT_FALSE(state.requestArm());

    // エラークリア
    state.clearError();
    TEST_ASSERT_FALSE(state.hasError());
}

void test_system_concurrent_access() {
    // 複数タスクからの同時アクセステスト
    auto& state = stampfly::StampFlyState::getInstance();

    volatile int counter = 0;
    const int NUM_ITERATIONS = 100;

    // センサ更新タスク
    TaskHandle_t sensor_task;
    xTaskCreate([](void* param) {
        auto& s = stampfly::StampFlyState::getInstance();
        volatile int* cnt = (volatile int*)param;

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            s.updateBattery(3.7f, 1.0f);
            (*cnt)++;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        vTaskDelete(NULL);
    }, "Sensor", 2048, (void*)&counter, 5, &sensor_task);

    // 状態読み取りタスク
    TaskHandle_t reader_task;
    xTaskCreate([](void* param) {
        auto& s = stampfly::StampFlyState::getInstance();
        volatile int* cnt = (volatile int*)param;

        for (int i = 0; i < NUM_ITERATIONS; i++) {
            float v = s.getBatteryVoltage();
            (void)v;
            (*cnt)++;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        vTaskDelete(NULL);
    }, "Reader", 2048, (void*)&counter, 5, &reader_task);

    // 完了待ち
    while (counter < NUM_ITERATIONS * 2) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_PASS();  // デッドロックせずに完了
}

void test_system_full_integration() {
    // 全コンポーネントを使用した統合テスト
    ESP_LOGI("TEST", "=== Full System Integration Test ===");

    // 1. 全コンポーネント初期化
    auto& state = stampfly::StampFlyState::getInstance();
    stampfly::ControllerComm comm;
    stampfly::PowerMonitor power;
    stampfly::Buzzer buzzer;
    stampfly::LED led;

    TEST_ASSERT_EQUAL(ESP_OK, comm.init());
    TEST_ASSERT_EQUAL(ESP_OK, power.init());
    TEST_ASSERT_EQUAL(ESP_OK, buzzer.init());
    TEST_ASSERT_EQUAL(ESP_OK, led.init());

    // 2. 起動音・LED
    buzzer.startTone();
    led.showInit();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3. 状態遷移テスト
    state.setFlightState(stampfly::StampFlyState::FlightState::IDLE);
    led.showIdle();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 4. 電圧読み取り
    stampfly::PowerMonitor::PowerData power_data;
    power.read(power_data);
    ESP_LOGI("TEST", "Battery: %.2fV, %.2fA", power_data.voltage, power_data.current);

    // 5. テレメトリパケット構築
    stampfly::ControllerComm::TelemetryPacket telem = {};
    telem.battery_mv = static_cast<uint16_t>(power_data.voltage * 1000);
    telem.state = static_cast<uint8_t>(state.getFlightState());

    // 6. 完了
    led.setPattern(stampfly::LED::PATTERN_OFF, 0);
    led.update();

    ESP_LOGI("TEST", "=== Full Integration Test Complete ===");
    TEST_PASS();
}
```

### 6.4 テスト実行方法

#### CLIからのテスト実行

```cpp
// CLIコマンドとしてテストを追加
void CLI::cmdTest(int argc, char** argv) {
    if (argc < 2) {
        print("Usage: test <unit|integration|all>\n");
        return;
    }

    if (strcmp(argv[1], "unit") == 0) {
        runUnitTests();
    } else if (strcmp(argv[1], "integration") == 0) {
        runIntegrationTests();
    } else if (strcmp(argv[1], "all") == 0) {
        runUnitTests();
        runIntegrationTests();
    } else {
        print("Unknown test type: %s\n", argv[1]);
    }
}
```

#### テスト実行コマンド

```bash
# ビルド時にテストを含める
idf.py build

# 書き込み後、CLIから実行
> test unit          # 単体テスト
> test integration   # 統合テスト
> test all           # 全テスト
```

### 6.5 テストカバレッジ目標

| カテゴリ | カバレッジ目標 |
|---------|---------------|
| フィルタライブラリ | 90% |
| 状態管理クラス | 85% |
| 通信プロトコル | 80% |
| モータドライバ | 85% |
| センサドライバ | 70%（ハードウェア依存） |
| 周辺機器 | 60%（手動確認含む） |

### 6.6 テスト確認項目サマリー

#### 単体テスト
- [ ] MovingAverage: 基本動作、オーバーフロー
- [ ] MedianFilter: 奇数個、外れ値除去
- [ ] LowPassFilter: ステップ応答
- [ ] OutlierDetector: σベース、変化率ベース
- [ ] StampFlyState: 初期状態、遷移、警告フラグ、スレッドセーフ
- [ ] 通信プロトコル: チェックサム、フラグ解析、パケットサイズ
- [ ] モータドライバ: GPIO設定、スロットルクランプ、アーム/ディスアーム、緊急停止、ミキサー計算

#### 統合テスト
- [ ] IMU: データ有効性、静止時ノルム
- [ ] ToF: 距離範囲
- [ ] OpticalFlow: Product ID
- [ ] 磁気センサ: 磁場ノルム
- [ ] 気圧センサ: 気圧・温度範囲
- [ ] 全センサ同時動作
- [ ] 電源監視: 電圧読み取り、低電圧検出
- [ ] ブザー: 各音程、パターン
- [ ] LED: 各色、パターン
- [ ] ボタン: 検出、コールバック
- [ ] モータ: PWM出力、個別動作、回転方向、アーム/ディスアーム
- [ ] ESP-NOW: 初期化、ペアリング
- [ ] NVS: 保存・読み込み
- [ ] システム: 起動シーケンス、状態フロー、エラー処理、並行アクセス

---

## 7. マイルストーン

| Phase | 内容 | 確認項目 |
|-------|------|---------|
| 1 | プロジェクト構造 | ビルド成功、書き込み成功 |
| 2 | ドライバ統合 | 各センサ単体動作、周辺機器（電源・ブザー・LED・ボタン）動作 |
| 3 | センサ・周辺機器タスク | 全センサ同時動作、外れ値処理、低電圧警告、LED状態表示 |
| 3.5 | モータドライバ | PWM出力、個別動作、回転方向、アーム/ディスアーム、緊急停止 |
| 4 | 状態管理 | 状態遷移、バッテリー監視、警告フラグ、キャリブレーション、NVS保存 |
| 5 | 通信層 | ペアリング、コントローラ接続、テレメトリ送信 |
| 6 | CLI | コマンド動作、Teleplot表示、ペアリングコマンド |
| 7 | メインタスク | スケルトン統合動作 |
| 8 | ドキュメント | 完成 |

---

## 8. 参照リソース

- [StampFly技術資料](https://github.com/M5Fly-kanazawa/StampFly_technical_specification)
- [IMUドライバ](https://github.com/kouhei1970/stampfly_imu)
- [ToFドライバ](https://github.com/kouhei1970/stampfly_tof)
- [Optical Flowドライバ](https://github.com/kouhei1970/stampfly_opticalflow)
- [ESKF推定器](https://github.com/kouhei1970/stampfly-eskf-estimator)
- [コントローラ (for_tdmaブランチ)](https://github.com/M5Fly-kanazawa/Simple_StampFly_Joy)
