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
| フレームワーク | ESP-IDF v5.4 |
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
│  │     AttitudeEstimator / AltitudeEstimator / VelocityEstimator││
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
| IMU CS | 46 | BMI270 |
| Flow CS | 12 | PMW3901 |

#### I2Cバス
| ピン | GPIO | 接続先 |
|------|------|--------|
| SDA | 3 | BMM150, BMP280, VL53L3CX, INA3221 |
| SCL | 4 | BMM150, BMP280, VL53L3CX, INA3221 |

#### ToF XSHUT
| ピン | GPIO | 接続先 |
|------|------|--------|
| XSHUT (Bottom) | 7 | VL53L3CX (底面) |
| XSHUT (Front) | 9 | VL53L3CX (前方) |

#### モーター (LEDC PWM)
| モーター | GPIO | 回転方向 |
|----------|------|----------|
| M1 (FR) | 42 | CCW |
| M2 (RR) | 41 | CW |
| M3 (RL) | 10 | CCW |
| M4 (FL) | 5 | CW |

#### 周辺機器
| デバイス | GPIO |
|----------|------|
| LED (WS2812) | 39 |
| Buzzer | 40 |
| Button | 0 |

---

## 4. コンポーネント構成

```
components/
├── stampfly_imu/           # BMI270 IMU ドライバ [完了]
├── stampfly_tof/           # VL53L3CX ToF ドライバ [完了]
├── stampfly_opticalflow/   # PMW3901 オプティカルフローセンサ [完了]
├── stampfly_mag/           # BMM150 地磁気センサ [完了]
├── stampfly_baro/          # BMP280 気圧センサ [完了]
├── stampfly_power/         # INA3221 電源監視 [完了]
├── stampfly_motor/         # モータードライバ (LEDC PWM) [完了]
├── stampfly_led/           # WS2812 RGB LED (RMT) [完了]
├── stampfly_buzzer/        # ブザー (LEDC PWM) [完了]
├── stampfly_button/        # ボタン制御 [スタブ]
├── stampfly_filter/        # フィルタライブラリ [スタブ]
├── stampfly_eskf/          # 姿勢・高度・速度推定 [完了]
├── stampfly_state/         # 状態管理・SystemManager [完了]
├── stampfly_comm/          # ESP-NOW コントローラ通信 [完了]
└── stampfly_cli/           # CLI コンソール [スタブ]
```

---

## 5. 実装フェーズ

### Phase 1: プロジェクト基盤 [完了]

- ESP-IDF v5.4 プロジェクト構成
- 全コンポーネントのスタブ作成
- CMakeLists.txt 設定
- GitHub リポジトリ作成

### Phase 2: ドライバ層実装 [完了]

実装済みドライバ:

| コンポーネント | ファイル | 行数 | 状態 |
|---------------|---------|------|------|
| stampfly_imu | bmi270_wrapper.cpp | 407 | 完了 |
| stampfly_tof | vl53l3cx_wrapper.cpp | 355 | 完了 |
| stampfly_opticalflow | pmw3901_wrapper.cpp | 344 | 完了 |
| stampfly_mag | bmm150.cpp | 275 | 完了 |
| stampfly_baro | bmp280.cpp | 254 | 完了 |
| stampfly_power | power_monitor.cpp | 210 | 完了 |

### Phase 3: サービス層実装 [完了]

実装済みサービス:

| コンポーネント | ファイル | 行数 | 状態 |
|---------------|---------|------|------|
| stampfly_led | led.cpp | 424 | 完了 |
| stampfly_buzzer | buzzer.cpp | 253 | 完了 |
| stampfly_motor | motor_driver.cpp | 268 | 完了 |
| stampfly_eskf | eskf.cpp | 472 | 完了 |
| stampfly_state | stampfly_state.cpp | 415 | 完了 |
| stampfly_comm | controller_comm.cpp | 454 | 完了 |

### Phase 4: 残機能実装 [進行中]

未完了コンポーネント:

| コンポーネント | ファイル | 行数 | 状態 |
|---------------|---------|------|------|
| stampfly_button | button.cpp | 32 | スタブのみ |
| stampfly_filter | filter.cpp | 8 | スタブのみ |
| stampfly_cli | cli.cpp | 45 | スタブのみ |

### Phase 5: タスク統合・テスト [未着手]

- センサタスク統合
- 制御ループ統合
- 単体テスト
- 統合テスト

### Phase 6: ドキュメント作成 [未着手]

- API ドキュメント
- ユーザーガイド
- 制御実装ガイド

---

## 6. 実装済みクラス詳細

### 6.1 センサドライバ

#### BMI270Wrapper (IMU)
```cpp
class BMI270Wrapper {
    esp_err_t init(const Config& config);
    esp_err_t readSensorData(AccelData& accel, GyroData& gyro);
    esp_err_t readFIFO(FIFOData* buffer, size_t max_frames, size_t& frames_read);
    // SPI通信、FIFO対応、1600Hz内部サンプリング
};
```

#### VL53L3CXWrapper (ToF)
```cpp
class VL53L3CXWrapper {
    esp_err_t init(const Config& config);
    esp_err_t startRanging();
    esp_err_t getDistance(uint16_t& distance_mm, uint8_t& status);
    // デュアルセンサ対応(前方・底面)、XSHUT制御
};
```

#### PMW3901Wrapper (Optical Flow)
```cpp
class PMW3901Wrapper {
    esp_err_t init(const Config& config);
    esp_err_t readMotion(MotionData& data);
    // バーストリード対応、SQUAL品質チェック
};
```

#### BMM150 (地磁気)
```cpp
class BMM150 {
    esp_err_t init(const Config& config);
    esp_err_t read(MagData& data);
    // I2C通信、XYZ磁場データ
};
```

#### BMP280 (気圧)
```cpp
class BMP280 {
    esp_err_t init(const Config& config);
    esp_err_t read(BaroData& data);
    float calculateAltitude(float pressure_pa);
    // I2C通信、温度補正
};
```

#### PowerMonitor (INA3221)
```cpp
class PowerMonitor {
    esp_err_t init(const Config& config);
    esp_err_t read(PowerData& data);
    bool isLowBattery() const;
    float getBatteryPercent() const;
    // 3チャンネル対応、低電圧警告3.4V閾値
};
```

### 6.2 サービス層

#### LED (WS2812)
```cpp
class LED {
    esp_err_t init(const Config& config);
    void setColor(uint8_t index, uint32_t color);
    void setPattern(Pattern pattern, uint32_t color);
    // パターン: OFF, SOLID, BLINK_SLOW, BLINK_FAST, BREATHE, RAINBOW
    // 状態表示: showInit, showCalibrating, showIdle, showArmed, showFlying...
};
```

#### Buzzer (LEDC PWM)
```cpp
class Buzzer {
    esp_err_t init(const Config& config);
    void playTone(uint16_t frequency, uint32_t duration_ms);
    void playToneAsync(uint16_t frequency, uint32_t duration_ms);
    // 音階: NOTE_C4〜NOTE_C6
    // プリセット: beep, startTone, armTone, disarmTone, lowBatteryWarning...
};
```

#### MotorDriver (LEDC PWM)
```cpp
class MotorDriver {
    esp_err_t init(const Config& config);
    esp_err_t arm();
    esp_err_t disarm();
    void setMixerOutput(float thrust, float roll, float pitch, float yaw);
    // X-quad構成、150kHz PWM、8bit解像度
    // ミキサー: M1=T-R+P+Y, M2=T-R-P-Y, M3=T+R-P+Y, M4=T+R+P-Y
};
```

### 6.3 推定器

#### AttitudeEstimator (相補フィルタ)
```cpp
class AttitudeEstimator {
    esp_err_t init(const Config& config);
    void update(const Vec3& accel, const Vec3& gyro, float dt);
    void updateMag(const Vec3& mag);
    State getState() const;
    // クォータニオン姿勢、ジャイロバイアス推定
};
```

#### AltitudeEstimator (カルマンフィルタ)
```cpp
class AltitudeEstimator {
    esp_err_t init(const Config& config);
    void predict(float accel_z, float dt);
    void updateBaro(float altitude);
    void updateToF(float distance, float pitch, float roll);
    // 2状態KF [高度, 速度]
};
```

#### VelocityEstimator (光流)
```cpp
class VelocityEstimator {
    esp_err_t init(const Config& config);
    void updateFlow(float flow_x, float flow_y, float height, float gyro_x, float gyro_y);
    // ジャイロ補償付き速度推定
};
```

### 6.4 通信

#### ControllerComm (ESP-NOW)
```cpp
class ControllerComm {
    esp_err_t init(const Config& config);
    esp_err_t start();
    void setControlCallback(ControlCallback callback);
    esp_err_t sendTelemetry(const TelemetryPacket& packet);
    bool isConnected() const;
    void enterPairingMode();
    esp_err_t clearPairingFromNVS();
    // 制御パケット受信、テレメトリ送信、ペアリング機能
};
```

#### ControlPacket (12バイト)
```cpp
struct ControlPacket {
    uint8_t drone_mac[3];   // 宛先MAC下位3バイト
    uint16_t throttle;      // 0-1000
    uint16_t roll;          // 0-1000 (500=中央)
    uint16_t pitch;         // 0-1000 (500=中央)
    uint16_t yaw;           // 0-1000 (500=中央)
    uint8_t flags;          // bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
    uint8_t checksum;
};
```

### 6.5 状態管理

#### StampFlyState
```cpp
class StampFlyState {
    FlightState getFlightState() const;
    PairingState getPairingState() const;
    void getIMUData(Vector3& accel, Vector3& gyro) const;
    void updateIMU(const Vector3& accel, const Vector3& gyro);
    bool requestArm();
    bool requestDisarm();
    void setError(ErrorCode code);
    // スレッドセーフ、センサデータ管理、状態遷移
};

enum class FlightState {
    INIT, CALIBRATING, IDLE, ARMED, FLYING, LANDING, ERROR
};
```

#### SystemManager
```cpp
class SystemManager {
    esp_err_t init(const Config& config);
    esp_err_t start();
    bool waitForEvents(uint32_t events, uint32_t timeout_ms);
    void setEvent(uint32_t event);
    esp_err_t runCalibration();
    // イベントグループによる初期化同期
};
```

---

## 7. 残作業一覧

### 7.1 Button (スタブ → 完全実装)

```cpp
class Button {
    esp_err_t init(const Config& config);
    void setCallback(EventCallback callback);
    void tick();  // デバウンス処理
    Event getLastEvent();
    // イベント: CLICK, DOUBLE_CLICK, LONG_PRESS, LONG_PRESS_START
};
```

### 7.2 Filter (スタブ → 完全実装)

```cpp
// 移動平均フィルタ
template<typename T, size_t N>
class MovingAverage { ... };

// メディアンフィルタ
template<typename T, size_t N>
class MedianFilter { ... };

// 1次IIRローパスフィルタ
class LowPassFilter { ... };

// 外れ値検出
class OutlierDetector { ... };
```

### 7.3 CLI (スタブ → 完全実装)

```cpp
class CLI {
    esp_err_t init();
    void registerCommand(const char* name, CommandHandler handler);
    void processInput();
    // コマンド: sensor, calib, motor, status, help...
};
```

### 7.4 タスク統合

センサタスク設計:
| タスク名 | 周期 | 優先度 | スタック | 内容 |
|---------|------|--------|---------|------|
| IMUTask | 2.5ms (400Hz) | 24 | 4096 | BMI270 FIFO読み出し |
| OptFlowTask | 10ms (100Hz) | 20 | 4096 | PMW3901読み出し |
| MagTask | 10ms (100Hz) | 18 | 2048 | BMM150読み出し |
| BaroTask | 20ms (50Hz) | 16 | 2048 | BMP280読み出し |
| ToFTask | 33ms (30Hz) | 14 | 4096 | VL53L3CX読み出し |
| PowerTask | 100ms (10Hz) | 12 | 2048 | INA3221監視 |
| LEDTask | 32ms (30Hz) | 8 | 2048 | LEDパターン更新 |
| ButtonTask | 10ms (100Hz) | 10 | 2048 | ボタン監視 |
| CommTask | 20ms (50Hz) | 15 | 4096 | ESP-NOW通信 |

---

## 8. 参考リソース

- StampFly技術仕様: https://github.com/M5Fly-kanazawa/StampFly_technical_specification
- IMUドライバ: https://github.com/kouhei1970/stampfly_imu
- ToFドライバ: https://github.com/kouhei1970/stampfly_tof
- Optical Flowドライバ: https://github.com/kouhei1970/stampfly_opticalflow
- ESKF推定器: https://github.com/kouhei1970/stampfly-eskf-estimator
- コントローラ: https://github.com/M5Fly-kanazawa/Simple_StampFly_Joy (for_tdmaブランチ)
