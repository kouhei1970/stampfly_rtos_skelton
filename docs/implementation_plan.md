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
├── stampfly_imu/           # BMI270 IMU ドライバ (C++ラッパ)
├── stampfly_tof/           # VL53L3CX ToF ドライバ (C++ラッパ)
├── stampfly_opticalflow/   # PMW3901 オプティカルフローセンサ (C++ラッパ)
├── stampfly_mag/           # BMM150 地磁気センサ (新規実装)
├── stampfly_baro/          # BMP280 気圧センサ (新規実装)
├── stampfly_power/         # INA3221 電源監視
├── stampfly_motor/         # モータードライバ (LEDC PWM)
├── stampfly_led/           # WS2812 RGB LED (RMT)
├── stampfly_buzzer/        # ブザー (LEDC PWM)
├── stampfly_button/        # ボタン制御
├── stampfly_filter/        # フィルタライブラリ
├── stampfly_eskf/          # 姿勢・高度・速度推定
├── stampfly_state/         # 状態管理・SystemManager
├── stampfly_comm/          # ESP-NOW コントローラ通信
└── stampfly_cli/           # CLI コンソール
```

---

## 5. 実装フェーズ

### Phase 1: プロジェクト基盤

- ESP-IDF v5.4 プロジェクト構成
- 全コンポーネントのスタブ作成
- CMakeLists.txt 設定
- GitHub リポジトリ作成

### Phase 2: ドライバ層実装

| コンポーネント | 内容 |
|---------------|------|
| stampfly_imu | BMI270 C++ラッパ、FIFO対応、1600Hz内部サンプリング |
| stampfly_tof | VL53L3CX C++ラッパ、デュアルセンサ対応 |
| stampfly_opticalflow | PMW3901 C++ラッパ、バーストリード対応 |
| stampfly_mag | BMM150 新規実装、I2C通信 |
| stampfly_baro | BMP280 新規実装、温度補正、高度計算 |
| stampfly_power | INA3221 電源監視、低電圧検出 (3.4V閾値) |

### Phase 3: サービス層実装

| コンポーネント | 内容 |
|---------------|------|
| stampfly_motor | LEDC PWM、X-quad構成ミキサー |
| stampfly_led | WS2812 RMT制御、状態表示パターン |
| stampfly_buzzer | LEDC PWM、プリセット音 |
| stampfly_button | GPIO0デバウンス、長押し検出 |
| stampfly_filter | LowPassFilter、MedianFilter、OutlierDetector |
| stampfly_eskf | 姿勢・高度・速度推定器 |
| stampfly_state | 状態管理、SystemManager |
| stampfly_comm | ESP-NOW通信、ペアリング機能 |
| stampfly_cli | USB CDCコンソール、コマンド処理 |

### Phase 4: タスク統合・テスト

- センサタスク統合
- 制御ループ統合
- 単体テスト
- 統合テスト

#### テスト計画

**単体テスト:**

| 対象 | テスト内容 | 確認項目 |
|------|----------|---------|
| BMI270Wrapper | 初期化・読み出し | デバイスID確認、データレンジ確認 |
| BMM150 | 初期化・読み出し | 磁場データ取得、範囲チェック |
| BMP280 | 初期化・読み出し | 気圧・温度データ、高度計算 |
| VL53L3CXWrapper | 初期化・距離取得 | 2センサ個別動作、XSHUT制御 |
| PMW3901Wrapper | 初期化・モーション取得 | SQUAL値確認、dX/dY取得 |
| PowerMonitor | 初期化・電圧取得 | 電圧・電流読み出し、低電圧検出 |
| MotorDriver | PWM出力 | 各モーター個別動作確認 |
| LED | パターン表示 | 各パターン・色の確認 |
| Buzzer | 音出力 | トーン生成、プリセット音 |
| LowPassFilter | 数値検証 | 既知入力に対する出力検証 |
| OutlierDetector | 閾値チェック | 境界値テスト |
| StampFlyState | 状態遷移 | 正常遷移・不正遷移テスト |

**統合テスト:**

| テスト名 | 内容 | 合格基準 |
|---------|------|---------|
| 全センサ同時動作 | 全センサタスク起動、データ取得 | 5分間エラーなし |
| ESP-NOW通信 | コントローラとの双方向通信 | パケットロス < 1% |
| タスク優先度検証 | 全タスク同時稼働時の遅延計測 | IMUタスク 2.5ms以内 |
| メモリリーク検出 | 長時間稼働 (1時間) | ヒープ使用量安定 |
| 状態遷移統合 | INIT→CALIBRATING→IDLE→ARMED→DISARM | 各遷移正常完了 |
| 低電圧シミュレーション | 3.4V以下検出 | 警告音・LED表示 |
| ペアリング機能 | ペアリング実行・NVS保存・復元 | 再起動後自動接続 |

**テスト手順:**
1. 各単体テスト用のテストプロジェクト作成 (`test/unit/`)
2. 統合テスト用シナリオ作成 (`test/integration/`)
3. 実機での動作確認チェックリスト作成

### Phase 5: ドキュメント作成

| ファイル名 | 内容 | 対象読者 |
|-----------|------|---------|
| `docs/api_reference.md` | 全クラスAPI、関数シグネチャ、使用例 | 開発者 |
| `docs/hardware_setup.md` | ハードウェア接続図、GPIO割り当て、センサ配置 | ハードウェア担当 |
| `docs/calibration_guide.md` | キャリブレーション手順（ジャイロ、加速度、地磁気） | ユーザー |
| `docs/control_implementation_guide.md` | 制御実装ガイド、スケルトン利用方法、タスク拡張方法 | 制御実装者 |

**control_implementation_guide.md の構成:**
1. スケルトン概要
2. 提供されるデータ（センサデータ、推定値）
3. ControlTaskの実装方法
4. StampFlyStateからのデータ取得方法
5. MotorDriverへの出力方法
6. 状態遷移との連携
7. サンプル制御コード（PID姿勢制御の例）

---

## 6. クラス設計

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

#### Button (GPIO0)
```cpp
class Button {
    esp_err_t init(const Config& config);
    void setCallback(EventCallback callback);
    void tick();  // デバウンス処理
    Event getLastEvent();
    // イベント: CLICK, DOUBLE_CLICK, LONG_PRESS, LONG_PRESS_START
};
```

**GPIO0ボタン機能:**

| 押下時間 | イベント | 動作 |
|---------|---------|------|
| 短押し (< 1秒) | CLICK | モード切替（将来用） |
| 長押し (3秒) | LONG_PRESS_3S | ペアリング情報クリア → 再ペアリングモード |
| 長押し (5秒) | LONG_PRESS_5S | システムリセット (esp_restart) |

### 6.3 フィルタ

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

#### IMU FIFOフィルタリング詳細

IMU FIFO読み出し後、推定器に入力する前にローパスフィルタを適用する：

| センサ | フィルタタイプ | カットオフ周波数 | 目的 |
|--------|---------------|-----------------|------|
| 加速度 | LowPassFilter | 50Hz | ノイズ除去、推定器入力用 |
| 角速度 | LowPassFilter | 100Hz | ノイズ除去、制御ループ用 |

```cpp
// IMUTask内での処理フロー
void IMUTask(void* pvParameters) {
    LowPassFilter accel_lpf[3];  // X, Y, Z
    LowPassFilter gyro_lpf[3];   // X, Y, Z

    // フィルタ初期化 (サンプリング周波数400Hz)
    for (int i = 0; i < 3; i++) {
        accel_lpf[i].init(400.0f, 50.0f);   // fs=400Hz, fc=50Hz
        gyro_lpf[i].init(400.0f, 100.0f);   // fs=400Hz, fc=100Hz
    }

    while (true) {
        // FIFO読み出し
        bmi270.readFIFO(fifo_buffer, max_frames, frames_read);

        for (size_t i = 0; i < frames_read; i++) {
            // フィルタ適用
            filtered_accel.x = accel_lpf[0].apply(fifo_buffer[i].accel.x);
            filtered_accel.y = accel_lpf[1].apply(fifo_buffer[i].accel.y);
            filtered_accel.z = accel_lpf[2].apply(fifo_buffer[i].accel.z);

            filtered_gyro.x = gyro_lpf[0].apply(fifo_buffer[i].gyro.x);
            filtered_gyro.y = gyro_lpf[1].apply(fifo_buffer[i].gyro.y);
            filtered_gyro.z = gyro_lpf[2].apply(fifo_buffer[i].gyro.z);

            // 推定器に入力
            attitude_estimator.update(filtered_accel, filtered_gyro, dt);
        }

        vTaskDelay(pdMS_TO_TICKS(2));  // 2.5ms周期 (400Hz)
    }
}
```

#### センサ外れ値処理詳細

各センサに対する外れ値検出と棄却の閾値：

| センサ | 検出条件 | 処理 |
|--------|---------|------|
| IMU (加速度) | ノルム < 0.5g または > 3.0g | 棄却、前回値保持 |
| IMU (角速度) | 絶対値 > 2000 dps | 棄却、前回値保持 |
| ToF | RangeStatus > 4 | 棄却、前回値保持 |
| ToF | SignalRate < 閾値 | 信頼度低下警告 |
| OpticalFlow | SQUAL < 30 | 棄却、前回値保持 |
| Mag | ノルム ±30%外（期待値比） | 棄却、前回値保持 |
| Baro | 変化率 > 10 m/s | 棄却、前回値保持 |

```cpp
// 外れ値検出の実装例
class OutlierDetector {
public:
    // IMU加速度チェック
    bool isAccelValid(const Vec3& accel) {
        float norm = std::sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
        return (norm >= 0.5f * 9.81f) && (norm <= 3.0f * 9.81f);
    }

    // IMU角速度チェック
    bool isGyroValid(const Vec3& gyro) {
        return (std::abs(gyro.x) <= 2000.0f) &&
               (std::abs(gyro.y) <= 2000.0f) &&
               (std::abs(gyro.z) <= 2000.0f);
    }

    // ToFチェック
    bool isToFValid(uint8_t range_status, float signal_rate) {
        return (range_status <= 4) && (signal_rate >= MIN_SIGNAL_RATE);
    }

    // OpticalFlowチェック
    bool isFlowValid(uint8_t squal) {
        return squal >= 30;
    }

    // 地磁気チェック (expected_norm: 地域の地磁気強度期待値)
    bool isMagValid(const Vec3& mag, float expected_norm) {
        float norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
        return (norm >= expected_norm * 0.7f) && (norm <= expected_norm * 1.3f);
    }

    // 気圧高度チェック
    bool isBaroValid(float current_altitude, float prev_altitude, float dt) {
        float rate = std::abs(current_altitude - prev_altitude) / dt;
        return rate <= 10.0f;  // 10 m/s以下
    }
};
```

### 6.4 推定器

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

### 6.5 通信

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

#### ControlPacket (14バイト)

コントローラ（for_tdmaブランチ）と同一の構造。WiFiチャンネルは1固定。

```cpp
struct ControlPacket {
    uint8_t drone_mac[3];   // byte 0-2: 宛先MAC下位3バイト
    uint16_t throttle;      // byte 3-4: 0-1000
    uint16_t roll;          // byte 5-6: 0-1000 (500=中央)
    uint16_t pitch;         // byte 7-8: 0-1000 (500=中央)
    uint16_t yaw;           // byte 9-10: 0-1000 (500=中央)
    uint8_t flags;          // byte 11: bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
    uint8_t reserved;       // byte 12: 予約（proactive_flag、無視）
    uint8_t checksum;       // byte 13: byte 0-12の単純合計
} __attribute__((packed));
```

**チェックサム計算:**
```cpp
uint8_t checksum = 0;
for (int i = 0; i < 13; i++) {
    checksum += data[i];
}
```

#### TelemetryPacket (22バイト)

StampFlyからコントローラへ送信するテレメトリパケット。将来のコントローラ改修時にはこの構造に合わせて受信側を実装する。

```cpp
struct TelemetryPacket {
    uint8_t header;           // byte 0: 0xAA (パケット識別子)
    uint8_t packet_type;      // byte 1: 0x01 = テレメトリ
    uint8_t sequence;         // byte 2: シーケンス番号 (0-255)
    uint16_t battery_mv;      // byte 3-4: バッテリー電圧 [mV]
    int16_t altitude_cm;      // byte 5-6: 高度 [cm]
    int16_t velocity_x;       // byte 7-8: X速度 [mm/s]
    int16_t velocity_y;       // byte 9-10: Y速度 [mm/s]
    int16_t velocity_z;       // byte 11-12: Z速度 [mm/s]
    int16_t roll_deg10;       // byte 13-14: ロール [0.1度]
    int16_t pitch_deg10;      // byte 15-16: ピッチ [0.1度]
    int16_t yaw_deg10;        // byte 17-18: ヨー [0.1度]
    uint8_t state;            // byte 19: FlightState
    uint8_t flags;            // byte 20: 警告フラグ
    uint8_t checksum;         // byte 21: byte 0-20の単純合計
} __attribute__((packed));    // 合計22バイト
```

**フラグビット定義:**
```cpp
enum TelemetryFlags : uint8_t {
    FLAG_LOW_BATTERY    = 0x01,  // bit0: 低電圧警告 (3.4V以下)
    FLAG_SENSOR_ERROR   = 0x02,  // bit1: センサエラー
    FLAG_COMM_LOST      = 0x04,  // bit2: 通信途絶警告
    FLAG_CALIBRATING    = 0x08,  // bit3: キャリブレーション中
    FLAG_GPS_FIX        = 0x10,  // bit4: GPS Fix (将来用)
    FLAG_RESERVED_5     = 0x20,  // bit5: 予約
    FLAG_RESERVED_6     = 0x40,  // bit6: 予約
    FLAG_RESERVED_7     = 0x80,  // bit7: 予約
};
```

**テレメトリ送信タイミング:** CommTask内で20ms (50Hz) 周期で送信

### 6.6 状態管理

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

### 6.7 CLI

```cpp
class CLI {
    esp_err_t init();
    void registerCommand(const char* name, CommandHandler handler);
    void processInput();
};
```

#### CLIコマンド一覧

USB_SERIAL (CDC) 経由でアクセス可能なコマンド：

| コマンド | 引数 | 説明 |
|---------|------|------|
| `sensor` | `[imu\|mag\|baro\|tof\|flow\|power]` | センサ値のリアルタイム表示 |
| `calib` | `[gyro\|accel\|mag]` | キャリブレーション実行 |
| `motor` | `test <id> <throttle>` | 指定モーターのテスト (id: 1-4, throttle: 0-100) |
| `motor` | `arm` | モーターアーム |
| `motor` | `disarm` | モーターディスアーム |
| `status` | - | システム状態表示 (FlightState, バッテリー, 通信状態) |
| `gain` | `<name> <value>` | ゲイン設定（将来の制御実装用） |
| `pair` | - | ペアリングモード開始 |
| `reset` | - | システムリセット (esp_restart) |
| `help` | - | ヘルプ表示 |

```cpp
// CLIコマンド実装例
void CLI::registerDefaultCommands() {
    registerCommand("sensor", [](int argc, char** argv) {
        if (argc < 2) {
            printf("Usage: sensor [imu|mag|baro|tof|flow|power]\n");
            return;
        }
        if (strcmp(argv[1], "imu") == 0) {
            // IMUデータ表示開始
            startSensorStream(SENSOR_IMU);
        }
        // ...
    });

    registerCommand("calib", [](int argc, char** argv) {
        if (argc < 2) {
            printf("Usage: calib [gyro|accel|mag]\n");
            return;
        }
        if (strcmp(argv[1], "gyro") == 0) {
            printf("Starting gyro calibration. Keep device still...\n");
            systemManager.runGyroCalibration();
        }
        // ...
    });

    registerCommand("motor", [](int argc, char** argv) {
        if (argc < 2) {
            printf("Usage: motor test <id> <throttle> | arm | disarm\n");
            return;
        }
        if (strcmp(argv[1], "test") == 0 && argc >= 4) {
            int id = atoi(argv[2]);
            int throttle = atoi(argv[3]);
            motorDriver.testMotor(id, throttle);
        }
        // ...
    });

    registerCommand("status", [](int argc, char** argv) {
        printf("FlightState: %s\n", getFlightStateName(state.getFlightState()));
        printf("Battery: %.2fV (%.0f%%)\n",
               powerMonitor.getVoltage(),
               powerMonitor.getBatteryPercent());
        printf("Controller: %s\n", controllerComm.isConnected() ? "Connected" : "Disconnected");
    });

    registerCommand("pair", [](int argc, char** argv) {
        printf("Entering pairing mode...\n");
        controllerComm.enterPairingMode();
    });

    registerCommand("reset", [](int argc, char** argv) {
        printf("Resetting system...\n");
        esp_restart();
    });
}
```

---

## 7. タスク設計

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
