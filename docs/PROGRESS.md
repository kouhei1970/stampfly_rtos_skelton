# StampFly RTOS Skeleton 実装進捗

## 最終更新: 2025-12-31 (テレメトリv2.1・地磁気表示・HTMLビューア改善)

---

## テレメトリv2.1・HTMLビューア改善 ✅ 完了 (2025-12-31)

### パケット構造拡張

TelemetryWSPacketを96バイトから108バイトに拡張し、地磁気データを追加。

**ファイル**: `components/stampfly_telemetry/include/telemetry.hpp`

```cpp
struct TelemetryWSPacket {
    // Header (2 bytes)
    uint8_t  header;          // 0xAA
    uint8_t  packet_type;     // 0x20 = extended packet (v2)
    uint32_t timestamp_ms;    // ms since boot

    // Attitude - ESKF estimated (12 bytes)
    float roll, pitch, yaw;   // [rad]

    // Position/Velocity - ESKF estimated (24 bytes)
    float pos_x, pos_y, pos_z;  // [m] NED
    float vel_x, vel_y, vel_z;  // [m/s]

    // IMU - bias corrected (24 bytes)
    float gyro_x, gyro_y, gyro_z;    // [rad/s]
    float accel_x, accel_y, accel_z; // [m/s²]

    // Control inputs (16 bytes)
    float ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw;

    // Magnetometer (12 bytes) [NEW v2.1]
    float mag_x, mag_y, mag_z;  // [uT]

    // System status (10 bytes)
    float voltage;
    uint8_t flight_state, sensor_status;
    uint32_t heartbeat;
    uint8_t checksum;
    uint8_t padding[3];
};
static_assert(sizeof(TelemetryWSPacket) == 108, "...");
```

### HTMLビューア改善

**ファイル**: `components/stampfly_telemetry/www/index.html`

#### 色の統一
- X軸系 = 赤 (#ff6b6b)
- Y軸系 = 緑 (#4edc4e)
- Z軸系 = 青 (#6b9fff)
- スロットル = オレンジ (#ffaa00)

#### グラフ改善
- オートフィット（対称スケール、ゼロ中央）
- デフォルト最小レンジ設定（データが小さい時の表示安定化）

| グラフ | 最小レンジ |
|--------|------------|
| Attitude | ±30 deg |
| Position | ±1 m |
| Velocity | ±0.5 m/s |
| Gyro | ±50 deg/s |
| Accel | ±12 m/s² |
| Magnetometer | ±50 uT |
| Control | ±1 |

#### 3カラムビューレイアウト
1. **姿勢指示器（Attitude Indicator）**: 地平線が回転するタイプ、ピッチラダー、ロール目盛り付き
2. **上面図（Top View）**: ドローンと座標軸が一緒に回転
3. **側面図（Side View）**: 原点が上下左右中央に配置

#### 地磁気表示追加
- データカード: X/Y/Z値をμT単位で表示
- グラフ: 時系列表示（±50μT デフォルト）
- CSVエクスポート: mag_x, mag_y, mag_z フィールド追加

### Z軸回転方向

Top Viewでの回転は正しく実装済み：

```
NED座標系: 正のヨー = 上から見て時計回り（右ネジ法則）
Canvas 2D: ctx.rotate(angle) = 正の角度で時計回り
→ ctx.rotate(currentState.yaw) で正しい
```

---

## 飛行制御基盤実装 ✅ 完了 (2025-12-29)

### ControlTask（飛行制御タスク）

400Hz で実行される飛行制御タスクを実装。IMUTaskとセマフォで同期。

**ファイル**: `main/main.cpp:569-634`

```cpp
static void ControlTask(void* pvParameters)
{
    while (true) {
        // Wait for control semaphore (given by IMU task after ESKF update)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Only run control when ARMED or FLYING
        if (flight_state != FlightState::ARMED && flight_state != FlightState::FLYING) {
            // Stop all motors
            continue;
        }

        // Get control input and apply to motors
        float throttle, roll, pitch, yaw;
        state.getControlInput(throttle, roll, pitch, yaw);

        // Simple throttle control (PID control is TODO)
        g_motor.setMotor(MotorDriver::MOTOR_FR, throttle);
        g_motor.setMotor(MotorDriver::MOTOR_RR, throttle);
        g_motor.setMotor(MotorDriver::MOTOR_RL, throttle);
        g_motor.setMotor(MotorDriver::MOTOR_FL, throttle);
    }
}
```

**モーターレイアウト図** (X-quad, 上から見た図):
```
           Front
      FL (M4)   FR (M1)
         ╲   ▲   ╱
          ╲  │  ╱
           ╲ │ ╱
            ╲│╱
             ╳
            ╱│╲
           ╱ │ ╲
          ╱  │  ╲
         ╱   │   ╲
      RL (M3)    RR (M2)
            Rear

Motor rotation:
  M1 (FR): CCW    M2 (RR): CW
  M3 (RL): CCW    M4 (FL): CW
```

### スロットル正規化修正

コントローラのスロットルスティックは上半分のみ使用（2048-4095 → 0〜1）。

**ファイル**: `components/stampfly_state/stampfly_state.cpp:396-407`

```cpp
void StampFlyState::getControlInput(float& throttle, float& roll, float& pitch, float& yaw) const
{
    // Throttle uses upper half only (2048-4095 -> 0 to 1), negative clipped
    float throttle_raw = (ctrl_throttle_ - 2048) / 2048.0f;
    throttle = (throttle_raw < 0) ? 0 : throttle_raw;  // 0 to 1 (clipped)
    roll = (ctrl_roll_ - 2048) / 2048.0f;              // -1 to +1
    pitch = (ctrl_pitch_ - 2048) / 2048.0f;            // -1 to +1
    yaw = (ctrl_yaw_ - 2048) / 2048.0f;                // -1 to +1
}
```

### CLIモータテストコマンド

**ファイル**: `components/stampfly_cli/cli.cpp:687-777`

```
motor test <id> <throttle>  - 単体モーターテスト (id:1-4, throttle:0-100)
motor all <throttle>        - 全モーターテスト (throttle:0-100)
motor stop                  - 全モーター停止
```

`testMotor()` を使用してarm状態に関係なく動作。

### コントローラからのARM/DISARMトグル

**ファイル**: `main/main.cpp:1209-1235`

立ち上がりエッジ検出によるトグル動作:

```cpp
// Handle arm/disarm toggle from controller (rising edge detection)
static bool prev_arm_flag = false;
bool arm_flag = (packet.flags & CTRL_FLAG_ARM) != 0;

// Detect rising edge (button press)
if (arm_flag && !prev_arm_flag) {
    if (flight_state == FlightState::IDLE || flight_state == FlightState::ERROR) {
        // IDLE/ERROR → ARM
        state.requestArm();
        g_motor.arm();  // Enable motor driver
    } else if (flight_state == FlightState::ARMED) {
        // ARMED → DISARM
        state.requestDisarm();
        g_motor.disarm();  // Disable motor driver
    }
}
prev_arm_flag = arm_flag;
```

### LED機能拡張

**ファイル**: `components/stampfly_led/led.cpp`

| 機能 | 説明 |
|------|------|
| NVS保存 | `setBrightness(val, true)` で明るさをNVSに保存 |
| 低バッテリー警告 | 3.4V未満でシアン点滅 (`showLowBatteryCyan()`) |
| FLYING色変更 | 緑→黄色に変更 |

### デバッグモード

電池なし（USB給電のみ）でもARMできるモード。

```
debug on   - デバッグモード有効（LOW_BATTERYエラー無視でARM可能）
debug off  - デバッグモード無効
```

### 開発者ドキュメント

**ファイル**: `docs/developer_guide.md` (新規作成)

- main.cpp構造説明
- タスク構成と優先度
- データフロー図
- 飛行制御実装の手引き（PID制御例含む）

### 変更ファイル一覧

| ファイル | 変更内容 |
|---------|---------|
| `main/main.cpp` | ControlTask追加、ARM/DISARMトグル、g_motor_ptr定義 |
| `components/stampfly_state/stampfly_state.cpp` | スロットル正規化修正 |
| `components/stampfly_cli/cli.cpp` | motorコマンド実装 |
| `components/stampfly_cli/CMakeLists.txt` | stampfly_motor依存追加 |
| `components/stampfly_led/led.cpp` | NVS保存、低バッテリーLED、FLYING色変更 |
| `docs/developer_guide.md` | 新規作成 |

---

## 完了済みフェーズ

### Phase 1: プロジェクト基盤 ✅ 完了
- ESP-IDF v5.4 プロジェクト構成
- 全コンポーネントのスタブ作成
- CMakeLists.txt 設定

### Phase 2: ドライバ層実装 ✅ 完了
- stampfly_imu (BMI270) - 既存リポジトリ利用
- stampfly_tof (VL53L3CX) - 既存リポジトリ利用
- stampfly_opticalflow (PMW3901) - 既存リポジトリ利用
- stampfly_mag (BMM150) - 新規実装
- stampfly_baro (BMP280) - 新規実装
- stampfly_power (INA3221) - 新規実装

### Phase 3: サービス層実装 ✅ 完了
- stampfly_motor - LEDC PWM、X-quad構成ミキサー
- stampfly_led - WS2812 RMT制御、状態表示パターン
- stampfly_buzzer - LEDC PWM、プリセット音
- stampfly_button - GPIO0デバウンス、長押し検出
- stampfly_filter - LowPassFilter、MedianFilter、OutlierDetector、Vec3
- stampfly_math - Vec3、Quaternion、Matrix テンプレート
- stampfly_eskf - ESKF統合推定器、AttitudeEstimator、AltitudeEstimator、VelocityEstimator
- stampfly_state - StampFlyState (シングルトン)、SystemManager
- stampfly_comm - ESP-NOW通信、ControlPacket、TelemetryPacket、ペアリング機能
- stampfly_cli - USB CDCコンソール、コマンド処理

### Phase 4: タスク統合 ✅ 完了
main.cppに以下を統合:

**FreeRTOSタスク:**
| タスク名 | 周波数 | 優先度 | Core | スタック |
|----------|--------|--------|------|---------|
| IMUTask | 400Hz | 24 | 1 | 16384 |
| ControlTask | 400Hz | 23 | 1 | 8192 |
| OptFlowTask | 100Hz | 20 | 1 | 8192 |
| MagTask | 100Hz | 18 | 1 | 8192 |
| BaroTask | 50Hz | 16 | 1 | 8192 |
| ToFTask | 30Hz | 14 | 1 | 8192 |
| PowerTask | 10Hz | 12 | 0 | 4096 |
| LEDTask | 30Hz | 8 | 0 | 4096 |
| ButtonTask | 100Hz | 10 | 0 | 4096 |
| CommTask | 50Hz | 15 | 0 | 4096 |
| CLITask | - | 5 | 0 | 4096 |

**初期化関数:**
- initI2C() - I2Cバス初期化
- initSensors() - 全センサ初期化
- initActuators() - モーター、LED、ブザー、ボタン
- initEstimators() - ESKF、姿勢/高度推定器
- initCommunication() - ESP-NOW
- initCLI() - USBシリアルCLI

**イベントハンドラ:**
- onButtonEvent() - ARM/DISARM、ペアリング、リセット
- onControlPacket() - コントローラからの制御入力

**ビルド状態:** 成功 (891KB, 72%空き)

---

## 実機テスト結果 (2025-11-28)

### 動作確認済み
| 項目 | 通信 | 値検証 | 備考 |
|------|------|--------|------|
| ブート・起動音 | ✅ | - | 正常に起動 |
| IMU (BMI270) | ✅ | ✅ | SPI通信成功、Teleplotで値確認 |
| Mag (BMM150) | ✅ | ✅ | I2C通信成功、Teleplotで値確認 |
| Baro (BMP280) | ✅ | ✅ | I2C通信成功、気圧・高度取得確認 |
| ToF Bottom (VL53L3CX) | ✅ | ✅ | 距離値取得確認 |
| ToF Front (VL53L3CX) | ✅ | - | オプション（バッテリアダプタ兼用）|
| OptFlow (PMW3901) | ✅ | ✅ | SPI通信成功、burst read動作確認 |
| Power (INA3221) | ✅ | 未確認 | 電圧・電流取得、値の妥当性は未検証 |
| LED (WS2812) | ✅ | ✅ | 状態表示パターン動作確認 |
| Buzzer | ✅ | ✅ | 起動音・警告音動作確認 |
| Button | ✅ | ✅ | イベント検出動作確認 |
| ESP-NOW | ✅ | 未確認 | 初期化成功、通信は未テスト |
| CLI | ✅ | ✅ | helpコマンド動作確認、エコーバック正常 |
| CLI teleplot | ✅ | ✅ | Teleplotストリーミング動作確認 |
| CLI binlog | ✅ | ✅ | バイナリログ出力、エラー率0%達成 |

### CLI コマンド一覧
| コマンド | 説明 |
|---------|------|
| `help` | 利用可能コマンド表示 |
| `status` | システム状態表示 |
| `sensor [imu\|mag\|baro\|tof\|flow\|power\|all]` | センサ値表示（実データ） |
| `teleplot [on\|off]` | Teleplotストリーミング開始/停止 |
| `binlog [on\|off]` | バイナリログ出力開始/停止（100Hz、128Bパケット） |
| `loglevel [level] [tag]` | ESP_LOGレベル設定（none/error/warn/info/debug/verbose） |
| `magcal [start\|stop\|status\|save\|clear]` | 地磁気キャリブレーション |
| `calib [gyro\|accel\|mag]` | キャリブレーション（stub） |
| `motor [arm\|disarm\|test <id> <throttle>]` | モーター制御（stub） |
| `pair` | ペアリングモード開始 |
| `unpair` | ペアリング解除 |
| `gain <name> <value>` | 制御ゲイン設定（stub） |
| `attitude` | 姿勢表示（stub） |
| `version` | バージョン情報表示 |
| `reset` | システムリセット |

### センサ軸変換 (NED座標系) ✅ 完了

| センサ | 変換式 | 備考 |
|--------|--------|------|
| BMI270 (IMU) | X=sensor_y, Y=sensor_x, Z=-sensor_z | 実測確認済 |
| BMM150 (Mag) | X=-sensor_y, Y=sensor_x, Z=sensor_z | 実測確認済 |
| PMW3901 (Flow) | X=-sensor_y, Y=sensor_x | 実測確認済 |

### 重力補償・姿勢計算 ✅ 修正完了

- **ESKF predict()**: `accel_world.z += gravity` (比力 + 重力 = 実加速度)
- **ESKF updateAccelAttitude()**: 期待重力ベクトル `(0, 0, -g)` に修正
- **AttitudeEstimator**: `atan2(-ay, -az)` / `atan2(ax, sqrt(...))` に修正
- **NED座標系での静止時加速度**: accel_z ≈ **-9.8 m/s²** が正しい

### センサ値の妥当性 ✅ 確認済
- IMU: 静止時にZ軸加速度≈-9.8m/s²、ジャイロ≈0
- Mag: 地磁気取得確認
- Baro: 気圧・高度取得確認
- ToF: 距離値取得確認
- OptFlow: delta値変化確認
- Power: 電圧取得確認

### 実機テスト中に修正した問題

1. **WiFi/ESP-NOW初期化エラー**
   - 原因: `esp_netif_init()`と`esp_event_loop_create_default()`が未呼び出し
   - 修正: main.cpp で初期化追加

2. **USB CDCコンソール問題**
   - 原因: USB CDCがログ出力と競合
   - 修正: `sdkconfig.defaults`で`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`に変更

3. **スタックオーバーフロー**
   - 原因: タスクスタックサイズ不足 (2048→4096/8192)
   - 修正: 全タスクのスタックサイズ増加

4. **バッテリーチャンネル誤り**
   - 原因: INA3221のCH0→CH1に変更が必要
   - 修正: `battery_channel = 1`に修正

5. **SPI重複初期化エラー**
   - 原因: BMI270とPMW3901でSPIバスを2回初期化
   - 修正: PMW3901に`skip_bus_init`フラグ追加

6. **CLI std::map/ラムダクラッシュ**
   - 原因: 動的メモリ割り当て問題
   - 修正: 静的配列と関数ポインタに変更

7. **起動時の誤低電圧警告**
   - 原因: `last_voltage_v_`初期値が0.0f
   - 修正: 初期値を4.2fに変更

8. **CLIエコーバック遅延**
   - 原因: stdioバッファリングによりESP_LOGと同期して出力
   - 修正: `write()`システムコール直接使用でバッファをバイパス

9. **ToFセンサーI2Cエラー連発**
   - 原因: センサー切断時にエラーログが大量出力
   - 修正: エラー10回連続で該当センサーを自動無効化

10. **正面ToF未接続時のI2Cエラー**
    - 原因: 正面ToF(バッテリアダプタ兼用)が未接続でもアクセス試行
    - 修正: initDualSensorsで正面ToF未検出時はXSHUTをLOWに保持、StampFlyStateにフラグ追加

11. **センサ値がOutlierDetectorでフィルタされる問題**
    - 原因: Mag/Baro/ToFのOutlierDetectorが厳しすぎて初期値を弾いていた
    - 修正: Outlierフィルタを一時的に無効化（キャリブレーション後に再有効化予定）

12. **ToFが0mm/status=255を返す問題**
    - 原因: VL53L3CXの読み取り後にclearInterruptAndStartMeasurement()未呼び出し
    - 修正: ToFTaskでisDataReady()確認後、getDistance()→clearInterruptAndStartMeasurement()の正しいシーケンスに修正

13. **ESKF/Estimator使用時のクラッシュ**
    - 原因: 15x15行列のローカル変数がスタックを消費しすぎ（4個×900バイト≒3.6KB）
    - 修正: 一時行列をクラスメンバ変数に移動（F_, Q_, temp1_, temp2_）してスタック使用量を削減

14. **センサ軸変換不一致**
    - 原因: 各センサの座標系がNED機体座標系と一致していなかった
    - 修正: BMI270/BMM150/PMW3901それぞれに座標変換を追加

15. **重力補償の符号誤り**
    - 原因: 静止時の加速度計測定値(比力)の物理的意味を誤解
    - 修正: `accel_world.z -= gravity` → `accel_world.z += gravity`

16. **姿勢推定の符号誤り**
    - 原因: AttitudeEstimatorのatan2計算で比力の符号を考慮していなかった
    - 修正: `atan2(ay, az)` → `atan2(-ay, -az)` に修正

17. **加速度の単位誤り**
    - 原因: BMI270の出力が[g]単位だがESKFは[m/s²]を期待
    - 修正: IMUタスクで`×9.81`の単位変換を追加

18. **ESKF計算負荷によるIMUハング**
    - 原因: 15x15行列演算が重く400Hzループに間に合わない
    - 暫定対策: ESKF predict頻度を100Hzに制限（4サンプル平均）
    - 恒久対策: スパース行列を展開してスカラー演算に変換（未実装）

19. **Teleplotによるシリアル出力ブロッキング**
    - 原因: 複数のprint()呼び出しがシリアル出力をブロック
    - 修正: バッファにまとめて一括write()に変更

20. **バイナリログのチェックサムエラー**
    - 原因: USB CDC VFSのline ending変換が0x0Aを0x0D 0x0Aに変換
    - 修正: `esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_LF)`で変換無効化
    - 結果: エラー率8.5%→0%に改善

---

## ESKFデバッグ環境 ✅ 完了 (2025-11-30)

PCでESKFのチューニング・デバッグを行うための環境を構築。

### 構成

```
tools/
├── eskf_debug/              # C++ ESKFリプレイ環境
│   ├── CMakeLists.txt
│   ├── eskf_pc.cpp          # PC用ESKF実装
│   ├── eskf_pc.hpp
│   └── replay.cpp           # バイナリログ再生・ESKF実行
└── scripts/
    ├── log_capture.py       # バイナリログキャプチャツール
    ├── visualize_eskf.py    # ESKF出力可視化
    └── estimate_qr.py       # Q/Rパラメータ推定
```

### バイナリログ形式（V2、128バイト）

> **注**: V1 (64バイト、ヘッダ 0xAA 0x55) は廃止されました。詳細は「V1パケット形式廃止」セクションを参照。

| オフセット | サイズ | 内容 |
|-----------|-------|------|
| 0-1 | 2 | ヘッダ (0xAA 0x56) |
| 2-5 | 4 | タイムスタンプ (ms) |
| 6-29 | 24 | IMU (accel xyz, gyro xyz) |
| 30-41 | 12 | 磁気 (xyz) |
| 42-49 | 8 | 気圧 (pressure, alt) |
| 50-57 | 8 | ToF (bottom, front) |
| 58-62 | 5 | OptFlow (dx, dy, squal) |
| 63-74 | 12 | Position (xyz) |
| 75-86 | 12 | Velocity (xyz) |
| 87-98 | 12 | Attitude (roll, pitch, yaw) |
| 99-110 | 12 | Bias (gyro_z, accel_x, accel_y) |
| 111 | 1 | ESKF status |
| 112-126 | 15 | Reserved (baro_ref_alt含む) |
| 127 | 1 | XORチェックサム |

### 使用方法

```bash
# 1. バイナリログキャプチャ
cd tools/scripts
python log_capture.py -p /dev/tty.usbmodem* -o sensor_log.bin

# 2. ESKFリプレイ (C++)
cd tools/eskf_debug
mkdir build && cd build
cmake .. && make
./eskf_debug ../../../sensor_log.bin output.csv

# 3. 可視化
cd tools/scripts
python visualize_eskf.py output.csv

# 4. Q/Rパラメータ推定
python estimate_qr.py sensor_log.bin
```

### 解決した技術的問題

**USB CDC バイナリ出力問題**
- 問題: stdoutの行末変換で0x0A→0x0D 0x0Aに変換され、バイナリが破損
- 原因: ESP-IDF VFSのデフォルト設定が`ESP_LINE_ENDINGS_CRLF`
- 解決: `esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_LF)`
- 注: `CONFIG_ESP_CONSOLE_USB_CDC`使用時は`esp_vfs_cdcacm.h`のAPIを使用

**ESP_LOGとバイナリストリームの競合**
- 問題: ESP_LOGの出力がバイナリストリームに混入
- 解決: binlog on時に`esp_log_level_set("*", ESP_LOG_NONE)`で抑制

---

## PC版ESKFチューニング結果 (2025-11-30)

### 修正内容

| ファイル | 修正 | 内容 |
|---------|------|------|
| `eskf_pc.cpp` predict() | 重力補正 | `accel_world.z -= gravity` → `+= gravity` |
| `eskf_pc.cpp` updateAccelAttitude() | 期待値 | `g_world = (0,0,+g)` → `(0,0,-g)` |
| `eskf_pc.cpp` updateAccelAttitude() | ヤコビアン | `H(0,ATT_Y)=-g, H(1,ATT_X)=+g` → `+g, -g` |
| `eskf_pc.cpp` updateToF() | 傾き閾値 | 傾き > threshold でスキップ追加 |
| `eskf_pc.cpp` updateFlow() | 高度閾値 | ハードコード`0.1f` → `config_.flow_min_height` |

### 推奨Config設定

```cpp
config.flow_min_height = 0.02f;     // 机上テスト対応
config.flow_noise = 0.01f;          // 速度補正改善（デフォルト1.0は大きすぎ）
config.tof_tilt_threshold = 0.35f;  // 20° - 傾き時のToF誤測定防止
```

### テスト結果

**静的テスト（机上静止）:**
| 項目 | 値 | 評価 |
|------|-----|------|
| Position X/Y | 数mm | ✓ ドリフトなし |
| Position Z | -19mm | ✓ ToF値と一致 |
| Velocity | ~1mm/s | ✓ 静止状態 |
| Roll/Pitch | ~-1.2° | ✓ 机の傾き検出 |

**動的テスト（持ち上げ→Roll/Pitch/Yaw揺らし）:**
| 項目 | 値 | 評価 |
|------|-----|------|
| 高度 | 最大0.40m | ✓ 期待値~40cmに一致 |
| Roll追従 | ±45° | ✓ |
| Pitch追従 | -29°~+62° | ✓ |
| 位置ドリフト | X:2mm, Y:6mm | ✓ |
| 最終速度 | ~1-2mm/s | ✓ 静止に収束 |

### 発見した問題と解決

1. **高度スパイク問題**
   - 原因: ToFセンサが傾くと床面でなく壁/遠方を測定
   - 解決: `updateToF()`に傾き閾値チェック追加（>20°でスキップ）

2. **水平速度ドリフト問題**
   - 原因: `flow_noise=1.0`が大きすぎてフロー観測が無視される
   - 解決: `flow_noise=0.01`に調整

3. **机上テストでフロー更新されない問題**
   - 原因: `flow_min_height=0.1m`で高度25mmの机上では無効
   - 解決: `flow_min_height=0.02m`に調整

### 次のステップ

- [x] 本体コード(`components/stampfly_eskf/eskf.cpp`)に修正適用 ✅
- [x] main.cppでupdateFlowWithGyro()を呼び出すように変更 ✅
- [x] フローオフセット補正を実機コードに適用 ✅
- [x] デフォルトQ/Rパラメータをチューニング済み値に更新 ✅
- [x] 磁力計キャリブレーション実装（Yaw精度向上） ✅ 2025-12-01
- [ ] 実機でESKFチューニング結果を検証
- [ ] **フローオフセットキャリブレーション** - 静止ホバリングデータでflow_dx/dy_offsetを決定
  - 現在の推定値: flow_dx_offset=0.29, flow_dy_offset=0.29 counts (閉ループテストから逆算)
  - 正確なキャリブレーションには実際のホバリング静止データが必要

---

## 四角形移動テスト結果 (2025-11-30)

### テスト内容
- 持ち上げ → 時計回りに四角形移動 → 下ろす
- 期待移動量: 20-30cm

### 実装した改良

**1. Body→NED座標変換** ✅ 実装・有効
```cpp
// updateFlowWithGyro()内
float cos_yaw = std::cos(state_.yaw);
float sin_yaw = std::sin(state_.yaw);
float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;  // North
float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;  // East
```
- Yaw角約110°変化に対応
- 軌跡がNED座標系で一貫した方向に表示

**2. ジャイロ補償** ✅ 実装・有効（回帰分析でキャリブレーション）
```cpp
// 回帰分析で得た補償係数（counts/[rad/s]単位）:
//   flow_dx = 1.35×gyro_x + 9.30×gyro_y + offset
//   flow_dy = -2.65×gyro_x + 0×gyro_y + offset
constexpr float flow_scale = 0.08f;  // rad/count
constexpr float k_xx = 1.35f * flow_scale;
constexpr float k_xy = 9.30f * flow_scale;
constexpr float k_yx = -2.65f * flow_scale;
constexpr float k_yy = 0.0f * flow_scale;

float flow_x_comp = flow_x - k_xx * gyro_x - k_xy * gyro_y;
float flow_y_comp = flow_y - k_yx * gyro_x - k_yy * gyro_y;
```
- 回帰分析により補償係数を決定
- flow_scaleと補償係数のスケールを統一

**3. フロースケール最適化** ✅ キャリブレーション完了
- PMW3901理論値: 0.021 rad/count (FOV=42°, 35px)
- 最適値: **0.08 rad/count** (理論値×4倍)
- Y範囲25.6cmが目標(20-30cm)に合致

### テスト結果（最終）

| 設定 | X範囲 | Y範囲 | 閉ループエラー |
|------|-------|-------|----------------|
| 初期 (flow_scale=0.1, no gyro comp) | 30.3cm | 14.2cm | 27cm |
| 軸入れ替えのみ (scale=0.1) | 15.3cm | 31.9cm | 25.6cm |
| **最終 (scale=0.08, gyro comp)** | **13.5cm** | **25.6cm** | **22.4cm** |

### 発見した問題と修正

**1. フロー軸変換の誤り**:
- 四角形移動データを分析し、実測と期待の対応を確認
- 右移動時: flow_x=-261 → vy_body > 0 が期待
- 修正前: `vx_body = -flow_x * h, vy_body = -flow_y * h`
- 修正後: `vx_body = -flow_y * h, vy_body = -flow_x * h`

**2. ジャイロ補償のスケール不一致**:
- 問題: 補償係数(counts/[rad/s])とflow値(rad)のスケール不一致
- 解決: `k * flow_scale * gyro` でスケールを統一

**3. フロースケールの決定**:
- 理論値(0.021)では範囲が小さすぎ、大きすぎるとドリフト
- 実験的に0.08が最適（範囲とエラーのバランス）

### 本体コードへの適用 ✅ 完了
`components/stampfly_eskf/eskf.cpp`に以下を適用:
- `updateFlowWithGyro()`関数追加
- 軸入れ替え: `vx_body = -flow_y * h, vy_body = -flow_x * h`
- ジャイロ補償: 回帰分析係数適用
- Body→NED変換: Yaw回転

---

## フローオフセットキャリブレーション (2025-11-30)

### キャリブレーション方法

2つの手法を比較検証:

1. **静止ホバリング法**: 手で持ち上げて静止させ、flow_dx/dyの平均値をオフセットとして使用
2. **閉ループ推定法**: 開始/終了位置の一致条件から逆算してオフセットを推定

### 結果比較

| 手法 | flow_dx_offset | flow_dy_offset | 閉ループエラー |
|------|---------------|----------------|----------------|
| 静止ホバリング | 0.21 | -0.05 | 20.0 cm |
| **閉ループ推定** | **0.29** | **0.29** | **5.1 cm** |

閉ループ推定法が77%優れた結果となり、この値を採用。

### 最終パラメータ

```cpp
// tools/eskf_debug/eskf_pc.cpp
constexpr float flow_scale = 0.08f;
constexpr float flow_dx_offset = 0.29f * flow_scale;  // 閉ループ推定
constexpr float flow_dy_offset = 0.29f * flow_scale;

// ジャイロ補償係数
constexpr float k_xx = 1.35f * flow_scale;
constexpr float k_xy = 9.30f * flow_scale;
constexpr float k_yx = -2.65f * flow_scale;
constexpr float k_yy = 0.0f * flow_scale;
```

### テスト結果サマリー

| データセット | 閉ループエラー | X範囲 | Y範囲 | 飛行時間 |
|-------------|---------------|-------|-------|---------|
| square_motion.bin | **5.1 cm** | 12.0 cm | 15.7 cm | 30.3 s |
| motion_test.bin | 20.0 cm | 22.3 cm | 28.6 cm | 15.3 s |

motion_test.binでエラーが大きい理由:
- より激しい動き（Roll ±45°, Pitch -32°~+59°）
- Yawがフル回転（-178°~+180°）
- 大きな高度変化（ToFスパイク含む）

### 出力ファイル

各データセットについて14ファイルの包括的な可視化を生成:
- `tools/scripts/square_result/closed_loop_analysis/` - square_motion.bin
- `tools/scripts/square_result/motion_test_analysis/` - motion_test.bin

### 可視化
- `tools/scripts/square_result/closed_loop_analysis/` - square_motion.bin
- `tools/scripts/square_result/motion_test_analysis/` - motion_test.bin

---

## Q/Rパラメータ検証 (2025-11-30)

### 検証内容

センサデータ（static_test02.bin）からAllan分散と統計分析でQ/Rパラメータを推定し、デフォルト値と比較検証。

### 推定値 vs デフォルト値

| パラメータ | 推定値 | デフォルト | 比率 |
|-----------|--------|----------|------|
| **Process Noise (Q)** |
| gyro_noise | 0.0002 rad/s/√Hz | 0.001 | 5x過大 |
| accel_noise | 0.001 m/s²/√Hz | 0.1 | 100x過大 |
| gyro_bias_noise | 0.00002 | 0.00005 | 2.5x過大 |
| accel_bias_noise | 0.0001 | 0.001 | 10x過大 |
| **Measurement Noise (R)** |
| baro_noise | 0.099 m | 1.0 m | 10x過大 |
| tof_noise | 0.0013 m | 0.05 m | 38x過大 |
| flow_noise | 0.74 | 1.0 | 妥当 |

### 検証結果

| 設定 | 閉ループエラー | Yaw Bias推定 | 評価 |
|-----|--------------|-------------|------|
| デフォルトQ/R | **5.1 cm** | 0.46°/s (安定) | ✓ 良好 |
| 推定Q/R | 9.1 cm | -9.5°/s (発散) | ✗ 不安定 |

### 結論

**プロセスノイズ(Q)はデフォルト値を維持すべき**

理由:
1. 推定したgyro_bias_noiseが小さすぎると、Yawバイアス推定が発散
2. 静止データからの推定値は「理論的最小値」であり、実運用にはマージンが必要
3. デフォルト値は経験的に調整されており、安定性を重視した設計

観測ノイズ(R)については推定値を参考に調整可能:
- baro_noise: 0.1m (推定0.099m)
- tof_noise: 現状のtof_tilt_thresholdで十分制御

### 出力ファイル
- `tools/scripts/qr_params.json` - 推定パラメータ
- `tools/scripts/qr_analysis.png` - Allan分散・ヒストグラム
- `tools/scripts/square_result/qr_comparison.png` - 比較結果

---

## 実機ESKFデバッグ (2025-12-01)

### 問題の経緯

PC版ESKFで閉ループエラー5.1cmを達成後、実機(ESP32-S3)での検証を実施。

### テスト結果サマリー

| バージョン | 問題 | 対策 | 結果 |
|-----------|------|------|------|
| test_v8 | Position発散(5-22m) | P_(BG_Z)=0による共分散特異性 | ✗ 発散 |
| test_v9 | Yaw 131.9°ドリフト | P_(BG_Z)修正、BG_Zリセット追加 | Position安定、Yaw発散 |
| test_v10-v11 | Mag有効化でGyro Bias Z -750°/s | Mag未キャリブ(420μT vs期待45μT) | ✗ 発散 |
| test_v12 | Gyro Bias Z -246°/s | mag_enabled処理が不完全 | ✗ 発散 |
| test_v13 | Yaw 164°ドリフト、軌跡歪み | Q_(BG_Z)=0、injectErrorStateスキップ追加 | Bias安定、Yaw発散 |
| test_v14 | Yaw固定成功、X軸符号問題 | gyro_corrected.z=0、yaw=0固定 | 四角形見えるがX反転 |
| test_v15 | Yaw固定失敗(40°変化) | injectErrorStateでYaw更新 | ✗ 発散 |
| test_v16 | ESKF発散(Position±10m) | injectErrorStateにYaw固定追加 | ✗ Roll/Pitch異常で発散 |
| **test_v17** | **Position安定(±0.8m)** | 再テスト | **✓ Yaw固定成功、座標系要調査** |

### 実装した修正

#### 1. mag_enabled=false時のGyro Bias Z処理

**問題**: 地磁気センサ無効時、Gyro Bias Zの観測可能性がなくバイアス推定が発散

**修正**:
```cpp
// predict() - Q行列
Q_(BG_Z, BG_Z) = config_.mag_enabled ? bg_var : 0.0f;

// injectErrorState() - バイアス更新スキップ
if (config_.mag_enabled) {
    state_.gyro_bias.z += dx(BG_Z, 0);
}

// measurementUpdate() - P行列リセット
if (!config_.mag_enabled) {
    for (int i = 0; i < 15; i++) {
        if (i != BG_Z) {
            P_(BG_Z, i) = 0.0f;
            P_(i, BG_Z) = 0.0f;
        }
    }
    P_(BG_Z, BG_Z) = 1e-10f;
}
```

#### 2. Yaw固定モード（地磁気無効時）

**問題**: Yawリファレンスがないとドリフトで軌跡が歪む

**修正**: predict()とinjectErrorState()の両方でYaw=0に固定
```cpp
// predict()
if (!config_.mag_enabled) {
    gyro_corrected.z = 0.0f;  // Yawレートを0に
}
// ... 姿勢更新後 ...
if (!config_.mag_enabled) {
    state_.yaw = 0.0f;
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
}

// injectErrorState() - 観測更新後もYaw固定
if (!config_.mag_enabled) {
    state_.yaw = 0.0f;
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
}
```

### test_v17の結果

| 項目 | 値 | 評価 |
|------|-----|------|
| Yaw固定 | 0.0° → 0.0° (ドリフト: 0.0°) | ✓ |
| Gyro Bias Z | 0.000 °/s | ✓ |
| Position範囲 | X: ±0.8m, Y: ±0.4m | ✓ 発散なし |
| Total drift | 0.023m | ✓ 良好 |
| Accel Bias | 安定 | ✓ |

### 残存課題

1. **X軸の座標系問題**: 四角移動で後方移動時にX+になるか確認が必要
   - NED座標系(Yaw=0): 前方=+X, 後方=-X, 右=+Y, 左=-Y
   - test_v17の軌跡分析では方向の確認が必要

2. **地磁気キャリブレーション**:
   - 現在の値(~420μT)は期待値(~45μT)の約10倍
   - キャリブレーション実装後にmag_enabled=trueで再テスト

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `components/stampfly_eskf/eskf.cpp` | mag_enabled処理、Yaw固定モード |
| `tools/eskf_debug/eskf_pc.cpp` | 同上（PC版同期） |
| `main/main.cpp` | mag_enabled=false設定 |

---

## 地磁気キャリブレーション ✅ 完了 (2025-12-01)

### 実装内容

`components/stampfly_mag/` に地磁気キャリブレーション機能を追加。

**キャリブレーションモデル:**
```
mag_calibrated = soft_iron * (mag_raw - hard_iron)
```

- **Hard Iron**: 定常的なオフセット（ハードウェアによる磁場歪み）
- **Soft Iron**: スケール係数（各軸の感度差を補正）

### ファイル構成

```
components/stampfly_mag/
├── include/
│   ├── bmm150.hpp
│   └── mag_calibration.hpp   # 新規追加
├── bmm150.cpp
└── mag_calibration.cpp       # 新規追加
```

### CLIコマンド

```
> magcal start    # キャリブレーション開始（デバイスを8の字に回転）
> magcal stop     # キャリブレーション終了・計算
> magcal status   # 現在の状態・校正値表示
> magcal save     # NVSに保存
> magcal clear    # NVSから削除
```

### キャリブレーションアルゴリズム

1. **Hard Iron推定**: 球体フィッティング（最小二乗法）
   - 収集したサンプルから中心点（オフセット）を推定
2. **Soft Iron推定**: 楕円体→球体正規化
   - 各軸の範囲から正規化スケールを計算
3. **品質評価**: フィットネス値（校正後のnormの標準偏差）

### 検証結果

| テスト | Center (X, Y) | 評価 |
|-------|---------------|------|
| mag_plane_test.png | (2.0, -1.2) | ✓ 原点近傍 |
| mag_plane_test2.png | (18.0, 1.4) | △ 環境依存（校正場所≠テスト場所）|

**注意**: 校正は実際の飛行環境で行う必要あり。

### ESKF連携

- `magcal save` 後にデバイスをリセットすると、ESKFの `mag_enabled` が自動的に `true` に設定
- main.cpp の `initEstimators()` で NVS から校正データをロード

---

## デバッグツール整理 ✅ 完了 (2025-12-01)

### 変更内容

1. **可視化ツール統合**: 複数の可視化スクリプトを `visualize_device_log.py` に統合
2. **V2フォーマット標準化**: V1(64B)を廃止、V2(128B)をデフォルトに
3. **アーカイブ整理**: 過去のログ・解析結果を `archive/` に移動

### ツール構成（整理後）

```
tools/scripts/
├── README.md                 # ドキュメント
├── log_capture.py            # ログキャプチャ・変換
├── visualize_device_log.py   # 統合可視化ツール
├── plot_mag_xy.py            # 磁気キャリブレーション確認
├── estimate_qr.py            # Q/Rパラメータ推定
├── requirements.txt          # Python依存関係
└── archive/                  # 過去のログ・解析結果（gitignore）
```

### バイナリログ形式（V2、128バイト）

| オフセット | サイズ | 内容 |
|-----------|-------|------|
| 0-1 | 2 | ヘッダ (0xAA 0x56) |
| 2-5 | 4 | タイムスタンプ (ms) |
| 6-29 | 24 | IMU (accel xyz, gyro xyz) |
| 30-41 | 12 | 磁気 (xyz) |
| 42-49 | 8 | 気圧 (pressure, alt) |
| 50-57 | 8 | ToF (bottom, front) |
| 58-62 | 5 | OptFlow (dx, dy, squal) |
| 63-74 | 12 | Position (xyz) |
| 75-86 | 12 | Velocity (xyz) |
| 87-98 | 12 | Attitude (roll, pitch, yaw) |
| 99-110 | 12 | Bias (gyro_z, accel_x, accel_y) |
| 111 | 1 | ESKF status |
| 112-126 | 15 | Reserved |
| 127 | 1 | XORチェックサム |

### 可視化モード

```bash
# デバイスログのみ
python visualize_device_log.py sensor.bin

# PC版ESKFのみ
python visualize_device_log.py --pc eskf.csv --mode pc

# デバイス vs PC 比較
python visualize_device_log.py sensor.bin --pc eskf.csv --mode both

# 磁気XYプロット
python visualize_device_log.py sensor.bin --mag-xy
```

---

## センサデータ整合性検証 ✅ 完了 (2025-12-01)

### 検証目的

バイナリログに記録されるセンサデータが、デバイスESKFとPC版ESKFで同一に使用されているか確認。

### 検証結果

| センサ | 単位 | 座標変換 | 整合性 |
|--------|------|---------|--------|
| IMU (加速度) | [m/s²] | NED変換済み | ✅ |
| IMU (ジャイロ) | [rad/s] | NED変換済み | ✅ |
| 磁気 | [µT] | NED+キャリブ済み | ✅ |
| 気圧 | [Pa], [m] | 絶対高度 | ✅ (基準高度追加で改善) |
| ToF | [m] | そのまま | ✅ |
| OptFlow | counts | 第1段階変換済み | ✅ |

### 発見・修正した不整合

#### 1. センサ更新レートの不一致
| センサ | デバイス | PC (修正前) | PC (修正後) |
|--------|---------|------------|------------|
| Flow | 100Hz | 20Hz | **100Hz** |
| AccelAtt | 50Hz | 10Hz | **50Hz** |

**修正**: `replay.cpp` の更新レートをデバイスに合わせた

#### 2. 気圧高度の基準点不一致

- **問題**: デバイスは起動時の気圧高度を基準、PCはログ最初のパケットを基準
- **修正**: V2パケットに `baro_ref_alt` フィールドを追加

**V2パケット構造変更**:
```cpp
// Status + metadata (17 bytes)
uint8_t eskf_status;
float baro_ref_alt;     // [m] 気圧基準高度（新規）
uint8_t reserved[11];   // 15→11に縮小
uint8_t checksum;
```

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `stampfly_state.hpp/cpp` | `baro_reference_altitude_` 追加、getter/setter |
| `main.cpp` | 基準高度をStateに保存 |
| `cli.hpp` | V2パケットに `baro_ref_alt` 追加 |
| `cli.cpp` | パケットに基準高度を記録 |
| `replay.cpp` | 更新レート修正、基準高度をログから使用 |

---

## 次のステップ

### 自動キャリブレーションフレームワーク 📋 設計完了

詳細設計: `docs/auto_calibration_design.md`

センサデータからESKFパラメータを自動最適化するフレームワーク:

```
Phase 1: センサノイズ特性推定 (静止データ)
    ↓ Allan分散、統計分析
Phase 2: スケール・軸変換 (単軸移動データ)
    ↓ 最小二乗法
Phase 3: オフセット最適化 (閉ループデータ)
    ↓ グリッドサーチ + Nelder-Mead
Phase 4: Q/R微調整 (複数データセット)
    ↓ 複合評価指標
最終パラメータ出力
```

**実装優先度**: Step 1-3を優先（Phase 1-3で実用的なキャリブレーション可能）

### 直近の作業予定（優先度順）

1. **モータードライバ実機テスト** - PWM出力確認

2. **ESP-NOW通信テスト** - コントローラとの双方向通信

3. **状態遷移統合テスト** - INIT→IDLE→ARMED

### 残りの Phase 4 作業 (テスト)

#### 単体テスト
| 対象 | テスト内容 | 状態 |
|------|----------|------|
| BMI270Wrapper | 初期化・読み出し | ✅ 実機確認済 |
| BMM150 | 初期化・読み出し | ✅ 実機確認済 |
| BMP280 | 初期化・読み出し | ✅ 実機確認済 |
| VL53L3CXWrapper | 初期化・距離取得 | ✅ 実機確認済 |
| PMW3901 | 初期化・モーション取得 | ✅ 実機確認済 |
| PowerMonitor | 初期化・電圧取得 | ✅ 実機確認済 |
| MotorDriver | PWM出力 | 未実施 |
| LED | パターン表示 | ✅ 実機確認済 |
| Buzzer | 音出力 | ✅ 実機確認済 |
| LowPassFilter | 数値検証 | 未実施 |
| StampFlyState | 状態遷移 | 未実施 |

#### 統合テスト
| テスト名 | 内容 | 状態 |
|---------|------|------|
| 全センサ同時動作 | 全センサタスク起動、データ取得 | ✅ 実機確認済 |
| ESP-NOW通信 | コントローラとの双方向通信 | 未実施 |
| タスク優先度検証 | 全タスク同時稼働時の遅延計測 | 未実施 |
| メモリリーク検出 | 長時間稼働 (1時間) | 未実施 |
| 状態遷移統合 | INIT→CALIBRATING→IDLE→ARMED | 未実施 |
| 低電圧シミュレーション | 3.4V以下検出 | 未実施 |
| ペアリング機能 | ペアリング実行・NVS保存・復元 | 未実施 |

### Phase 5: ドキュメント作成 (未着手)

| ファイル名 | 内容 | 状態 |
|-----------|------|------|
| `docs/api_reference.md` | 全クラスAPI、関数シグネチャ、使用例 | 未着手 |
| `docs/hardware_setup.md` | ハードウェア接続図、GPIO割り当て | 未着手 |
| `docs/calibration_guide.md` | キャリブレーション手順 | 未着手 |
| `docs/control_implementation_guide.md` | 制御実装ガイド | 未着手 |

---

## ファイル構成

```
stampfly_rtos_skelton/
├── main/
│   └── main.cpp                 # ✅ 全タスク統合済み
├── components/
│   ├── stampfly_imu/            # ✅ BMI270 (既存)
│   ├── stampfly_tof/            # ✅ VL53L3CX (既存)
│   ├── stampfly_opticalflow/    # ✅ PMW3901 (既存)
│   ├── stampfly_mag/            # ✅ BMM150 (新規)
│   ├── stampfly_baro/           # ✅ BMP280 (新規)
│   ├── stampfly_power/          # ✅ INA3221 (新規)
│   ├── stampfly_motor/          # ✅ LEDC PWM (新規)
│   ├── stampfly_led/            # ✅ WS2812 (新規)
│   ├── stampfly_buzzer/         # ✅ LEDC PWM (新規)
│   ├── stampfly_button/         # ✅ GPIO0 (新規)
│   ├── stampfly_filter/         # ✅ LPF, Median, Outlier (新規)
│   ├── stampfly_math/           # ✅ Vec3, Quat, Matrix (新規)
│   ├── stampfly_eskf/           # ✅ ESKF (新規)
│   ├── stampfly_state/          # ✅ State, SystemManager (新規)
│   ├── stampfly_comm/           # ✅ ESP-NOW (新規)
│   └── stampfly_cli/            # ✅ CLI (新規)
├── tools/
│   ├── eskf_debug/              # ✅ ESKFリプレイ環境 (新規)
│   │   ├── CMakeLists.txt
│   │   ├── eskf_pc.cpp/hpp
│   │   └── replay.cpp
│   └── scripts/                 # ✅ デバッグツール (新規)
│       ├── log_capture.py
│       ├── visualize_eskf.py
│       └── estimate_qr.py
└── docs/
    ├── implementation_plan.md   # 実装計画
    └── PROGRESS.md              # この進捗ファイル
```

---

## 次回作業への引き継ぎ事項

### 推奨する次のアクション

1. **実機テスト準備**
   - StampFly実機でファームウェアを書き込み
   - センサ接続確認
   - シリアルモニタでログ確認

2. **CLIによる動作確認**
   - `help` コマンドで利用可能コマンド表示
   - `sensor imu` でIMUデータ確認 (現在はstub)
   - `status` でシステム状態確認 (現在はstub)

3. **CLIのstub実装を実データに置き換え**
   - cli.cpp の各センサコマンドで StampFlyState から実データ取得
   - `sensor imu` → state.getIMUData()
   - `sensor mag` → state.getMagData()
   - など

4. **単体テスト実施**
   - test/unit/ ディレクトリ作成
   - 各センサの初期化・読み出しテスト

5. **ドキュメント作成**
   - api_reference.md
   - hardware_setup.md
   - calibration_guide.md
   - control_implementation_guide.md

### 注意点

- PMW3901 は例外ベースのAPI (try-catch必須)
- I2Cセンサは共通の `g_i2c_bus` ハンドルを使用
- ESP-NOW通信は for_tdma ブランチ互換のパケット形式

### ビルド方法

```bash
cd /Users/kouhei/Dropbox/01教育研究/20マルチコプタ/stampfly_rtos_skelton
source /Users/kouhei/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

---

## ESKF設計指針 (2025-12-01)

### 重要な設計決定

デバイス版ESKFとPC版ESKFの同期を維持するための重要な設計指針を記録。

#### 1. Baro高度のNED変換方式

**設計指針**: Baro高度のNED変換（上昇=マイナス）は`ESKF::updateBaro()`関数**内部**で行う

```cpp
// ESKF::updateBaro() 内部で変換
void ESKF::updateBaro(float altitude) {
    Matrix<1, 1> z;
    z(0, 0) = -altitude;  // NED: 上昇=マイナス
    // ...
}

// 呼び出し側は相対高度（上昇=プラス）をそのまま渡す
float relative_alt = baro.altitude_m - g_baro_reference_altitude;
g_eskf.updateBaro(relative_alt);  // NED変換しない
```

**理由**:
- 座標系変換の責務をESKF内部に集約
- 呼び出し側での変換忘れを防止
- デバイス版とPC版で同一の処理を保証

#### 2. メンバ変数による行列管理（ESP32-S3スタック制約）

**設計指針**: 15x15行列はローカル変数ではなく**クラスメンバ変数**として保持する

```cpp
// eskf.hpp - メンバ変数として宣言
Matrix<15, 15> F_;      // 状態遷移行列
Matrix<15, 15> Q_;      // プロセスノイズ共分散
Matrix<15, 15> temp1_;  // 一時計算用
Matrix<15, 15> temp2_;  // 一時計算用
```

**理由**:
- ESP32-S3のスタックサイズ制限（タスクあたり数KB）
- 15x15 float行列は約900バイト（15×15×4=900）
- ローカル変数として複数作成するとスタックオーバーフロー発生
- メンバ変数化によりヒープ領域を使用し、スタック圧迫を回避

#### 3. PC版とデバイス版の同期

**設計指針**: PC版ESKFはデバイス版と同一の変数管理方式に合わせる

- PC版はメモリ制約がないが、コード構造をデバイス版に合わせる
- これにより両者の動作の一貫性を保証
- デバッグ時にPC版とデバイス版の差異による問題を排除

### 関連ファイル

| ファイル | 役割 |
|---------|------|
| `components/stampfly_eskf/eskf.cpp` | デバイス版ESKF（メンバ変数方式 + 内部NED変換）|
| `components/stampfly_eskf/include/eskf.hpp` | メンバ変数宣言 |
| `tools/eskf_debug/eskf_pc.cpp` | PC版ESKF（デバイス版と同期）|
| `tools/eskf_debug/replay.cpp` | バイナリログ再生（外部NED変換なし）|
| `main/main.cpp` | Baro呼び出し（外部NED変換なし）|

---

## PC vs Device ESKFデバッグ (2025-12-01 続き)

### 実施した修正

#### 1. ESKF updateをIMUTaskに集約（レースコンディション対策）

**問題**: 複数タスクから同時にESKFのupdate関数が呼ばれ、状態不整合が発生

**修正**:
- OptFlowTask: ESKF update削除（重複していた）
- MagTask/BaroTask/ToFTask: ESKF update → `data_ready` フラグ設定
- IMUTask: フラグを確認してESKF update（100Hz周期で実行）

```cpp
// センサタスク（例: BaroTask）
if (g_baro_reference_set) {
    float relative_alt = baro.altitude_m - g_baro_reference_altitude;
    g_baro_data_cache = relative_alt;
    g_baro_data_ready = true;  // フラグを立てるだけ
}

// IMUTask内で集約処理
if (g_baro_data_ready) {
    g_baro_data_ready = false;
    g_eskf.updateBaro(g_baro_data_cache);
}
if (g_tof_data_ready) { ... }
if (g_mag_data_ready) { ... }
```

#### 2. tof_noiseパラメータ調整

**問題**: ToFの信頼度が過小評価され、Baroに過度に依存

| 項目 | 修正前 | 修正後 | 実測値 |
|------|--------|--------|--------|
| tof_noise | 0.05m | 0.002m | 0.001m |

**効果**:
- Pos Z vs ToF 相関: 0.18 → 0.74-0.79
- Pos Z vs Baro 相関: 0.90 → 0.07
- 高度推定がToFに追従するように改善

#### 3. PC版にreset()とgyro_bias復元を追加

**問題**: DeviceはBinlog開始時にreset()するが、PC版は初期化のまま

**修正**: `replay.cpp`
```cpp
eskf.init(config);
eskf.reset();  // Deviceと同じ初期状態に

// Deviceが保持しているgyro_bias_zを復元
if (has_device_eskf && packets.size() > 0) {
    Vector3 initial_gyro_bias(0.0f, 0.0f, packets[0].gyro_bias_z);
    eskf.setGyroBias(initial_gyro_bias);
}
```

#### 4. PC版にsetGyroBias()追加

**問題**: PC版(eskf_pc.cpp)にsetGyroBias()関数がなかった

**修正**: `eskf_pc.cpp`に追加
```cpp
void ESKF::setGyroBias(const Vector3& bias)
{
    state_.gyro_bias = bias;
}
```

### Static Test 8 結果

| 項目 | Test 7 (修正前) | Test 8 (修正後) | 改善率 |
|------|----------------|-----------------|--------|
| 位置誤差 (Mean) | 1.3 cm | 0.27 cm | 5倍改善 |
| 位置誤差 (Max) | 6.1 cm | 0.53 cm | 12倍改善 |
| 最終位置誤差 | 5.3 mm | 5.0 mm | - |
| Yaw差 (Mean) | 0.18° | 0.26° | - |

### 残存課題と原因分析

#### 1. PCの速度がDeviceよりノイジー（Velocity X）

**現象**:
```
Velocity std (static test 8, t>5s):
  Velocity X: PC 0.14 cm/s, Device 0.07 cm/s (PC 2.0倍)
  Velocity Y: PC 0.08 cm/s, Device 0.08 cm/s (1.0倍 - 一致!)
```

**原因分析**:
- `flow_noise = 0.01` → `flow_noise = 0.5` に修正で改善
- 以前は観測を信頼しすぎて白色ノイズ的挙動、修正後は正しいランダムウォーク挙動
- Velocity Xのみ2倍ノイジーな原因: **Pitch推定の差**
  - Pitch std: PC=0.017°, Device=0.006° (PC 2.7倍)
  - PitchはVelocity Xに影響（前後方向の傾き）

**根本原因**:
- Device: 400Hz IMU → 4サンプル平均 → 100Hz ESKF
- PC: 100Hzログ → 4サンプル移動平均（40ms遅延）
- IMUサンプリングタイミングの構造的違いによる限界

#### 2. Q/Rパラメータの整合性確認 ✅ 完了

PC版とDevice版の設定を比較検証:

| パラメータ | PC | Device | 一致 |
|-----------|-----|--------|------|
| gyro_noise | 0.001 | 0.001 | ✅ |
| accel_noise | 0.1 | 0.1 | ✅ |
| gyro_bias_noise | 0.00005 | 0.00005 | ✅ |
| accel_bias_noise | 0.001 | 0.001 | ✅ |
| baro_noise | 0.1 | 0.1 | ✅ |
| tof_noise | 0.002 | 0.002 | ✅ |
| mag_noise | 0.3 | 0.3 | ✅ |
| flow_noise | 0.5 | 0.5 | ✅ (修正済) |
| accel_att_noise | 1.0 | 1.0 | ✅ |

**発見**: PC版で `flow_noise = 0.01` に誤設定されていた（Deviceは0.5）
**修正**: PC版もdefaultConfigを使用するように統一

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `main/main.cpp` | data_readyフラグ追加、IMUTaskにESKF update集約、OptFlowTask重複削除 |
| `components/stampfly_eskf/include/eskf.hpp` | tof_noise 0.05→0.002、setAccelBias()追加 |
| `tools/eskf_debug/eskf_pc.cpp` | setGyroBias()、setAccelBias()追加 |
| `tools/eskf_debug/replay.cpp` | reset()、gyro/accel_bias復元追加、flow_noise=0.01削除、IMU平滑化追加 |
| `tools/scripts/visualize_device_log.py` | 速度グラフを色分け実線に変更 |

### 最終テスト結果 (Static Test 8)

| 項目 | PC | Device | 評価 |
|------|-----|--------|------|
| Position Error | - | - | **0.5cm** ✅ |
| Position X/Y 相関 | - | - | **0.999+** ✅ |
| Velocity X std | 0.14 cm/s | 0.07 cm/s | 2.0x (構造的限界) |
| Velocity Y std | 0.08 cm/s | 0.08 cm/s | **1.0x** ✅ |
| Q/Rパラメータ | - | - | **同一** ✅ |

### 結論

- **位置推定**: PC vs Device誤差0.5cmで良好に一致
- **速度推定**: Velocity Yは一致、Velocity Xは構造的な差（IMUサンプリング）
- **Q/Rパラメータ**: 同一設定で統一完了

### 次のステップ

1. **動的テスト（手で動かす）** - 位置推定精度の検証
2. **地磁気有効化テスト** - Yaw推定精度の検証

---

## フローオフセット除去 ✅ 完了 (2025-12-01)

### 問題

静止状態で20秒間に約2cmのXYドリフトが発生。

### 原因分析

1. **フローはほぼゼロ**（mean=-0.002, 累積も-3,-4程度）→ フローはドリフトの原因ではない
2. **しかし速度に1mm/s程度のオフセット**が存在

調査の結果、`updateFlowWithGyro()`内のフローオフセット補正が原因と判明:

```cpp
// 以前の設定
constexpr float flow_dx_offset = 0.29f * flow_scale;  // = 0.0232
constexpr float flow_dy_offset = 0.29f * flow_scale;

// 静止状態での影響計算
// velocity = offset * height = 0.0232 * 0.034m = 0.79 mm/s
```

このオフセットは閉ループテスト（動的飛行）から逆算した値だが、静止状態ではフローが0なのでオフセット補正が逆効果になっていた。

### 修正

フローオフセットを0に設定（動的検討時に再評価）:

```cpp
// components/stampfly_eskf/eskf.cpp
// tools/eskf_debug/eskf_pc.cpp

// フローオフセット補正（動的検討時に再評価）
constexpr float flow_dx_offset = 0.0f;
constexpr float flow_dy_offset = 0.0f;
```

### 結果

| 項目 | Before (offset=0.29) | After (offset=0) | 改善率 |
|------|---------------------|------------------|--------|
| XY Drift (PC, 20s) | 2.20 cm | **0.21 cm** | 90% |
| XY Drift (Device, 20s) | 2.74 cm | **0.20 cm** | 93% |
| PC vs Device Error | 0.54 cm | **0.04 cm** | 93% |
| Velocity X mean | 1.18 mm/s | 0.23 mm/s | - |
| Velocity Y mean | 0.29 mm/s | -0.06 mm/s | - |

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `components/stampfly_eskf/eskf.cpp` | flow_dx/dy_offset = 0 |
| `tools/eskf_debug/eskf_pc.cpp` | 同上 |

### 今後の検討事項

- 動的飛行テスト時にフローオフセットが必要かどうか再評価
- 必要な場合は、静止状態と動的状態を区別するロジックを検討

---

## 動的四角形移動テスト ✅ 修正完了 (2025-12-01)

### 実施内容

30cm高度で20cm四辺の四角形を時計回りに移動するテストを実施。

### 発見した問題と修正

#### 1. フロー軸マッピングの誤り

**問題**: ユーザーが「右」に動かしたが、ESKFは「後方(-X)」を検出

**原因**: フローセンサーの軸マッピングが入れ替わっていた + 符号が逆

```cpp
// 修正前（誤り）
vx_body = -flow_y_comp * height;
vy_body = -flow_x_comp * height;

// 修正後（正しい）
vx_body = flow_x_comp * height;
vy_body = flow_y_comp * height;
```

**検証結果**:
- 最初の動き: Right (+Y) ✓ 正しく検出
- 2番目の動き: Back (-X) ✓ 正しく検出
- 時計回りパターン: Right → Back → Left → Front ✓ 正しい順序

#### 2. 速度収束の遅延（Device）

**問題**: PCは速度が速やかに0に収束するが、Deviceは遅い

**原因**: flow_noiseパラメータの不一致
- PC (replay.cpp): `flow_noise = 0.1f`
- Device (eskf.hpp default): `flow_noise = 0.5f`

**修正**: デバイスのデフォルト値を0.1に変更

```cpp
// components/stampfly_eskf/include/eskf.hpp
cfg.flow_noise = 0.1f;  // m/s (PCデバッグ済み: フローを信頼)
```

**結果**:
- Device最終速度: 12 cm/s → **1.2 cm/s** (90%改善)
- PC vs Device誤差: **2.2 cm** ✓ 良好な一致

#### 3. 軌跡スケールが実際の半分

**問題**: 検出された四角形の辺が実際の約半分

**修正**: flow_scaleを2倍に変更（0.08 → 0.16）

```cpp
// 全ファイルで修正
constexpr float flow_scale = 0.16f;  // 実測から2倍に修正
```

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `components/stampfly_eskf/eskf.cpp` | 軸マッピング修正、flow_scale=0.16 |
| `components/stampfly_eskf/include/eskf.hpp` | flow_noise=0.1 |
| `tools/eskf_debug/eskf_pc.cpp` | 軸マッピング修正、flow_scale=0.16 |
| `tools/eskf_debug/replay.cpp` | flow_scale=0.16 |
| `main/main.cpp` | flow_scale=0.16 |

### テスト結果サマリー

| 項目 | 修正前 | 修正後 |
|------|--------|--------|
| 軸方向 | XY入れ替わり + 符号逆 | ✓ 正しい |
| PC vs Device誤差 | - | 2.2 cm |
| Device速度収束 | 12 cm/s残留 | 1.2 cm/s |
| 軌跡スケール | 約50% | 調整中 (flow_scale=0.16) |

### 次のステップ

- [ ] flow_scale=0.16でのテスト（軌跡スケール検証）
- [ ] 地磁気有効化でのYaw推定検証

---

## ESKFパラメータ自動最適化 ✅ 完了 (2025-12-27)

### 実施内容

20cm四方移動テストの再現精度を最大化するため、ESKFパラメータの自動最適化システムを構築。

### 作成したツール

#### 1. optimize_params.py（新規作成）

グリッドサーチによるパラメータ最適化ツール:

```python
# 使用方法
python3 optimize_params.py <input.bin> [--target_range=0.20] [--extended]

# 最適化パラメータ
- flow_noise: フロー観測ノイズ
- accel_noise: 加速度観測ノイズ
- gyro_noise: ジャイロ観測ノイズ
- tof_noise: ToF観測ノイズ
- flow_rad_per_pixel: フロー校正係数
```

**コスト関数**:
```python
# 非対称ペナルティ: 20cm未達を3倍のペナルティ
if x_range_err < 0:
    x_range_cost = abs(x_range_err) * 3.0  # 未達に厳しい
else:
    x_range_cost = x_range_err

total_cost = range_cost * 4.0 + return_cost * 2.0 + att_cost * 0.5
```

#### 2. replay.cpp 拡張

コマンドライン引数でパラメータを動的に指定可能に:

```bash
./eskf_replay input.bin output.csv \
    --flow_noise=0.05 \
    --accel_noise=0.05 \
    --gyro_noise=0.001 \
    --tof_noise=0.007 \
    --flow_rad_per_pixel=0.00222 \
    --flow_scale_x=0.943 \
    --flow_scale_y=1.015 \
    --quiet  # JSON出力（自動化用）
```

**追加された出力メトリクス**:
```json
{
  "pos_x_range": 0.202,
  "pos_y_range": 0.200,
  "final_dist": 0.023,
  "roll": 0.5,
  "pitch": -0.3,
  "yaw": 2.1
}
```

### 最適化結果

#### 発見した問題: X/Y軸感度の非対称性

同じ`flow_rad_per_pixel`でもX軸とY軸で検出範囲が異なる:

| flow_rad_per_pixel | X範囲 | Y範囲 | 差 |
|-------------------|-------|-------|-----|
| 0.00205 (デフォルト) | 19.7cm | 18.3cm | 1.4cm |
| 0.00222 | 21.3cm | 19.7cm | 1.6cm |

**解決策**: 軸別スケーリングを導入

```cpp
// eskf.hpp
cfg.flow_cam_to_body[0] = 0.943f;  // X軸スケール (c2b_xx)
cfg.flow_cam_to_body[3] = 1.015f;  // Y軸スケール (c2b_yy)
```

#### 最終パラメータ

```cpp
// components/stampfly_eskf/include/eskf.hpp

// 20cm四方移動テストで最適化済み (2025-12-27)
cfg.gyro_noise = 0.001f;
cfg.accel_noise = 0.05f;
cfg.tof_noise = 0.007f;
cfg.mag_noise = 0.1f;
cfg.flow_noise = 0.05f;
cfg.accel_att_noise = 0.7f;

// フロー校正: 理論値(0.00205) × 1.083 = 0.00222
cfg.flow_rad_per_pixel = 0.00222f;

// 軸別スケーリング
cfg.flow_cam_to_body[0] = 0.943f;  // X軸
cfg.flow_cam_to_body[3] = 1.015f;  // Y軸
```

#### 検証結果

| 項目 | デフォルト | 最適化後 | 改善 |
|------|-----------|---------|------|
| X範囲 | 19.7cm | **20.2cm** | 目標20cmに到達 |
| Y範囲 | 18.3cm | **20.0cm** | 目標20cmに到達 |
| 原点復帰誤差 | 5.0cm | **2.3cm** | 54%改善 |
| 総軌跡距離 | - | 86.7cm | (目標80cm比 +8%) |

### トレードオフ分析

**軌跡総距離80cm vs 各辺20cm**:

| 目標 | flow_rad_per_pixel | X範囲 | Y範囲 | 総距離 |
|------|-------------------|-------|-------|--------|
| 80cm総距離 | 0.00202 | 18.6cm | 18.4cm | 79.9cm |
| **20cm各辺** | **0.00222** | **20.2cm** | **20.0cm** | **86.7cm** |

→ ユーザー選択により「20cm各辺」を優先

### 変更ファイル

| ファイル | 変更内容 |
|---------|---------|
| `tools/eskf_debug/optimize_params.py` | 新規作成: グリッドサーチ最適化 |
| `tools/eskf_debug/replay.cpp` | パラメータ引数追加、--quiet/JSON出力 |
| `components/stampfly_eskf/include/eskf.hpp` | 最適化済みデフォルト値 |

---

## V1パケット形式廃止 ✅ 完了 (2025-12-28)

### 概要

バイナリログ形式をV2 (128バイト) に統一し、レガシーV1 (64バイト) コードを無効化。

### 背景

- V1 (64バイト): センサデータのみ、ヘッダ 0xAA 0x55
- V2 (128バイト): センサデータ + ESKF推定結果、ヘッダ 0xAA 0x56

V2がデフォルトとなり、V1は実質使用されていなかったため、コードを整理。

### 変更内容

#### 1. replay.cpp (PC版ESKFリプレイ)

```cpp
// コメントアウト (#if 0 ... #endif)
- BinaryLogPacketV1 構造体
- verify_checksum_v1() 関数
- detect_log_format() のV1検出 (0xAA 0x55)
- load_log_file() のV1パース分岐
```

#### 2. cli.hpp (デバイス側CLI)

```cpp
// コメントアウト
- BinaryLogPacketV1 構造体
- using BinaryLogPacket = BinaryLogPacketV1; エイリアス
- isBinlogEnabled(), setBinlogEnabled(), outputBinaryLog()
- binlog_enabled_ メンバ変数
```

#### 3. cli.cpp

```cpp
// コメントアウト
- outputBinaryLog() V1関数
- cmd_binlog() 内の setBinlogEnabled() 呼び出し
```

#### 4. main.cpp

```cpp
// 簡略化
// 修正前: V1/V2分岐ロジック
if (g_cli.isBinlogEnabled() || g_cli.isBinlogV2Enabled()) {
    if (g_cli.isBinlogV2Enabled()) {
        g_cli.outputBinaryLogV2();
    } else {
        g_cli.outputBinaryLog();
    }
}

// 修正後: V2のみ
if (g_cli.isBinlogV2Enabled()) {
    g_cli.outputBinaryLogV2();
}
```

### ビルド確認

| ターゲット | 結果 |
|-----------|------|
| eskf_replay (PC) | ✅ ビルド成功 |
| デバイスファームウェア | ✅ ビルド成功 |
| 動作テスト (flow01.bin) | ✅ 正常動作 |

### 今後

- V1コードは `#if 0` でコメントアウト済み
- 完全削除は将来のクリーンアップ時に実施

---

## 今後の拡張予定

### センサーヘルスチェック関連

1. **ESKFモード表示**
   - Telemetryパケットに現在のESKFモード（FULL/NO_MAG/ALTITUDE_ONLY等）を含める
   - ブラウザUIでモード表示

2. **CLIコマンド拡張**
   - `sensor health`: 各センサーのヘルス状態を表示
   - `eskf mode`: 現在のESKFモードを表示

3. **LED/ブザー通知**
   - センサー異常時の視覚・音声フィードバック
   - 例: ToF異常→青点滅、Flow異常→紫点滅

4. **共分散上限設定**
   - 長時間のセンサー欠落時に共分散が発散しないようキャップ
   - `enforceCovarianceLimits()` 関数をESKFに追加

5. **センサー復帰時のスムージング**
   - 長時間スキップ後のセンサー復帰時、急激な補正を避けるためカルマンゲインを段階的に上げる

### WiFi Telemetry関連

1. **双方向通信**
   - WebSocketでパラメータ変更コマンドを受信
   - PIDゲインのリアルタイム調整

2. ~~**ログ記録**~~ ✅ 実装済み (2025-12-31)
   - ~~ブラウザでのログダウンロード機能~~ → CSVエクスポート実装
   - WebSocket経由でバイナリログ転送（未実装）

3. ~~**3D可視化**~~ → 2Dビュー方式に変更
   - ~~Three.jsによる機体姿勢の3D表示~~ → 姿勢指示器 + 上面/側面ビューで代替
   - ~~軌跡表示~~ → 上面/側面ビューに軌跡表示実装済み

詳細設計: [docs/eskf_sensor_health.md](eskf_sensor_health.md), [docs/telemetry_design.md](telemetry_design.md)

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2025-12-31 | テレメトリv2.1: パケット96→108バイト拡張（地磁気追加）、HTMLビューア改善（色統一、オートフィット、姿勢指示器、座標軸回転）、サイドビュー原点中央化 |
| 2025-12-29 | センサーヘルスチェック実装: 各タスクにヘルスフラグ追加、ESKF観測更新にガード追加、異常センサーからの更新を防止 |
| 2025-12-29 | ESKF発散問題修正: g_eskf_readyフラグで起動制御、発散時の常時リセット（バグ修正）、設計ドキュメント追加 |
| 2025-12-29 | WiFi WebSocketテレメトリ実装: APSTA + ESP-NOW並行動作、50Hz姿勢データ配信、ブラウザUI |
| 2025-12-29 | 飛行制御基盤実装: ControlTask(400Hz)追加、モータCLIコマンド(test/all/stop)、ARMトグル(立ち上がりエッジ検出)、モータドライバarm/disarm連携 |
| 2025-12-29 | スロットル正規化修正: 上半分のみ使用(2048-4095→0〜1)、負値クリップ |
| 2025-12-29 | LED機能拡張: 明るさNVS保存、低バッテリー(3.4V未満)シアン警告、FLYING色を黄色に変更 |
| 2025-12-29 | 開発者ドキュメント作成: docs/developer_guide.md (main.cpp構造、タスク構成、PID制御例) |
| 2025-12-28 | 疎行列展開版ESKFバグ修正: updateMagのS行列計算でH行列のインデックス誤り(H0x→H1x/H2x)を修正、PC版とデバイス版で同一の疎行列実装に統一 |
| 2025-12-28 | updateAccelAttitudeWithGyro廃止: updateAccelAttitudeに統一、k_adaptiveはconfig_から取得（デフォルト0.0f） |
| 2025-12-28 | visualize_eskf.py: --pcオプション追加（PCシミュレーション結果のみ表示） |
| 2025-12-28 | 姿勢推定400Hz化: updateAccelAttitudeを毎サイクル実行（50Hz→400Hz） |
| 2025-12-28 | ESP Timer導入: 正確な2.5ms周期(400Hz)をESP Timerで実現、セマフォによるIMUタスク同期 |
| 2025-12-28 | ESKF 400Hz化: 4サンプル平均を廃止し毎サイクルpredict実行、dt=0.0025s、Flow=100Hz維持 |
| 2025-12-28 | 地磁気Yaw推定検証完了: 360度回転テストで正常動作確認、「次のステップ」から地磁気関連項目を削除 |
| 2025-12-28 | V1パケット形式廃止: replay.cpp/cli.hpp/cli.cpp/main.cppでV1コードをコメントアウト、V2のみに統一 |
| 2025-12-27 | ESKFパラメータ自動最適化: optimize_params.py作成、replay.cppパラメータ引数追加、flow_rad_per_pixel=0.00222、軸別スケーリング導入、X=20.2cm/Y=20.0cm達成 |
| 2025-12-01 | 動的四角形テスト: 軸マッピング修正(XY入替+符号)、flow_noise=0.1、flow_scale=0.16 |
| 2025-12-01 | フローオフセット除去: flow_dx/dy_offset=0に設定、静止ドリフト2.2cm→0.2cm (90%改善) |
| 2025-12-01 | PC vs Device ESKFデバッグ: レースコンディション対策(data_readyフラグ)、tof_noise調整、PC版reset()/setGyroBias()追加 |
| 2025-12-01 | ESKF設計指針追加: Baro NED変換方式、メンバ変数による行列管理、PC/デバイス版同期 |
| 2025-12-01 | センサデータ整合性検証: 更新レート修正(Flow/AccelAtt)、V2パケットにbaro_ref_alt追加 |
| 2025-12-01 | デバッグツール整理: visualize_device_log.pyに統合、V2デフォルト化、archive/移動 |
| 2025-12-01 | 地磁気キャリブレーション実装: Hard Iron/Soft Iron補正、CLIコマンド(magcal)、NVS保存 |
| 2025-12-01 | 実機ESKFデバッグ: mag_enabled処理修正、Yaw固定モード実装、test_v17でPosition安定化達成 |
| 2025-11-30 | PCデバッグ済みESKFパラメータを実機コードに完全反映（オフセット、Q/R、updateFlowWithGyro） |
| 2025-11-30 | 自動キャリブレーションフレームワーク設計完了（docs/auto_calibration_design.md） |
| 2025-11-30 | Q/Rパラメータ検証：センサデータから推定した値と比較、デフォルト値が安定と結論 |
| 2025-11-30 | フローオフセットキャリブレーション：閉ループ推定法でオフセット決定、エラー5.1cm達成 |
| 2025-11-30 | フロー最終チューニング：ジャイロ補償(回帰分析)、flow_scale=0.08、閉ループエラー22.4cm達成、本体コードに適用 |
| 2025-11-30 | フロー軸変換修正：実測データ分析でX/Y軸入れ替えを発見・修正、閉ループエラー25.6cm達成 |
| 2025-11-30 | 四角形移動テスト：Body→NED座標変換実装、flow_scale=0.1キャリブレーション |
| 2025-11-30 | PC版ESKFチューニング完了（重力補正、ToF傾き閾値、フローパラメータ）、静的・動的テストで検証 |
| 2025-11-30 | ESKFデバッグ環境完成、binlogコマンド追加、USB CDC line ending問題解決 |
| 2025-11-28 | ESKF計算負荷問題特定・暫定対策：predict頻度を400Hz→100Hzに制限、IMUデータ平均化で安定動作確認 |
| 2025-11-28 | 加速度単位修正：g→m/s²変換追加（×9.81）、teleplot出力最適化（バッファ一括出力） |
| 2025-11-28 | センサ軸変換・重力補償修正完了：BMI270/BMM150/PMW3901→NED座標系変換、ESKF重力補償符号修正、AttitudeEstimator姿勢計算修正 |
| 2025-11-28 | GitHub stampfly-eskf-estimatorと比較・改善：Mahalanobis距離棄却、共分散対称性強制、加速度姿勢補正追加 |
| 2025-11-28 | ESKFスタックメモリ問題解決：一時行列をメンバ変数化、全センサからのESKF更新を有効化 |
| 2025-11-28 | 正面ToF未接続対応、OutlierDetector緩和、ToF読み取りシーケンス修正、全センサ値取得確認 |
| 2025-11-28 | CLI teleplotコマンド追加、sensorコマンドを実データ対応 |
| 2025-11-28 | 実機テスト完了、各種バグ修正、CLI動作確認 |
| 2025-11-27 | Phase 4 タスク統合完了、ビルド成功 |
