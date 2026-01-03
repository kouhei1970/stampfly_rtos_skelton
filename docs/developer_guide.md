# StampFly 開発者ガイド

このドキュメントは、StampFly RTOSスケルトンに機能を追加する開発者向けのガイドです。

## 目次

1. [プロジェクト構造](#プロジェクト構造)
2. [タスク構成とタイミング](#タスク構成とタイミング)
3. [データフロー](#データフロー)
4. [飛行制御（Rate Control）](#飛行制御rate-control)
5. [安全機能](#安全機能)
6. [LED表示システム](#led表示システム)
7. [CLIコマンド](#cliコマンド)
8. [バイナリログとESKFデバッグ](#バイナリログとeskfデバッグ)
9. [設定パラメータ](#設定パラメータ)
10. [APIリファレンス](#apiリファレンス)
11. [座標系](#座標系)

---

## プロジェクト構造

### ディレクトリ構成

```
stampfly_rtos_skelton/
├── main/
│   ├── main.cpp              # エントリポイント（薄く保つ）
│   ├── config.hpp            # 全設定パラメータ
│   ├── globals.hpp/cpp       # グローバル変数定義
│   ├── init.cpp              # 初期化関数群
│   ├── rate_controller.hpp   # レート制御器定義
│   └── tasks/                # タスク実装（分離済み）
│       ├── tasks.hpp         # タスクプロトタイプ
│       ├── tasks_common.hpp  # 共通インクルード
│       ├── imu_task.cpp      # IMU + ESKF (400Hz)
│       ├── control_task.cpp  # PID制御 (400Hz)
│       ├── optflow_task.cpp  # 光学フロー (100Hz)
│       ├── mag_task.cpp      # 地磁気 (100Hz)
│       ├── baro_task.cpp     # 気圧 (50Hz)
│       ├── tof_task.cpp      # ToF (30Hz)
│       ├── comm_task.cpp     # ESP-NOW (50Hz)
│       ├── power_task.cpp    # 電源監視 (10Hz)
│       ├── led_task.cpp      # LED更新 (30Hz)
│       ├── button_task.cpp   # ボタン (50Hz)
│       ├── cli_task.cpp      # USB CLI
│       └── telemetry_task.cpp # WebSocket (50Hz)
├── components/               # コンポーネント群
├── docs/                     # ドキュメント
└── tools/                    # 開発ツール
```

### コンポーネント命名規則

| Prefix | レイヤー | FreeRTOS依存 | 例 |
|--------|---------|-------------|-----|
| `sf_hal_*` | HALドライバ | 極力なし | bmi270, motor, led |
| `sf_algo_*` | アルゴリズム | なし | eskf, pid, filter |
| `sf_svc_*` | サービス | あり | state, cli, led_manager |

### main.cpp構造

main.cppは**薄く**保ち、主要ロジックはタスクファイルに分離：

| セクション | 行 | 内容 |
|-----------|---:|------|
| ヘルパー関数 | 64-166 | 姿勢初期化、binlogコールバック |
| イベントハンドラ | 210-329 | ボタン、コントローラパケット |
| startTasks() | 332-411 | 全タスク生成 |
| app_main() | 417-839 | 4段階起動シーケンス |

---

## タスク構成とタイミング

### CPUコア割り当て

| Core | タスク | 理由 |
|------|--------|------|
| Core 1 | IMUTask, ControlTask, OptFlowTask | 高頻度・リアルタイム制御 |
| Core 0 | その他すべて | I2Cセンサー、通信、UI |

### タスク優先度（高い順）

| 優先度 | タスク | 周波数 | 役割 |
|--------|--------|--------|------|
| 24 | IMUTask | 400Hz | センサー読取 + ESKF予測 |
| 23 | ControlTask | 400Hz | PID制御 + モーター出力 |
| 20 | OptFlowTask | 100Hz | 光学フロー読取 |
| 18 | MagTask | 100Hz | 地磁気読取 |
| 16 | BaroTask | 50Hz | 気圧読取 |
| 15 | CommTask | 50Hz | ESP-NOW通信 |
| 14 | ToFTask | 30Hz | ToF距離測定 |
| 12 | PowerTask | 10Hz | 電源監視 + 低バッテリー警告 |
| 10 | ButtonTask | 50Hz | ボタン入力 |
| 8 | LEDTask | 30Hz | LED更新 |
| 5 | CLITask | - | USBシリアル |

### 起動シーケンス（4段階）

| Phase | LED | 時間 | 内容 |
|-------|-----|------|------|
| 1 | 白・ブリーズ | 3秒 | 地面配置確認 |
| 2 | 青・ブリーズ | - | センサータスク起動待ち |
| 3 | マゼンタ・点滅 | MAX 10秒 | 全センサー安定判定 |
| 4 | 緑・点灯 | - | Ready (IDLE状態) |

---

## データフロー

```
[センサータスク]              [ESKF (IMUTask内)]           [ControlTask]
     │                              │                          │
 IMUTask ──400Hz──→ predict() ──→ 姿勢推定 ──→ セマフォ ──→ PID制御
 MagTask ──────────→ updateMag()      │                          │
 BaroTask ─────────→ updateBaro()     │                          │
 ToFTask ──────────→ updateToF()      ↓                          ↓
 OptFlowTask ──────→ updateFlow() → StampFlyState ←──────── モーター出力
                                      ↑
                              CommTask (入力)
```

### StampFlyState（共有状態）

| カテゴリ | データ |
|---------|--------|
| センサーデータ | accel, gyro, mag, baro, tof, flow |
| 推定状態 | position, velocity, roll, pitch, yaw |
| 制御入力 | throttle, roll, pitch, yaw (正規化済み) |
| システム状態 | flight_state, error_code, pairing_state |

---

## 飛行制御（Rate Control）

### 現在の実装

ControlTask（`main/tasks/control_task.cpp`）にPID角速度制御が実装済み：

```cpp
// 制御ループ概要
while (true) {
    xSemaphoreTake(g_control_semaphore, ...);  // IMUTaskから同期

    // 1. コントローラ入力取得
    state.getControlInput(throttle, roll_cmd, pitch_cmd, yaw_cmd);

    // 2. 目標角速度計算 [rad/s]
    float roll_rate_target = roll_cmd * g_rate_controller.roll_rate_max;
    float pitch_rate_target = pitch_cmd * g_rate_controller.pitch_rate_max;
    float yaw_rate_target = yaw_cmd * g_rate_controller.yaw_rate_max;

    // 3. IMUデータ取得
    state.getIMUData(accel, gyro);

    // 4. 衝撃検出（省略 - 安全機能セクション参照）

    // 5. PID制御
    float roll_out = g_rate_controller.roll_pid.update(
        roll_rate_target, gyro.x, dt);
    float pitch_out = g_rate_controller.pitch_pid.update(
        pitch_rate_target, gyro.y, dt);
    float yaw_out = g_rate_controller.yaw_pid.update(
        yaw_rate_target, gyro.z, dt);

    // 6. モーターミキサー
    g_motor.setMixerOutput(throttle, roll_out, pitch_out, yaw_out);
}
```

### PIDゲイン設定（config.hpp）

```cpp
namespace rate_control {
// 感度（スティック最大時の目標角速度 [rad/s]）
inline constexpr float ROLL_RATE_MAX = 5.0f;   // ~286 deg/s
inline constexpr float PITCH_RATE_MAX = 5.0f;
inline constexpr float YAW_RATE_MAX = 5.0f;

// Roll PID
inline constexpr float ROLL_RATE_KP = 0.65f;
inline constexpr float ROLL_RATE_TI = 0.7f;    // 積分時間 [s]
inline constexpr float ROLL_RATE_TD = 0.01f;   // 微分時間 [s]

// Pitch PID
inline constexpr float PITCH_RATE_KP = 0.95f;
inline constexpr float PITCH_RATE_TI = 0.7f;
inline constexpr float PITCH_RATE_TD = 0.025f;

// Yaw PID
inline constexpr float YAW_RATE_KP = 3.0f;
inline constexpr float YAW_RATE_TI = 0.8f;
inline constexpr float YAW_RATE_TD = 0.01f;

// 共通
inline constexpr float PID_ETA = 0.125f;       // 不完全微分係数
inline constexpr float OUTPUT_LIMIT = 3.7f;    // 出力制限 [V]
}
```

### モーターミキシング（X-Quad）

```
          前方
           ▲
    FL(M4)     FR(M1)
      CW  ╲ ╱  CCW
           X
     CCW  ╱ ╲  CW
    RL(M3)     RR(M2)
          後方

M1 (FR) = Thrust - Roll + Pitch + Yaw
M2 (RR) = Thrust - Roll - Pitch - Yaw
M3 (RL) = Thrust + Roll - Pitch + Yaw
M4 (FL) = Thrust + Roll + Pitch - Yaw
```

---

## 安全機能

### 衝撃検出（自動Disarm）

`control_task.cpp` で実装。飛行中に衝撃を検出すると即座にDisarm。

| 検出タイプ | 閾値 | 検出対象 |
|-----------|------|---------|
| HIGH_ACCEL | 3G (29.4 m/s²) | 衝突の衝撃 |
| HIGH_GYRO | 800 deg/s | 急激な回転・転倒 |

```cpp
// config.hpp
namespace safety {
inline constexpr float IMPACT_ACCEL_THRESHOLD_G = 3.0f;
inline constexpr float IMPACT_GYRO_THRESHOLD_DPS = 800.0f;
inline constexpr int IMPACT_COUNT_THRESHOLD = 2;  // 連続検出回数
}
```

**動作:**
1. 閾値超過を連続2回検出
2. 自動Disarm（モーター停止）
3. エラートーン + 赤LED点滅5秒
4. ログ出力: `CRASH DETECTED [HIGH_ACCEL/HIGH_GYRO]`

### 低バッテリー警告

`power_task.cpp` で実装。**警告のみ**で飛行は継続可能。

| 項目 | 値 |
|------|-----|
| 閾値 | 3.4V |
| LED | シアン・遅い点滅 |
| ブザー | 初回 + 10秒間隔で繰り返し |

**重要**: 低バッテリーはエラー状態にせず、パイロットが自主的に着陸。

### センサー安定判定（起動時）

Phase 3で全6センサーの安定を確認：

| センサー | 判定基準 |
|---------|---------|
| Accel | 標準偏差 < 0.1 m/s² |
| Gyro | 標準偏差 < 0.01 rad/s |
| Mag | 標準偏差 < 5.0 μT |
| Baro | 標準偏差 < 0.5 m |
| ToF | 標準偏差 < 0.02 m |
| Flow | dx+dy標準偏差 < 5.0 |

---

## LED表示システム

### ハードウェア構成

| LED | GPIO | チャンネル | 用途 |
|-----|------|-----------|------|
| MCU LED | 21 | SYSTEM | 起動/エラー/ペアリング |
| BODY_TOP | 39-0 | FLIGHT | 飛行状態 |
| BODY_BOTTOM | 39-1 | STATUS | センサー/バッテリー |

### 優先度システム

| 優先度 | 用途 |
|--------|------|
| 0: BOOT | 起動シーケンス |
| 1: CRITICAL_ERROR | センサー異常 |
| 2: LOW_BATTERY | 低バッテリー |
| 3: DEBUG_ALERT | デバッグ通知 |
| 4: PAIRING | ペアリング中 |
| 5: FLIGHT_STATE | 飛行状態 |
| 6: DEFAULT | 待機状態 |

### 飛行状態LED（FLIGHT チャンネル）

| 状態 | 色 | パターン |
|------|-----|---------|
| INIT | 青 | ブリーズ |
| CALIBRATING | 黄 | 高速点滅 |
| IDLE | 緑 | 点灯 |
| ARMED | 緑 | 遅い点滅 |
| FLYING | 黄 | 点灯 |
| ERROR | 赤 | 高速点滅 |

詳細: [docs/led_display.md](led_display.md)

---

## CLIコマンド

USB Serial (115200bps) で接続。

### システム

| コマンド | 説明 |
|----------|------|
| `help` | コマンド一覧 |
| `status` | システム状態表示 |
| `reset` | システムリセット |

### センサー

| コマンド | 説明 |
|----------|------|
| `sensor all` | 全センサーデータ |
| `sensor imu` | 加速度・ジャイロ |
| `sensor mag` | 地磁気 |
| `sensor baro` | 気圧・高度 |
| `sensor tof` | ToF距離 |
| `sensor flow` | 光学フロー |

### 制御

| コマンド | 説明 |
|----------|------|
| `ctrl` | コントローラ入力（1回） |
| `ctrl watch` | 入力監視（10秒間） |
| `motor test 1 50` | モーター1を50%で回転 |
| `motor stop` | 全モーター停止 |

### ログ

| コマンド | 説明 |
|----------|------|
| `binlog on` | 400Hzバイナリログ開始 |
| `binlog off` | ログ停止 |
| `binlog freq 100` | 周波数設定 (10-1000Hz) |
| `loglevel info` | ESP_LOGレベル設定 |

### その他

| コマンド | 説明 |
|----------|------|
| `debug on` | デバッグモード（ARM時エラー無視） |
| `led brightness 64` | LED明るさ (0-255) |
| `magcal start/stop` | 磁気キャリブレーション |
| `pair start` | ペアリングモード |

---

## バイナリログとESKFデバッグ

### ワークフロー

```bash
# 1. ログ取得 (60秒)
python tools/scripts/log_capture.py capture \
  -p /dev/tty.usbmodem* -o logs/flight.bin -d 60

# 2. 可視化（バイナリ直接）
python tools/scripts/visualize_eskf.py logs/flight.bin --all

# 3. PCでESKFリプレイ
./tools/eskf_debug/build/eskf_replay logs/flight.bin logs/result.csv

# 4. PC vs デバイス比較
python tools/scripts/visualize_eskf.py logs/result.csv --compare
```

### LogPacket構造 (128バイト)

| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0 | 2B | ヘッダ (0xAA 0x56) |
| 2 | 4B | タイムスタンプ [ms] |
| 6 | 24B | IMU (accel 12B + gyro 12B) |
| 30 | 12B | 地磁気 |
| 42 | 8B | 気圧 + 高度 |
| 50 | 8B | ToF (bottom + front) |
| 58 | 5B | Flow (dx, dy, quality) |
| 63 | 52B | ESKF推定値 |
| 115 | 1B | ステータス |
| 116 | 11B | 予約 |
| 127 | 1B | チェックサム (XOR) |

---

## 設定パラメータ

### config.hpp 名前空間一覧

| 名前空間 | 内容 |
|---------|------|
| `config` | GPIO、タスク優先度、スタックサイズ |
| `timing` | タスク周期 |
| `eskf` | Q/R行列、初期共分散 |
| `stability` | センサー安定判定閾値 |
| `lpf` | ローパスフィルタ設定 |
| `rate_control` | PIDゲイン、感度 |
| `safety` | 衝撃検出閾値 |
| `led` | LED設定 |

---

## APIリファレンス

### StampFlyState

```cpp
auto& state = stampfly::StampFlyState::getInstance();

// IMUデータ
state.getIMUData(accel, gyro);         // フィルタ済み生データ
state.getIMUCorrected(accel, gyro);    // バイアス補正済み（制御用）

// 姿勢
state.getAttitudeEuler(roll, pitch, yaw);  // [rad]

// 位置・速度
auto pos = state.getPosition();   // [m] NED
auto vel = state.getVelocity();   // [m/s] NED

// 制御入力
state.getControlInput(throttle, roll, pitch, yaw);
// throttle: 0.0~1.0, roll/pitch/yaw: -1.0~+1.0

// システム状態
auto fs = state.getFlightState();  // INIT/IDLE/ARMED/FLYING/ERROR
auto err = state.getErrorCode();   // NONE/SENSOR_*/LOW_BATTERY/...
```

### LEDManager

```cpp
auto& led = stampfly::LEDManager::getInstance();

// チャンネル表示要求（優先度付き）
led.requestChannel(LEDChannel::SYSTEM, LEDPriority::BOOT,
                   LEDPattern::BREATHE, 0xFFFFFF);

// タイムアウト付き（5秒後に自動解除）
led.requestChannel(LEDChannel::STATUS, LEDPriority::DEBUG_ALERT,
                   LEDPattern::BLINK_FAST, 0xFFFF00, 5000);

// 解除
led.releaseChannel(LEDChannel::SYSTEM, LEDPriority::BOOT);

// イベント通知
led.onFlightStateChanged(FlightState::ARMED);
led.onBatteryStateChanged(voltage, is_low);
```

### MotorDriver

```cpp
// 推奨: ミキサー経由
g_motor.setMixerOutput(thrust, roll, pitch, yaw);

// 低レベル: 個別モーター
g_motor.setMotor(MotorDriver::MOTOR_FR, value);  // 0.0-1.0

// Arm/Disarm
g_motor.arm();
g_motor.disarm();
```

---

## 座標系

**NED座標系**（North-East-Down）を採用：

| 軸 | 方向 | 正の向き |
|----|------|---------|
| X | 前方 | 機首方向 |
| Y | 右方 | 右翼方向 |
| Z | 下方 | 地面方向 |

| 回転 | 軸 | 正の向き |
|------|-----|---------|
| Roll | X軸周り | 右翼下げ |
| Pitch | Y軸周り | 機首上げ |
| Yaw | Z軸周り | 右回り |

---

## 注意事項

1. **タスク分離**: 制御ロジックは`main/tasks/`に実装。main.cppは薄く保つ。

2. **スレッドセーフティ**: `StampFlyState`はmutex保護済み。getter/setterを使用。

3. **タイミング**: IMUTask/ControlTaskは400Hz。Core 1で重い処理を避ける。

4. **安全機能**: 衝撃検出は自動Disarm。低バッテリーは警告のみ。

5. **LED優先度**: 複数の表示要求は優先度で制御。高優先度が表示される。
