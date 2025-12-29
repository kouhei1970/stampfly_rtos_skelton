# ESKF PCデバッグ環境 実装計画

## 1. 概要

ESKFが実機で正しく動作しない問題をデバッグするため、以下の環境を構築する：

1. **実機**: センサデータをUSBシリアル経由でバイナリログ出力
2. **PC**: ログを受信・保存し、同じESKFコードを実行して結果を検証
3. **可視化**: Pythonで推定結果をグラフ化・解析

```
┌─────────────┐     USB Serial      ┌─────────────┐
│  StampFly   │ ──────────────────> │     PC      │
│  (実機)     │   Binary Log        │             │
│             │   60B @ 100Hz       │  ┌────────┐ │
│ - Sensors   │                     │  │ログ保存│ │
│ - ESKF      │                     │  └────────┘ │
└─────────────┘                     │      ↓      │
                                    │  ┌────────┐ │
                                    │  │C++ ESKF│ │
                                    │  │ Replay │ │
                                    │  └────────┘ │
                                    │      ↓      │
                                    │  ┌────────┐ │
                                    │  │ Python │ │
                                    │  │可視化  │ │
                                    │  └────────┘ │
                                    └─────────────┘
```

---

## 2. バイナリログ形式

### 2.1 パケットバージョン

| バージョン | サイズ | ヘッダー | 内容 | 状態 |
|-----------|--------|---------|------|------|
| V1 | 64 bytes | 0xAA 0x55 | センサデータのみ | **非推奨** |
| V2 | 128 bytes | 0xAA 0x56 | センサ + ESKF推定値 | **現行** |

> **注意**: 2024年12月以降、V1は非推奨です。すべてのツールはV2のみサポートします。

---

### 2.2 V2 パケット構造 (128 bytes) - 現行フォーマット

**ヘッダー**: `0xAA 0x56`

#### センサデータ部 (63 bytes)

| Offset | Size | Type | Field | Description | 単位 |
|--------|------|------|-------|-------------|------|
| 0 | 2 | uint8_t[2] | header | 0xAA, 0x56 | - |
| 2 | 4 | uint32_t | timestamp_ms | ブート後経過時間 | ms |
| 6 | 4 | float | accel_x | 加速度 X | m/s² |
| 10 | 4 | float | accel_y | 加速度 Y | m/s² |
| 14 | 4 | float | accel_z | 加速度 Z | m/s² |
| 18 | 4 | float | gyro_x | 角速度 X | rad/s |
| 22 | 4 | float | gyro_y | 角速度 Y | rad/s |
| 26 | 4 | float | gyro_z | 角速度 Z | rad/s |
| 30 | 4 | float | mag_x | 地磁気 X | uT |
| 34 | 4 | float | mag_y | 地磁気 Y | uT |
| 38 | 4 | float | mag_z | 地磁気 Z | uT |
| 42 | 4 | float | pressure | 気圧 | Pa |
| 46 | 4 | float | baro_alt | 気圧高度 | m |
| 50 | 4 | float | tof_bottom | ToF底面距離 | m |
| 54 | 4 | float | tof_front | ToF前方距離 | m |
| 58 | 2 | int16_t | flow_dx | OptFlow delta X | counts |
| 60 | 2 | int16_t | flow_dy | OptFlow delta Y | counts |
| 62 | 1 | uint8_t | flow_squal | OptFlow品質 (0-255) | - |

#### ESKF推定値部 (48 bytes)

| Offset | Size | Type | Field | Description | 単位 |
|--------|------|------|-------|-------------|------|
| 63 | 4 | float | pos_x | 位置 X (NED) | m |
| 67 | 4 | float | pos_y | 位置 Y (NED) | m |
| 71 | 4 | float | pos_z | 位置 Z (NED, 下向き正) | m |
| 75 | 4 | float | vel_x | 速度 X | m/s |
| 79 | 4 | float | vel_y | 速度 Y | m/s |
| 83 | 4 | float | vel_z | 速度 Z | m/s |
| 87 | 4 | float | roll | ロール角 | rad |
| 91 | 4 | float | pitch | ピッチ角 | rad |
| 95 | 4 | float | yaw | ヨー角 | rad |
| 99 | 4 | float | gyro_bias_z | ジャイロバイアス Z | rad/s |
| 103 | 4 | float | accel_bias_x | 加速度バイアス X | m/s² |
| 107 | 4 | float | accel_bias_y | 加速度バイアス Y | m/s² |

#### メタデータ部 (17 bytes)

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 111 | 1 | uint8_t | eskf_status | ESKF状態 (0=未初期化, 1=動作中) |
| 112 | 4 | float | baro_ref_alt | 気圧基準高度 (PCリプレイ用) |
| 116 | 11 | uint8_t[11] | reserved | 予約 (将来拡張用) |
| 127 | 1 | uint8_t | checksum | XOR (byte 2-126) |

**合計: 128 bytes**

#### Python struct フォーマット

```python
PACKET_FORMAT = '<2sI6f3f2f2f2hB3f3f3f3fB4s11sB'
# 2s  : header (2 bytes)
# I   : timestamp_ms (4 bytes)
# 6f  : accel_xyz + gyro_xyz (24 bytes)
# 3f  : mag_xyz (12 bytes)
# 2f  : pressure + baro_alt (8 bytes)
# 2f  : tof_bottom + tof_front (8 bytes)
# 2h  : flow_dx + flow_dy (4 bytes)
# B   : flow_squal (1 byte)
# 3f  : pos_xyz (12 bytes)
# 3f  : vel_xyz (12 bytes)
# 3f  : roll + pitch + yaw (12 bytes)
# 3f  : gyro_bias_z + accel_bias_xy (12 bytes)
# B   : eskf_status (1 byte)
# 4s  : baro_ref_alt as bytes (4 bytes)
# 11s : reserved (11 bytes)
# B   : checksum (1 byte)
```

#### C++ 構造体定義

```cpp
// components/stampfly_cli/include/cli.hpp
#pragma pack(push, 1)
struct BinaryLogPacketV2 {
    uint8_t header[2];      // 0xAA, 0x56
    uint32_t timestamp_ms;
    // IMU (24 bytes)
    float accel_x, accel_y, accel_z;  // [m/s²]
    float gyro_x, gyro_y, gyro_z;     // [rad/s]
    // Mag (12 bytes)
    float mag_x, mag_y, mag_z;        // [uT]
    // Baro (8 bytes)
    float pressure;                    // [Pa]
    float baro_alt;                    // [m]
    // ToF (8 bytes)
    float tof_bottom, tof_front;      // [m]
    // Flow (5 bytes)
    int16_t flow_dx, flow_dy;
    uint8_t flow_squal;
    // ESKF Position (12 bytes)
    float pos_x, pos_y, pos_z;        // [m] NED
    // ESKF Velocity (12 bytes)
    float vel_x, vel_y, vel_z;        // [m/s]
    // ESKF Attitude (12 bytes)
    float roll, pitch, yaw;           // [rad]
    // ESKF Biases (12 bytes)
    float gyro_bias_z;                // [rad/s]
    float accel_bias_x, accel_bias_y; // [m/s²]
    // Metadata (17 bytes)
    uint8_t eskf_status;
    float baro_ref_alt;               // [m]
    uint8_t reserved[11];
    uint8_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(BinaryLogPacketV2) == 128);
```

### 2.3 チェックサム計算

```cpp
uint8_t checksum = 0;
for (int i = 2; i < 127; i++) {
    checksum ^= packet[i];
}
// packet[127] = checksum;
```

### 2.4 出力レート

- **100Hz** (10ms周期) - IMUの400Hzからダウンサンプリング
- **転送速度**: 128B × 100Hz = 12.8KB/s (115200bpsで十分)

---

### 2.5 V1 パケット構造 (64 bytes) - 非推奨

> **警告**: V1は非推奨です。新規開発ではV2を使用してください。

**ヘッダー**: `0xAA 0x55`

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 2 | uint8_t[2] | header | 0xAA, 0x55 |
| 2 | 4 | uint32_t | timestamp_ms | ブート後経過時間 [ms] |
| 6-29 | 24 | float[6] | accel + gyro | IMUデータ |
| 30-41 | 12 | float[3] | mag | 地磁気 |
| 42-49 | 8 | float[2] | pressure + baro_alt | 気圧 |
| 50-57 | 8 | float[2] | tof_bottom + tof_front | ToF |
| 58-62 | 5 | int16_t[2] + uint8_t | flow | OptFlow |
| 63 | 1 | uint8_t | checksum | XOR (byte 2-62) |

V1チェックサム:
```cpp
for (int i = 2; i < 63; i++) checksum ^= packet[i];
```

---

## 3. 実装フェーズ

### Phase 1: 実機側 - バイナリログ出力 (CLI拡張)

**ファイル**: `components/stampfly_cli/`

1. `cli.hpp` に追加:
   - `binlog` コマンド登録
   - `outputBinaryLog()` メソッド
   - `BinaryLogPacket` 構造体定義

2. `cli.cpp` に追加:
   - `cmd_binlog()` ハンドラ (`binlog on/off`)
   - `outputBinaryLog()` 実装 (パケット構築・送信)

3. `main.cpp` 修正:
   - CLITaskにバイナリログ出力処理追加 (100Hz)

### Phase 2: PC側 - ログ受信・保存スクリプト

**ファイル**: `tools/scripts/log_capture.py`

機能:
- シリアルポートからバイナリログ受信
- パケット同期 (0xAA 0x55 検出)
- チェックサム検証
- バイナリファイル保存 (.bin)
- リアルタイム表示 (オプション)

使用方法:
```bash
python log_capture.py --port /dev/tty.usbmodem* --output sensor_log.bin --duration 60
```

### Phase 3: PC側 - C++ ESKF リプレイ環境

**ディレクトリ**: `tools/eskf_debug/`

```
tools/eskf_debug/
├── CMakeLists.txt          # CMakeビルド設定
├── stubs/
│   └── esp_err.h           # ESP-IDF型スタブ
├── eskf_pc.cpp             # ESKF実装 (元のeskf.cppをPC用に修正)
├── main.cpp                # 簡単なテスト
└── replay.cpp              # ログファイル再生・ESKF実行
```

**replay.cpp 処理フロー**:
1. バイナリログファイル読み込み
2. パケットごとにESKF更新
   - IMU: `eskf.predict(accel, gyro, dt)`
   - Baro: `eskf.updateBaro(altitude)` (50Hzでサブサンプル)
   - ToF: `eskf.updateToF(distance)` (30Hzでサブサンプル)
   - Mag: `eskf.updateMag(mag)` (10Hzでサブサンプル)
   - Flow: `eskf.updateFlow(...)` (条件付き)
3. 各ステップの状態をCSV出力
4. 共分散行列もデバッグ用に出力可能

### Phase 4: PC側 - Python可視化スクリプト

**ファイル**: `tools/scripts/visualize_eskf.py`

機能:
- ESKFリプレイ結果CSVの読み込み
- センサ生データとの比較プロット
- 状態推定値の時系列プロット:
  - 位置 (x, y, z)
  - 速度 (vx, vy, vz)
  - 姿勢 (roll, pitch, yaw)
  - バイアス (gyro_bias, accel_bias)
- 共分散の時間変化プロット
- イノベーション (予測誤差) のプロット

**出力グラフ例**:
```
┌─────────────────────────────────────┐
│ Roll/Pitch/Yaw vs Time             │
│  ─ ESKF推定値                       │
│  ─ 加速度から計算した姿勢            │
│  ─ ジャイロ積分姿勢                  │
└─────────────────────────────────────┘
┌─────────────────────────────────────┐
│ Altitude vs Time                    │
│  ─ ESKF推定値                       │
│  ─ Baro生値                         │
│  ─ ToF生値                          │
└─────────────────────────────────────┘
```

---

## 4. ファイル構成 (完成後)

```
stampfly_rtos_skelton/
├── components/
│   └── stampfly_cli/
│       ├── include/cli.hpp      # BinaryLogPacket追加
│       └── cli.cpp              # cmd_binlog, outputBinaryLog追加
├── main/
│   └── main.cpp                 # CLITaskにバイナリログ処理追加
├── tools/
│   ├── eskf_debug/
│   │   ├── CMakeLists.txt
│   │   ├── stubs/
│   │   │   └── esp_err.h
│   │   ├── eskf_pc.cpp
│   │   ├── main.cpp
│   │   └── replay.cpp
│   └── scripts/
│       ├── log_capture.py       # ログ受信・保存
│       ├── visualize_eskf.py    # 可視化
│       └── requirements.txt     # Python依存関係
└── docs/
    └── eskf_debug_plan.md       # この計画書
```

---

## 5. 使用手順

### Step 1: ログ記録

```bash
# 実機に接続してログ開始
python tools/scripts/log_capture.py \
    --port /dev/tty.usbmodem14101 \
    --output logs/test_flight_001.bin \
    --duration 120

# CLI経由でログ開始 (実機側)
# > binlog on
# (120秒後)
# > binlog off
```

### Step 2: ESKFリプレイ

```bash
# PC用ESKFビルド
cd tools/eskf_debug
mkdir build && cd build
cmake ..
make

# ログファイルでESKFを実行
./eskf_replay ../../logs/test_flight_001.bin output_states.csv
```

### Step 3: 可視化・解析

```bash
# Python環境セットアップ
pip install -r tools/scripts/requirements.txt

# 可視化
python tools/scripts/visualize_eskf.py \
    --log logs/test_flight_001.bin \
    --eskf logs/output_states.csv \
    --output plots/
```

---

## 6. デバッグポイント

ESKFで問題が起きやすい箇所と確認方法:

### 6.1 姿勢推定

| 問題 | 症状 | 確認方法 |
|------|------|---------|
| ジャイロバイアス未補正 | ドリフト | 静止状態でyaw角が変化 |
| 座標系不一致 | 姿勢が逆転 | roll/pitchの符号確認 |
| 重力方向誤り | pitch/rollオフセット | 静止時にroll≈0, pitch≈0か |

### 6.2 高度推定

| 問題 | 症状 | 確認方法 |
|------|------|---------|
| Baro/ToF不整合 | 高度ジャンプ | 両センサの生値比較 |
| ToF姿勢補正誤り | 傾けると高度変化 | cos(roll)*cos(pitch)の適用確認 |

### 6.3 速度推定

| 問題 | 症状 | 確認方法 |
|------|------|---------|
| 加速度バイアス | 速度発散 | 静止時にvelocity≈0か |
| Flow座標系誤り | 移動方向逆 | 手で動かして符号確認 |

### 6.4 数値安定性

| 問題 | 症状 | 確認方法 |
|------|------|---------|
| 共分散発散 | NaN発生 | P行列の対角成分監視 |
| 逆行列計算失敗 | 状態ジャンプ | S行列の条件数確認 |

---

## 7. 期待される成果

1. **問題の特定**: どの観測更新で問題が起きているか特定
2. **パラメータチューニング**: ノイズパラメータの適正化
3. **コード修正**: PC上で修正→実機に反映のサイクル高速化
4. **回帰テスト**: 保存したログで修正後のコード検証

---

## 8. Q/R パラメータ自動推定機能

### 8.1 概要

ESKFの性能はプロセスノイズ **Q** と観測ノイズ **R** に大きく依存する。
ログデータから統計的にこれらのパラメータを推定する機能を追加する。

**ファイル**: `tools/scripts/estimate_qr.py`

### 8.2 推定手法

#### (A) 観測ノイズ R の推定（静止データから）

静止状態のログを使用し、各センサの分散を計算：

```python
# 静止データ (機体を動かさない状態で記録)
R_accel = np.var(accel_data, axis=0)      # [m/s²]²
R_gyro = np.var(gyro_data, axis=0)        # [rad/s]²
R_mag = np.var(mag_data, axis=0)          # [uT]²
R_baro = np.var(baro_alt_data)            # [m]²
R_tof = np.var(tof_data)                  # [m]²
```

| センサ | 推定方法 | 期待値（参考） |
|--------|---------|---------------|
| 加速度 | 静止時分散 | 0.01〜0.1 [m/s²]² |
| ジャイロ | 静止時分散 | 0.0001〜0.001 [rad/s]² |
| 地磁気 | 静止時分散 | 1〜25 [uT]² |
| 気圧高度 | 静止時分散 | 0.1〜1 [m]² |
| ToF | 静止時分散 | 0.001〜0.01 [m]² |
| OptFlow | 静止時分散 (squal>30) | 0.1〜1 |

#### (B) プロセスノイズ Q の推定

**方法1: Allan分散解析（ジャイロ・加速度）**

長時間の静止データからAllan分散を計算し：
- **角度ランダムウォーク (ARW)** → `gyro_noise`
- **バイアス不安定性** → `gyro_bias_noise`
- **速度ランダムウォーク (VRW)** → `accel_noise`

```python
def allan_variance(data, dt, max_cluster=1000):
    """Allan分散を計算"""
    n = len(data)
    taus = []
    avars = []
    for m in range(1, min(n//2, max_cluster)):
        tau = m * dt
        clusters = data[:n//m*m].reshape(-1, m).mean(axis=1)
        avar = 0.5 * np.mean(np.diff(clusters)**2)
        taus.append(tau)
        avars.append(avar)
    return np.array(taus), np.array(avars)
```

**方法2: イノベーションベース調整**

ESKFを実行しながらイノベーション（予測誤差）を監視：
- イノベーションの分散 ≈ HPH' + R であるべき
- 実際の分散が大きすぎる → R を増加
- 実際の分散が小さすぎる → R を減少

```python
def tune_R_from_innovation(innovations, H, P, R_init):
    """イノベーションからRを調整"""
    expected_var = H @ P @ H.T + R_init
    actual_var = np.var(innovations)

    # スケーリング係数
    scale = actual_var / expected_var
    R_tuned = R_init * scale
    return R_tuned
```

### 8.3 使用手順

#### Step 1: 静止データ収集

```bash
# 機体を平らな場所に置いて60秒間記録
python tools/scripts/log_capture.py \
    --port /dev/tty.usbmodem* \
    --output logs/static_calibration.bin \
    --duration 60
```

#### Step 2: Q/R パラメータ推定

```bash
python tools/scripts/estimate_qr.py \
    --input logs/static_calibration.bin \
    --output config/eskf_params.json \
    --plot plots/noise_analysis.png
```

#### Step 3: 出力ファイル

```json
// config/eskf_params.json
{
  "process_noise": {
    "gyro_noise": 0.012,
    "accel_noise": 0.08,
    "gyro_bias_noise": 0.00015,
    "accel_bias_noise": 0.0012
  },
  "measurement_noise": {
    "accel_noise": [0.015, 0.018, 0.022],
    "gyro_noise": [0.00032, 0.00028, 0.00035],
    "mag_noise": [8.2, 7.5, 12.1],
    "baro_noise": 0.45,
    "tof_noise": 0.008,
    "flow_noise": 0.35
  },
  "analysis": {
    "static_duration_sec": 60,
    "samples": 6000,
    "timestamp": "2025-11-30T12:00:00"
  }
}
```

#### Step 4: ESKFリプレイで検証

```bash
./eskf_replay \
    --log logs/test_flight.bin \
    --params config/eskf_params.json \
    --output results/tuned_states.csv
```

### 8.4 出力グラフ

```
┌─────────────────────────────────────────────┐
│ Allan Deviation (Gyro X)                    │
│                                             │
│  σ(τ)                                       │
│   │    ╲                                    │
│   │     ╲___                                │
│   │         ╲____                           │
│   │              ╲____                      │
│   └──────────────────────────── τ [s]      │
│        ↑                                    │
│      ARW = 0.012 rad/s/√Hz                 │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│ Sensor Noise Histogram                      │
│                                             │
│  Accel X: σ = 0.082 m/s²                   │
│  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░                         │
│                                             │
│  Gyro X: σ = 0.018 rad/s                   │
│  ▓▓▓▓▓▓▓▓▓▓░░░░░░░                         │
│                                             │
│  Baro Alt: σ = 0.67 m                      │
│  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░                        │
└─────────────────────────────────────────────┘
```

### 8.5 実装ファイル追加

```
tools/scripts/
├── estimate_qr.py       # Q/Rパラメータ推定 (新規)
├── allan_variance.py    # Allan分散計算 (新規)
└── ...
```

---

## 9. 今後の拡張

- **リアルタイムデバッグ**: PCでシリアル受信しながらリアルタイムでESKF実行・可視化
- **Hardware-in-the-loop**: PC上のESKF結果を実機にフィードバック
- **自動テスト**: CI/CDでログファイルを使った回帰テスト
- **適応的Q/R調整**: 飛行中にイノベーションを監視してQ/Rをオンライン調整

