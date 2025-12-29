# ESKF 開発ワークフロー

本プロジェクトでは、実機のセンサログをPCでリプレイしてESKFのデバッグ・パラメータ調整を行うワークフローを標準としています。

## 概要

```
┌─────────────┐                    ┌─────────────┐
│  StampFly   │  USB Serial        │     PC      │
│   (実機)    │ ────────────────>  │             │
│             │  バイナリログ       │  1. キャプチャ (log_capture.py)
│ センサ+ESKF │  128B @ 100Hz      │  2. リプレイ  (eskf_replay)
└─────────────┘                    │  3. 可視化    (visualize_eskf.py)
                                   │  4. 最適化    (optimize_eskf.py)
                                   └─────────────┘
```

**メリット:**
- 実機を何度も飛ばさずに同じデータで繰り返し検証
- Q/Rパラメータの自動最適化
- PC vs Device の結果比較でバグ発見

---

## クイックスタート

### 1. ログ取得

```bash
cd tools/scripts

# 実機を接続して60秒間ログ取得
python log_capture.py capture -p /dev/tty.usbmodem* -o ../../logs/flight.bin -d 60
```

実機側では自動的に`binlog on`が送信されます。

### 2. PC でリプレイ

```bash
cd ../eskf_debug/build

# 初回のみビルド
cmake .. && make -j4

# ログをリプレイしてCSV出力
./eskf_replay ../../../logs/flight.bin ../../../logs/flight_result.csv
```

### 3. 可視化

```bash
cd ../../scripts

# 全パネル表示
python visualize_eskf.py ../../logs/flight_result.csv --all

# PC vs Device 比較
python visualize_eskf.py ../../logs/flight_result.csv --compare
```

### 4. パラメータ最適化（オプション）

```bash
# シミュレーテッドアニーリングで最適化
python optimize_eskf.py ../../logs/flight.bin --method sa --iter 500

# 最適化結果を eskf.hpp に自動適用
python optimize_eskf.py ../../logs/flight.bin --apply
```

---

## 詳細ワークフロー

### Step 1: データ収集

#### 静止データ（キャリブレーション用）

```bash
# 機体を平らな場所に置いて60秒間
python log_capture.py capture -p /dev/tty.usbmodem* -o ../../logs/static.bin -d 60
```

用途:
- センサノイズ特性の確認
- Q/R初期値の推定（`estimate_qr.py`）

#### 動的データ（最適化・検証用）

```bash
# 手で持って揺動（小振幅・大振幅）
python log_capture.py capture -p /dev/tty.usbmodem* -o ../../logs/oscillation.bin -d 30

# 実際のフライト
python log_capture.py capture -p /dev/tty.usbmodem* -o ../../logs/hover.bin -d 60
```

用途:
- パラメータ最適化（複数データで汎用性向上）
- ESKF動作検証

### Step 2: PCリプレイ

```bash
cd tools/eskf_debug/build
./eskf_replay ../../../logs/flight.bin ../../../logs/result.csv
```

出力CSVには以下が含まれます:
- センサ生値（Device記録）
- ESKF推定値（Device）
- ESKF推定値（PC再計算）

**PC vs Device の比較**で不一致があれば:
- 浮動小数点精度の違い（正常）
- タイミングの違いによるセンサフュージョン差（正常）
- 重大なバグ（要調査）

### Step 3: 可視化・解析

```bash
cd tools/scripts

# 基本的な可視化
python visualize_eskf.py ../../logs/result.csv --all

# 姿勢のみ
python visualize_eskf.py ../../logs/result.csv --attitude

# 3Dアニメーション
python visualize_attitude_3d.py ../../logs/result.csv
python visualize_pose_3d.py ../../logs/result.csv --mp4
```

#### チェックポイント

| 項目 | 正常な状態 | 問題のサイン |
|------|-----------|-------------|
| Roll/Pitch | 静止時 ≈ 0° | 大きなオフセット |
| Yaw | ゆっくりドリフト | 急速なドリフト |
| 高度 | Baro/ToFと一致 | 大きな乖離 |
| 速度 | 静止時 ≈ 0 | 発散 |
| ジャイロバイアス | 徐々に収束 | 発散 or 振動 |

### Step 4: パラメータ最適化

```bash
# 複数データセットで最適化（推奨）
python optimize_eskf.py ../../logs/small_osc.bin ../../logs/large_osc.bin --apply

# 単一データセット
python optimize_eskf.py ../../logs/flight.bin --method sa --iter 500
```

最適化対象パラメータ:

| パラメータ | 意味 | 調整の効果 |
|-----------|------|-----------|
| `gyro_noise` | ジャイロノイズ | 大きい→姿勢追従遅い、小さい→ノイズ乗る |
| `accel_noise` | 加速度計ノイズ | 同上 |
| `gyro_bias_noise` | バイアスドリフト | 大きい→バイアス追従早い |
| `flow_noise` | オプティカルフロー | 大きい→フロー信頼度低 |
| `tof_noise` | ToFノイズ | 大きい→ToF信頼度低 |
| `accel_att_noise` | 加速度姿勢補正 | 大きい→加速度姿勢信頼度低 |

---

## ツールリファレンス

### log_capture.py

デバイスからバイナリログをキャプチャ。

```bash
python log_capture.py capture -p PORT -o OUTPUT -d DURATION [--live] [--debug]
python log_capture.py convert -i INPUT -o OUTPUT.csv
python log_capture.py info FILE.bin
```

| オプション | 説明 |
|-----------|------|
| `-p`, `--port` | シリアルポート（必須） |
| `-o`, `--output` | 出力ファイル（必須） |
| `-d`, `--duration` | キャプチャ時間[秒]（デフォルト60） |
| `--live` | リアルタイム表示 |
| `--no-auto` | binlog on/off を送信しない |

### eskf_replay

PCでESKFをリプレイ。

```bash
./eskf_replay INPUT.bin OUTPUT.csv
```

ビルド:
```bash
cd tools/eskf_debug
mkdir -p build && cd build
cmake .. && make -j4
```

### visualize_eskf.py

センサ・ESKF状態の可視化。

```bash
python visualize_eskf.py FILE [--all] [--sensors] [--attitude] [--position] [--compare]
```

| オプション | 表示内容 |
|-----------|---------|
| `--all` | 全パネル（デフォルト） |
| `--sensors` | センサ生値のみ |
| `--attitude` | Roll/Pitch/Yaw |
| `--position` | 位置・速度 |
| `--compare` | PC vs Device 比較 |
| `--pc` | PC結果のみ（Device非表示） |
| `--save FILE` | 画像保存 |

### optimize_eskf.py

Q/Rパラメータ自動最適化。

```bash
python optimize_eskf.py FILE1 [FILE2 ...] [--method {sa,gd}] [--iter N] [--apply]
```

| オプション | 説明 |
|-----------|------|
| `--method sa` | シミュレーテッドアニーリング（推奨） |
| `--method gd` | 最急降下法（高速だが局所解） |
| `--iter N` | イテレーション数 |
| `--apply` | eskf.hpp に自動反映 |
| `--roll-weight W` | Roll誤差の重み（0-1） |

### その他のツール

| ツール | 用途 |
|--------|------|
| `visualize_attitude_3d.py` | 姿勢3Dアニメーション |
| `visualize_pose_3d.py` | 位置+姿勢3Dアニメーション（`--mp4`で動画保存） |
| `estimate_qr.py` | 静止データからQ/R推定 |
| `plot_mag_xy.py` | 地磁気キャリブレーション確認 |

詳細は [`tools/scripts/README.md`](../tools/scripts/README.md) を参照。

---

## バイナリログ形式

### パケット構造（V2, 128 bytes）

| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0-1 | 2 | ヘッダー: `0xAA 0x56` |
| 2-5 | 4 | タイムスタンプ [ms] |
| 6-29 | 24 | IMU: accel_xyz, gyro_xyz [float] |
| 30-41 | 12 | Mag: mag_xyz [float] |
| 42-49 | 8 | Baro: pressure, altitude [float] |
| 50-57 | 8 | ToF: bottom, front [float] |
| 58-62 | 5 | Flow: dx, dy [int16], squal [uint8] |
| 63-110 | 48 | ESKF推定値: pos, vel, att, bias |
| 111 | 1 | ESKFステータス |
| 112-126 | 15 | メタデータ・予約 |
| 127 | 1 | チェックサム (XOR) |

詳細な構造体定義は `components/stampfly_cli/include/cli.hpp` の `BinaryLogPacketV2` を参照。

---

## トラブルシューティング

### ログキャプチャ

| 問題 | 対処 |
|------|------|
| ポートが見つからない | `ls /dev/tty.usbmodem*` で確認 |
| パケットが取れない | 実機CLIで `binlog on` を手動実行 |
| チェックサムエラー多発 | ボーレート確認（115200） |

### ESKF推定

| 症状 | 可能性のある原因 |
|------|-----------------|
| Roll/Pitchオフセット | 加速度バイアス未補正、機体傾斜 |
| Yaw急速ドリフト | ジャイロバイアス推定が追従できていない |
| 高度ジャンプ | ToF/Baro切り替え閾値、姿勢補正誤り |
| 速度発散 | フローノイズ過小、加速度バイアス |
| NaN発生 | 共分散発散、逆行列計算失敗 |

### 最適化

| 問題 | 対処 |
|------|------|
| 収束しない | イテレーション増加 `--iter 1000` |
| 局所解に陥る | SA使用（`--method sa`）、複数データセット |
| 過学習 | 複数の異なる動きのデータで最適化 |

---

## ファイル構成

```
stampfly_rtos_skelton/
├── logs/                          # ログファイル保存先（gitignore）
│   ├── *.bin                      # バイナリログ
│   └── *.csv                      # 変換・リプレイ結果
├── tools/
│   ├── scripts/
│   │   ├── log_capture.py         # ログキャプチャ
│   │   ├── visualize_eskf.py      # 可視化
│   │   ├── optimize_eskf.py       # パラメータ最適化
│   │   └── README.md              # ツール詳細ドキュメント
│   └── eskf_debug/
│       ├── build/
│       │   └── eskf_replay        # PCリプレイ実行ファイル
│       └── *.cpp                  # ESKFのPC版ソース
└── components/
    └── stampfly_eskf/
        └── include/eskf.hpp       # ESKFパラメータ定義
```

---

*最終更新: 2025-12-29*
