# StampFly ログキャプチャ・可視化ツール

StampFlyデバイスからバイナリログをキャプチャし、ESKFの動作検証を行うためのツール群です。

## ファイル構成

```
tools/scripts/
├── log_capture.py          # ログキャプチャ・変換ツール
├── visualize_device_log.py # 統合可視化ツール
├── plot_mag_xy.py          # 磁気キャリブレーション確認用
└── estimate_qr.py          # QRパラメータ推定
```

## 必要なライブラリ

```bash
pip install numpy pandas matplotlib pyserial
```

---

## 1. log_capture.py - ログキャプチャツール

StampFlyデバイスからUSBシリアル経由でバイナリログをキャプチャします。

### パケットフォーマット

| サイズ | ヘッダ | 内容 |
|--------|--------|------|
| 128 bytes | 0xAA 0x56 | センサデータ + ESKF推定結果 |

**含まれるデータ:** IMU, Mag, Baro, ToF, Flow, 位置, 速度, 姿勢, バイアス

### 使い方

#### ログキャプチャ

```bash
# 基本的な使い方
python log_capture.py capture --port /dev/tty.usbmodem* --output sensor.bin --duration 60

# ライブ表示付き
python log_capture.py capture --port /dev/tty.usbmodem* --output sensor.bin --duration 30 --live

# デバッグモード
python log_capture.py capture --port /dev/tty.usbmodem* --output sensor.bin --duration 30 --debug
```

**オプション:**
| オプション | 説明 |
|-----------|------|
| `--port`, `-p` | シリアルポート（必須） |
| `--output`, `-o` | 出力ファイル（必須） |
| `--duration`, `-d` | キャプチャ時間（秒、デフォルト: 60） |
| `--baudrate`, `-b` | ボーレート（デフォルト: 115200） |
| `--live`, `-l` | パケットをリアルタイム表示 |
| `--no-auto` | binlog on/offコマンドを自動送信しない |
| `--debug` | デバッグ出力を有効化 |

#### CSV変換

```bash
python log_capture.py convert --input sensor.bin --output sensor.csv
```

#### ログ情報表示

```bash
python log_capture.py info sensor.bin
```

出力例:
```
File: sensor.bin
Packet version: V2
Packet size: 128 bytes
Packets: 3000
Duration: 30.00 seconds
Start timestamp: 12345 ms
End timestamp: 42345 ms

First packet:
  [V2] t=   12345ms pos=[ 0.000, 0.000,-0.300] vel=[ 0.00, 0.00, 0.00] att=[  0.0,  0.0,   0.0]deg

Last packet:
  [V2] t=   42345ms pos=[ 0.015,-0.008,-0.298] vel=[ 0.01,-0.00, 0.00] att=[  0.5, -0.3,   2.1]deg

ESKF Statistics:
  Roll:  min=-1.2°, max=1.5°
  Pitch: min=-0.8°, max=1.0°
  Yaw:   min=-0.5°, max=5.2°
  Pos Z: min=-0.310m, max=-0.290m
```

---

## 2. visualize_device_log.py - 統合可視化ツール

デバイスログとPC版ESKFの結果を可視化・比較します。

### 可視化モード

| モード | 説明 | 必要なファイル |
|--------|------|---------------|
| `device` | デバイスログのみ表示（デフォルト） | .bin |
| `pc` | PC版ESKF結果のみ表示 | .csv |
| `both` | デバイスとPCの比較表示 | .bin + .csv |

### 使い方

#### デバイスログの可視化

```bash
# ログを可視化
python visualize_device_log.py sensor.bin

# 画像ファイルに保存（表示なし）
python visualize_device_log.py sensor.bin --output result.png --no-show
```

#### PC版ESKF結果の可視化

```bash
python visualize_device_log.py --pc eskf_output.csv --mode pc
```

#### デバイスとPCの比較

```bash
python visualize_device_log.py sensor.bin --pc eskf_output.csv --mode both
```

#### 磁気キャリブレーション確認

```bash
python visualize_device_log.py sensor.bin --mag-xy
```

XY平面上の磁気データをプロットし、キャリブレーションが正しいか確認できます。校正が正しければ、点が原点を中心とした円状に分布します。

### オプション一覧

| オプション | 説明 |
|-----------|------|
| `input` | デバイスバイナリログ (.bin) |
| `--pc`, `-p` | PC版ESKF出力CSV |
| `--mode`, `-m` | 可視化モード: device, pc, both |
| `--mag-xy` | 磁気XYプロットを表示 |
| `--output`, `-o` | 出力画像ファイル (.png) |
| `--no-show` | プロットウィンドウを表示しない |

### 出力グラフ

#### デバイスログ
- 加速度計・ジャイロスコープ
- 高度（気圧/ToF/ESKF推定）
- 姿勢（Roll/Pitch/Yaw）
- 位置・速度
- 2Dトラジェクトリ
- オプティカルフロー
- バイアス推定値
- 統計情報

#### 比較モード (both)
- 姿勢比較（デバイス vs PC）
- Yaw差分グラフ
- 位置・速度比較
- 2Dトラジェクトリ比較
- バイアス比較
- ドリフト統計

---

## 3. 典型的なワークフロー

### ESKF検証の手順

1. **デバイスでログキャプチャ**
   ```bash
   python log_capture.py capture -p /dev/tty.usbmodem* -o flight.bin -d 60
   ```

2. **PC版ESKFでリプレイ**
   ```bash
   cd ../eskf_debug/build
   ./eskf_replay ../../scripts/flight.bin flight_pc.csv
   ```

3. **デバイスとPCを比較**
   ```bash
   python visualize_device_log.py flight.bin --pc flight_pc.csv --mode both -o comparison.png
   ```

### 磁気キャリブレーション確認

1. **キャリブレーション実行**（CLI）
   ```
   > magcal start
   > (デバイスを回転)
   > magcal stop
   > magcal save
   ```

2. **平面上で回転してログ取得**
   ```bash
   python log_capture.py capture -p /dev/tty.usbmodem* -o mag_test.bin -d 30
   ```

3. **XYプロットで確認**
   ```bash
   python visualize_device_log.py mag_test.bin --mag-xy -o mag_verify.png
   ```

   校正が正しければ、点が原点 (0, 0) を中心とした円状に分布します。

---

## 4. トラブルシューティング

### シリアルポートが見つからない

```bash
# macOS
ls /dev/tty.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

### チェックサムエラーが多い

- ボーレートが正しいか確認（デフォルト: 115200）
- USBケーブルを短いものに交換
- `--debug` オプションで詳細を確認

### グラフが表示されない

- `--no-show` オプションを外す
- matplotlibのバックエンドを確認
- `--output` で画像ファイルに保存して確認
