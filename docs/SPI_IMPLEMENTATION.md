# SPI バス設計と実装

## SPI2_HOST 共有バス構成

BMI270（IMU）とPMW3901（OpticalFlow）が同一SPIバスを共有:

| デバイス | CS | 用途 | 転送頻度 |
|----------|-----|------|----------|
| BMI270 | GPIO39 | 加速度・角速度 | 400Hz |
| PMW3901 | GPIO12 | オプティカルフロー | 100Hz |

## 解決済みの問題

**問題**: `spi_device_polling_transmit`（BMI270）と`spi_device_transmit`（PMW3901）の混在により、ESP-IDF SPI ドライバ内部でバスロック競合が発生し、タスクが停止。

**解決**: 全デバイスを`spi_device_polling_transmit`に統一（2024-12実装）

## 現在の転送方式

| 関数 | サイズ | バッファ | 用途 |
|------|--------|----------|------|
| `bmi270_read_burst` ≤32B | スタック | 通常IMU読み取り(14B) |
| `bmi270_read_burst` >32B | DMA(malloc) | FIFO読み取り(最大2KB) |
| `bmi270_write_burst` ≤32B | スタック | レジスタ書き込み |
| `bmi270_write_burst` >32B | DMA(malloc) | config file(8KB) |
| `pmw3901_*` | スタック | 全て32B以下 |

**32バイト閾値の理由**: ESP-IDF SPIドライバは32バイト以下でCPU転送、超過でDMA転送を使用

## 今後の実装計画

### Phase 1: SPI統一（完了）
- [x] PMW3901を`spi_device_polling_transmit`に変更
- [x] BMI270バッファ最適化（小転送はスタック）
- [ ] 長時間動作テスト（10分以上）

### Phase 2: オーバーサンプリング対応
BMI270を高ODRで動作させ、FIFOで複数サンプルを一括取得:

```
BMI270: 1600Hz ODR
    ↓ FIFO (4 frames蓄積)
    ↓ Watermark割り込み (400Hz)
IMUTask: 4サンプル読み取り → フィルタ → ESKF
```

- [ ] BMI270 FIFO設定変更（ODR=1600Hz, watermark=4 frames）
- [ ] 割り込みハンドラ調整（Data Ready → FIFO Watermark）
- [ ] フィルタ処理統合

### Phase 3: フィルタコンポーネント
`stampfly_filter`コンポーネント新規作成:

- [ ] LPF（Butterworth等）実装
- [ ] ノッチフィルタ（モーター振動除去）
- [ ] 将来: 適応フィルタ（RPM連動ダイナミックノッチ）

### Phase 4: 非接触回転計測（将来）
加速度FFT解析によるモーターRPM推定:

- BPF (Blade Passing Frequency) = RPM × ブレード枚数 / 60
- esp-dspライブラリでFFT実行
- ESCテレメトリ不要の回転数取得

## BMI270 ODR仕様

| センサー | 最大ODR | レジスタ値 |
|----------|---------|------------|
| 加速度計 | 1600Hz | 0x0C |
| ジャイロスコープ | 3200Hz | 0x0D |

ジャイロは加速度の2倍の最大ODRをサポート。
FIFO使用時は両者同一ODRが推奨（フレーム解析が単純）。

## 関連ファイル

- `components/stampfly_imu/src/bmi270_spi.c` - BMI270 SPI転送実装
- `components/stampfly_opticalflow/src/pmw3901.c` - PMW3901 SPI転送実装
- `components/stampfly_imu/CLAUDE.md` - BMI270ドライバ詳細
- `components/stampfly_opticalflow/CLAUDE.md` - PMW3901ドライバ詳細
