# ESKFシミュレーションと実機の一致

PCリプレイ（シミュレーション）と実機デバイスのESKF出力を一致させるための知見をまとめる。

## 背景

バイナリログを使用してPCでESKFをリプレイし、パラメータチューニングを行う際、シミュレーションと実機の挙動が一致していないと、調整したパラメータが実機で期待通りに動作しない。

## 重要な原則

**Kalmanフィルタでは、センサー更新のタイミングがP行列（共分散行列）の進化に影響し、結果としてKalmanゲインが変わる。**

- 更新頻度が高い → P行列が小さく保たれる → 各更新のKalmanゲインが小さい
- 更新頻度が低い → P行列が大きくなる → 各更新のKalmanゲインが大きい

同じセンサーデータでも、更新タイミングが異なれば推定結果が異なる。

## 実機のセンサー更新タイミング

StampFlyデバイスの実装（main.cpp IMUTask内）:

| センサー | 更新レート | トリガー条件 |
|---------|-----------|-------------|
| IMU Predict | 400Hz | 毎サイクル |
| AccelAttitude | 400Hz | 毎サイクル |
| Flow | 100Hz | `flow_update_counter >= 4`（固定間隔） |
| ToF | 30Hz | `g_tof_data_ready`フラグ（固定間隔） |
| Mag | 10Hz | `g_mag_data_ready`フラグ（固定間隔） |
| Baro | 50Hz | `g_baro_data_ready`フラグ（固定間隔） |

**重要**: デバイスは「データが変化したか」ではなく、「data_readyフラグが立ったか」で更新を行う。

## PCリプレイでの実装

### 間違った実装（データ変化検出）

```cpp
// NG: データが変化した時のみ更新
if (pkt.flow_dx != last_flow_dx || pkt.flow_dy != last_flow_dy) {
    eskf.updateFlow(...);
    last_flow_dx = pkt.flow_dx;
    last_flow_dy = pkt.flow_dy;
}
```

この方法だと:
- 静止時はフローデータが変化しないため更新されない
- 実際の更新レートがデバイスより低くなる（例: 27Hz vs 100Hz）
- P行列が大きくなり、Kalmanゲインが高くなる

### 正しい実装（固定間隔）

```cpp
// OK: 固定間隔で更新（デバイスと同じ）
constexpr size_t FLOW_UPDATE_INTERVAL = 4;  // 400Hz / 100Hz = 4
size_t flow_update_counter = 0;

// メインループ内
flow_update_counter++;
if (flow_update_counter >= FLOW_UPDATE_INTERVAL) {
    flow_update_counter = 0;
    if (pkt.flow_squal >= FLOW_SQUAL_MIN) {
        eskf.updateFlow(...);
    }
}
```

## 各センサーの更新間隔

PCリプレイで使用すべき更新間隔（400Hz IMUループ基準）:

```cpp
// Flow: 100Hz = 400Hz / 4
constexpr size_t FLOW_UPDATE_INTERVAL = 4;

// ToF: 30Hz ≈ 400Hz / 13
constexpr size_t TOF_UPDATE_INTERVAL = 13;

// Mag: 10Hz = 400Hz / 40
constexpr size_t MAG_UPDATE_INTERVAL = 40;

// Baro: 50Hz = 400Hz / 8
constexpr size_t BARO_UPDATE_INTERVAL = 8;
```

## 修正結果

修正前後の比較（updown4.binログ使用）:

| 指標 | 修正前 | 修正後 |
|------|--------|--------|
| ポジションエラー | 0.126m | 0.004m |
| Vx ノイズ比率 (PC/Device) | 1.144 | 0.992 |
| Vy ノイズ比率 (PC/Device) | 1.069 | 1.015 |
| 高周波ノイズ比率 | 1.154 | 1.006 |

97%の改善を達成し、PCとデバイスがほぼ完全に一致するようになった。

## 関連ファイル

- `main/main.cpp` - デバイス側ESKF更新ロジック（IMUTask内）
- `tools/eskf_debug/replay.cpp` - PCリプレイ実装
- `tools/scripts/optimize_eskf.py` - パラメータ最適化ツール

## 参考: 問題の発見方法

1. PC vs Deviceの比較グラフで速度ノイズの差を確認
2. フロー更新時の速度変化量を比較（PC: 24%大きい）
3. センサー更新レートを分析（PC: 27Hz vs Device: 100Hz）
4. デバイスコードを確認し、固定間隔更新であることを発見
