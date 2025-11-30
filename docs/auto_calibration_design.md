# ESKF自動キャリブレーションフレームワーク設計

## 概要

センサデータからESKFパラメータ（Q/R、オフセット、スケール）を自動的に最適化するフレームワーク。

## 現状の課題

1. **手動チューニングの限界**
   - flow_scale、オフセット、Q/Rを個別に手動調整
   - パラメータ間の相互依存を考慮できない
   - 再現性がない

2. **検証で判明した問題**
   - Q値を推定値に近づけるとバイアス推定が不安定化
   - オフセットは特定のQ/R設定下で最適化されるため、Q/Rを変えると再調整が必要

## 設計方針

### 階層的キャリブレーション

パラメータ間の依存関係を考慮し、以下の順序でキャリブレーション:

```
Phase 1: センサノイズ特性 (静止データ)
    ↓
Phase 2: スケール・軸変換 (既知動作データ)
    ↓
Phase 3: バイアス・オフセット (閉ループ最適化)
    ↓
Phase 4: Q/R微調整 (グリッドサーチ)
```

### キャリブレーションデータ要件

| Phase | データ種別 | 収集方法 | 目的 |
|-------|----------|---------|------|
| 1 | 静止データ (30秒以上) | 机上に静置 | センサノイズ推定 |
| 2 | 単軸移動データ | 1軸方向に既知距離移動 | フロースケール決定 |
| 3 | 閉ループデータ | 開始点に戻る動作 | オフセット最適化 |
| 4 | 多様な動作データ | 複数の飛行パターン | 汎化性能検証 |

---

## Phase 1: センサノイズ特性推定

### 入力
- 静止状態のセンサログ (≥30秒)

### 処理
1. Allan分散分析でIMUノイズ特性を抽出
2. 各センサの統計量（平均、分散）を計算
3. バイアス初期値を推定

### 出力
```json
{
  "imu": {
    "gyro_arw": [x, y, z],      // Angle Random Walk
    "gyro_bias_inst": [x, y, z], // Bias Instability
    "accel_vrw": [x, y, z],      // Velocity Random Walk
    "gyro_bias_init": [x, y, z], // Initial bias estimate
    "accel_bias_init": [x, y, z]
  },
  "sensors": {
    "baro_std": 0.099,
    "tof_std": 0.0013,
    "flow_std": [dx, dy],
    "mag_std": [x, y, z]
  }
}
```

### 既存ツール
- `estimate_qr.py` を拡張

---

## Phase 2: スケール・軸変換キャリブレーション

### 入力
- 単軸移動データ（例: X方向に20cm移動して戻る）

### 処理
1. 移動区間を検出（速度閾値）
2. フロー積分値と期待移動量を比較
3. 最小二乗法でスケール係数を決定
4. 軸変換行列を推定

### アルゴリズム
```python
def estimate_flow_scale(flow_data, gyro_data, expected_displacement):
    # ジャイロ補償
    flow_compensated = flow_data - gyro_compensation(gyro_data)

    # フロー積分
    integrated = cumsum(flow_compensated * height * dt)

    # スケール推定
    scale = expected_displacement / integrated[-1]
    return scale
```

### 出力
```json
{
  "flow_scale": 0.08,
  "axis_transform": {
    "vx_body": "-flow_y * h",
    "vy_body": "-flow_x * h"
  },
  "gyro_compensation": {
    "k_xx": 1.35, "k_xy": 9.30,
    "k_yx": -2.65, "k_yy": 0.0
  }
}
```

---

## Phase 3: オフセット最適化

### 入力
- 閉ループデータ（開始点に戻る動作）

### 処理
1. ESKFを実行し最終位置を取得
2. 閉ループエラーを目的関数として最適化
3. グリッドサーチまたは勾配法でオフセットを探索

### 目的関数
```python
def objective(offset_dx, offset_dy):
    # ESKFを実行
    final_pos = run_eskf(data, offset=(offset_dx, offset_dy))

    # 閉ループエラー
    error = sqrt(final_pos.x**2 + final_pos.y**2)
    return error
```

### 最適化手法
1. **グリッドサーチ**: 粗い探索（-1.0〜1.0, step=0.1）
2. **Nelder-Mead**: 細かい最適化（初期値=グリッド最良点）

### 出力
```json
{
  "flow_offset": {
    "dx": 0.29,
    "dy": 0.29
  },
  "closed_loop_error": 0.051,
  "optimization": {
    "method": "nelder-mead",
    "iterations": 42,
    "convergence": true
  }
}
```

---

## Phase 4: Q/R微調整

### 入力
- Phase 3で最適化されたオフセット
- 複数の動作データセット

### 処理
1. Q/Rパラメータの探索範囲を定義
2. 各パラメータセットでESKFを実行
3. 複合評価指標で最適パラメータを選択

### 評価指標
```python
def evaluate(params, datasets):
    scores = []
    for data in datasets:
        result = run_eskf(data, params)

        # 複合スコア
        score = (
            0.4 * closed_loop_error(result) +
            0.3 * velocity_consistency(result) +
            0.2 * height_accuracy(result, data.tof) +
            0.1 * bias_stability(result)
        )
        scores.append(score)

    return mean(scores), std(scores)
```

### 探索空間（デフォルト値の倍率）
```python
param_grid = {
    'gyro_noise': [0.5, 1.0, 2.0],      # × default
    'accel_noise': [0.5, 1.0, 2.0],
    'baro_noise': [0.1, 0.5, 1.0],
    'tof_noise': [0.04, 0.1, 0.5],
    'flow_noise': [0.01, 0.05, 0.1]
}
```

### 制約
- バイアス推定の安定性を監視
- Yawバイアスが発散したらそのパラメータセットを棄却

---

## 実装計画

### ファイル構成

```
tools/
├── calibration/
│   ├── __init__.py
│   ├── calibration_manager.py    # メインエントリーポイント
│   ├── phase1_noise.py           # センサノイズ推定
│   ├── phase2_scale.py           # スケール・軸変換
│   ├── phase3_offset.py          # オフセット最適化
│   ├── phase4_qr_tuning.py       # Q/R微調整
│   ├── eskf_runner.py            # ESKFラッパー (C++呼び出し)
│   ├── data_loader.py            # データ読み込み
│   └── visualizer.py             # 結果可視化
├── scripts/
│   └── run_calibration.py        # CLIエントリーポイント
└── eskf_debug/
    └── (既存のC++ツール)
```

### CLI インターフェース

```bash
# 全フェーズ実行
python run_calibration.py \
    --static static_data.bin \
    --motion motion_data.bin \
    --closed-loop closed_loop_data.bin \
    --output calibration_result.json

# 個別フェーズ
python run_calibration.py --phase 1 --input static_data.bin
python run_calibration.py --phase 3 --input closed_loop_data.bin --params phase2_result.json

# 結果の適用
python run_calibration.py --apply calibration_result.json --target components/stampfly_eskf/
```

### 出力形式

```json
{
  "version": "1.0",
  "timestamp": "2025-11-30T15:30:00",
  "phases": {
    "phase1": { ... },
    "phase2": { ... },
    "phase3": { ... },
    "phase4": { ... }
  },
  "final_params": {
    "process_noise": {
      "gyro_noise": 0.001,
      "accel_noise": 0.1,
      "gyro_bias_noise": 0.00005,
      "accel_bias_noise": 0.001
    },
    "measurement_noise": {
      "baro_noise": 0.1,
      "tof_noise": 0.002,
      "flow_noise": 0.01
    },
    "flow_calibration": {
      "scale": 0.08,
      "offset_dx": 0.29,
      "offset_dy": 0.29,
      "gyro_comp": { "k_xx": 1.35, "k_xy": 9.30, "k_yx": -2.65, "k_yy": 0.0 }
    }
  },
  "validation": {
    "closed_loop_error_cm": 5.1,
    "datasets_tested": 3,
    "pass": true
  }
}
```

---

## 開発スケジュール

### Step 1: 基盤整備
- [ ] calibrationモジュール構造作成
- [ ] data_loader.py (既存log_capture.pyから抽出)
- [ ] eskf_runner.py (C++ツール呼び出しラッパー)

### Step 2: Phase 1-2 実装
- [ ] phase1_noise.py (estimate_qr.pyをリファクタ)
- [ ] phase2_scale.py (移動検出、スケール推定)
- [ ] 単体テスト

### Step 3: Phase 3 実装
- [ ] phase3_offset.py (グリッドサーチ)
- [ ] Nelder-Mead最適化追加
- [ ] 検証テスト

### Step 4: Phase 4 実装
- [ ] phase4_qr_tuning.py (グリッドサーチ)
- [ ] 複合評価指標実装
- [ ] 安定性チェック

### Step 5: 統合・CLI
- [ ] calibration_manager.py
- [ ] run_calibration.py
- [ ] ドキュメント

---

## 将来の拡張

1. **オンラインキャリブレーション**
   - 飛行中のバイアス・オフセット適応
   - 温度補償

2. **機体個体差対応**
   - 機体IDごとのパラメータ保存
   - NVSへの書き込み

3. **自動データ収集**
   - キャリブレーション用動作の自動実行
   - 品質チェック（データ十分性）

4. **Webインターフェース**
   - ブラウザからのキャリブレーション実行
   - リアルタイム可視化
