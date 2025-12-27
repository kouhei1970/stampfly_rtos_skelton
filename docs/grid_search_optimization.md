# グリッドサーチ最適化入門

## 概要

グリッドサーチ（Grid Search）は、パラメータ最適化のための**網羅的探索アルゴリズム**です。指定した範囲内のすべてのパラメータ組み合わせを評価し、最良の結果を与える組み合わせを見つけます。

## 基本原理

### 1. パラメータ空間の離散化

連続的なパラメータ空間を格子（グリッド）状に離散化します。

```
例: 2パラメータの場合

param_a: [0.1, 0.2, 0.3]     # 3つの候補値
param_b: [1.0, 2.0, 3.0, 4.0] # 4つの候補値

→ 3 × 4 = 12 通りの組み合わせを評価
```

視覚的なイメージ:

```
param_b
  4.0 ─┼───●───●───●
  3.0 ─┼───●───●───●
  2.0 ─┼───●───●───●
  1.0 ─┼───●───●───●
       └───┼───┼───┼──→ param_a
          0.1 0.2 0.3

       ● = 評価点
```

### 2. 評価関数（コスト関数）

各パラメータ組み合わせに対して「良さ」を数値化する関数を定義します。

```python
def cost_function(params):
    """
    パラメータの良さを評価
    戻り値が小さいほど良い（最小化問題）
    """
    result = run_simulation(params)

    # 例: 目標値との誤差を計算
    error = abs(result - target)
    return error
```

### 3. 全探索と最良解の選択

```python
best_cost = infinity
best_params = None

for params in all_combinations:
    cost = cost_function(params)
    if cost < best_cost:
        best_cost = cost
        best_params = params

return best_params
```

## 本プロジェクトでの実装

`tools/eskf_debug/optimize_params.py` での実装例:

### パラメータグリッドの定義

```python
param_grid = {
    'flow_noise': [0.01, 0.05, 0.1, 0.2, 0.5],      # 5値
    'accel_noise': [0.05, 0.1, 0.2, 0.5],           # 4値
    'gyro_noise': [0.0005, 0.001, 0.002, 0.005],    # 4値
    'tof_noise': [0.001, 0.002, 0.005, 0.01],       # 4値
    'flow_rad_per_pixel': [0.00185, 0.00205, 0.00226], # 3値
}

# 総組み合わせ数: 5 × 4 × 4 × 4 × 3 = 960通り
```

### コスト関数の設計

```python
def compute_cost(metrics, target_range=0.20):
    """
    20cm四方移動テストの再現精度を評価

    評価基準:
    1. 位置範囲が目標(20cm)に近いか
    2. 原点に戻れているか
    3. 姿勢が安定しているか
    """

    # 位置範囲誤差（非対称ペナルティ）
    x_range_err = metrics['pos_x_range'] - target_range

    # 20cm未達は3倍のペナルティ（過小評価を防ぐ）
    if x_range_err < 0:
        x_range_cost = abs(x_range_err) * 3.0
    else:
        x_range_cost = x_range_err

    # 原点復帰誤差
    return_cost = metrics['final_dist'] / target_range

    # 姿勢安定性
    att_cost = (abs(metrics['roll']) + abs(metrics['pitch'])) / 10.0

    # 重み付き合計
    total_cost = range_cost * 4.0 + return_cost * 2.0 + att_cost * 0.5

    return total_cost
```

### 並列実行による高速化

```python
from concurrent.futures import ProcessPoolExecutor

with ProcessPoolExecutor(max_workers=8) as executor:
    futures = {executor.submit(evaluate, task): task for task in tasks}

    for future in as_completed(futures):
        params, metrics, cost = future.result()
        results.append((params, metrics, cost))
```

## アルゴリズムの特徴

### メリット

| 特徴 | 説明 |
|------|------|
| **実装が簡単** | ループで全組み合わせを評価するだけ |
| **最適解の保証** | グリッド上では確実に最良解を発見 |
| **並列化が容易** | 各評価は独立しており、完全並列化可能 |
| **再現性がある** | 同じグリッドで同じ結果が得られる |
| **局所解に陥らない** | 全探索なので局所最適に囚われない |

### デメリット

| 特徴 | 説明 |
|------|------|
| **計算量の爆発** | パラメータ数に対して指数的に増加 |
| **離散化の限界** | グリッド点間の最適値は見つからない |
| **非効率** | 明らかに悪い領域も探索してしまう |

### 計算量の例

```
パラメータ数: n
各パラメータの候補数: k

総組み合わせ数 = k^n

例:
- 3パラメータ × 5候補 = 125通り ← 現実的
- 5パラメータ × 5候補 = 3,125通り ← まだ可能
- 10パラメータ × 5候補 = 9,765,625通り ← 困難
- 20パラメータ × 5候補 = 約95兆通り ← 不可能
```

## 他の最適化手法との比較

### 1. ランダムサーチ（Random Search）

```
特徴: パラメータ空間からランダムにサンプリング
利点: 高次元でも適用可能、計算量を制御できる
欠点: 最適解の保証なし
```

```python
for _ in range(n_iterations):
    params = {key: random.choice(values) for key, values in param_grid.items()}
    cost = evaluate(params)
```

### 2. ベイズ最適化（Bayesian Optimization）

```
特徴: 過去の評価結果から次の探索点を賢く選択
利点: 少ない評価回数で良い解を発見
欠点: 実装が複雑、並列化が難しい
```

```
評価履歴 → ガウス過程回帰 → 獲得関数最大化 → 次の探索点
```

### 3. 進化的アルゴリズム（Genetic Algorithm）

```
特徴: 生物の進化を模倣した探索
利点: 複雑な探索空間に対応、局所解を脱出可能
欠点: ハイパーパラメータの調整が必要
```

```
初期集団 → 選択 → 交叉 → 突然変異 → 次世代
```

### 4. 勾配降下法（Gradient Descent）

```
特徴: コスト関数の勾配方向に更新
利点: 連続最適化に最適、収束が速い
欠点: 勾配計算が必要、局所解に陥りやすい
```

```
θ_new = θ_old - learning_rate × ∇cost(θ)
```

### 比較表

| 手法 | 最適解保証 | 計算効率 | 実装難易度 | 高次元対応 |
|------|-----------|---------|-----------|-----------|
| グリッドサーチ | ○（グリッド上） | × | 簡単 | × |
| ランダムサーチ | × | ○ | 簡単 | ○ |
| ベイズ最適化 | × | ◎ | 難しい | △ |
| 遺伝的アルゴリズム | × | ○ | 中程度 | ○ |
| 勾配降下法 | △（局所解） | ◎ | 中程度 | ◎ |

## 実践的なテクニック

### 1. 粗→細の2段階探索

```python
# Stage 1: 粗いグリッドで大域探索
coarse_grid = {
    'param': [0.1, 0.5, 1.0, 5.0, 10.0]  # 広い範囲
}
best_coarse = grid_search(coarse_grid)

# Stage 2: 最良点周辺で細かく探索
fine_grid = {
    'param': [best * 0.5, best * 0.7, best, best * 1.4, best * 2.0]
}
best_fine = grid_search(fine_grid)
```

本プロジェクトでも `fine_tune()` 関数で実装:

```python
def fine_tune(input_file, best_params, target_range=0.20):
    """最良パラメータ周辺を細かく探索"""
    fine_grid = {}
    for key, value in best_params.items():
        # ±50%の範囲で5段階
        fine_grid[key] = [value * m for m in [0.5, 0.7, 1.0, 1.4, 2.0]]

    return grid_search(fine_grid)
```

### 2. 対数スケールの使用

ノイズパラメータなど桁が大きく変わる値には対数スケールが有効:

```python
# 線形スケール（不適切）
noise = [0.001, 0.002, 0.003, ..., 0.100]  # 低い値の解像度が不足

# 対数スケール（適切）
noise = [0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
# または
noise = np.logspace(-3, -1, 7)  # 10^-3 から 10^-1 まで7点
```

### 3. 早期終了（Early Stopping）

明らかに悪い結果は途中で打ち切り:

```python
def evaluate_with_early_stop(params, threshold):
    for step in simulation_steps:
        if current_cost > threshold * 2:
            return float('inf')  # 早期終了
    return final_cost
```

### 4. キャッシュの活用

同じパラメータの再評価を防止:

```python
from functools import lru_cache

@lru_cache(maxsize=10000)
def cached_evaluate(params_tuple):
    params = dict(params_tuple)
    return evaluate(params)
```

## グリッドサーチが適している場面

1. **パラメータ数が少ない**（3〜5個程度）
2. **各パラメータの有効範囲が既知**
3. **評価が比較的高速**（数秒以内）
4. **最適解の保証が必要**
5. **並列計算リソースが豊富**

## まとめ

グリッドサーチは最もシンプルで信頼性の高い最適化手法です。本プロジェクトのESKFパラメータ最適化では:

- **5パラメータ × 3〜5候補** = 約1000通りの探索
- **8並列実行**で数分で完了
- **粗→細の2段階探索**で精度向上
- **非対称コスト関数**で目標達成を重視

計算量が許容範囲であれば、グリッドサーチは確実に最良のパラメータを発見できる優れた手法です。

## 参考文献

- Bergstra, J., & Bengio, Y. (2012). Random Search for Hyper-Parameter Optimization. JMLR.
- Snoek, J., Larochelle, H., & Adams, R. P. (2012). Practical Bayesian Optimization of Machine Learning Algorithms. NeurIPS.
