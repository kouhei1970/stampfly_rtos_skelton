# シミュレーテッドアニーリング入門

## 概要

シミュレーテッドアニーリング（Simulated Annealing, SA）は、金属の焼きなまし（アニーリング）過程から着想を得た確率的最適化アルゴリズムである。局所解に陥りやすい最急降下法の欠点を克服し、グローバル最適解を探索できる。

## 金属の焼きなましとの類似性

### 物理現象
金属を高温に加熱してからゆっくり冷却すると、原子が最も安定したエネルギー状態（結晶構造）に配列する。

- **高温時**: 原子は激しく振動し、高エネルギー状態も許容される
- **冷却過程**: 徐々にエネルギーの低い状態へ移行
- **低温時**: 最低エネルギー状態（グローバル最適）に収束

### 最適化への対応

| 物理現象 | 最適化問題 |
|----------|-----------|
| エネルギー | コスト関数 |
| 温度 | 探索の自由度 |
| 原子配置 | パラメータ |
| 最低エネルギー状態 | 最適解 |

## アルゴリズム

### 基本手順

```
1. 初期解 x を設定
2. 初期温度 T を設定（十分高い値）
3. 以下を繰り返す:
   a. 現在の解 x の近傍から新しい解 x' をランダムに生成
   b. エネルギー差を計算: ΔE = f(x') - f(x)
   c. 遷移判定:
      - ΔE < 0 なら（改善）: x' を採用
      - ΔE >= 0 なら（改悪）: 確率 exp(-ΔE/T) で x' を採用
   d. 温度を下げる: T = α * T （0 < α < 1）
4. 温度が十分低くなったら終了
```

### 遷移確率（メトロポリス基準）

改悪解を受け入れる確率:

```
P(accept) = exp(-ΔE / T)
```

- **T が高い時**: P ≈ 1 → 改悪解もほぼ受け入れる（広い探索）
- **T が低い時**: P ≈ 0 → 改悪解は拒否（局所探索）

### 温度スケジュール例

```python
# 指数冷却
T = T0 * alpha^k  # alpha = 0.95〜0.99

# 対数冷却（理論的に最適だが遅い）
T = T0 / log(1 + k)

# 線形冷却
T = T0 - k * delta
```

## 具体例：ESKFパラメータ最適化

### 問題設定
- **パラメータ**: gyro_noise, accel_noise, flow_noise など7次元
- **コスト関数**: 位置誤差 + Roll誤差 × 0.3
- **目標**: コストを最小化するパラメータを発見

### Pythonコード例

```python
import numpy as np

def simulated_annealing(cost_func, x0, bounds, T0=100, alpha=0.95, max_iter=1000):
    """
    シミュレーテッドアニーリング

    Args:
        cost_func: コスト関数 f(x) -> float
        x0: 初期解（numpy配列）
        bounds: パラメータ範囲 [(min, max), ...]
        T0: 初期温度
        alpha: 冷却率
        max_iter: 最大イテレーション数

    Returns:
        best_x: 最良解
        best_cost: 最良コスト
    """
    x = x0.copy()
    cost = cost_func(x)

    best_x = x.copy()
    best_cost = cost

    T = T0

    for k in range(max_iter):
        # 近傍解を生成（正規分布で摂動）
        x_new = x + np.random.normal(0, T * 0.1, size=x.shape)

        # 範囲制限
        for i, (lo, hi) in enumerate(bounds):
            x_new[i] = np.clip(x_new[i], lo, hi)

        cost_new = cost_func(x_new)

        # エネルギー差
        delta_E = cost_new - cost

        # 遷移判定（メトロポリス基準）
        if delta_E < 0:
            # 改善 → 必ず採用
            accept = True
        else:
            # 改悪 → 確率的に採用
            p = np.exp(-delta_E / T)
            accept = np.random.random() < p

        if accept:
            x = x_new
            cost = cost_new

            if cost < best_cost:
                best_x = x.copy()
                best_cost = cost

        # 温度を下げる
        T = T * alpha

    return best_x, best_cost
```

### 実行例

```python
# コスト関数（ESKFシミュレーション）
def cost_function(params):
    result = run_eskf(params)
    return result['dist'] * 100 + abs(result['roll']) * 0.3

# 初期値とパラメータ範囲
x0 = np.array([0.001, 0.1, 0.00005, 0.001, 0.1, 0.002, 1.0])
bounds = [
    (0.0001, 0.01),   # gyro_noise
    (0.01, 0.5),      # accel_noise
    (1e-5, 0.1),      # gyro_bias_noise
    (1e-5, 0.1),      # accel_bias_noise
    (0.005, 0.5),     # flow_noise
    (0.001, 0.1),     # tof_noise
    (0.1, 5.0),       # accel_att_noise
]

# 最適化実行
best_params, best_cost = simulated_annealing(
    cost_function, x0, bounds,
    T0=10, alpha=0.99, max_iter=1000
)
```

## 最急降下法との比較

### 最急降下法の問題

```
      コスト
        ^
        |    /\
        |   /  \      /\
        |  /    \    /  \
        | /      \  /    \_____ グローバル最適
        |/        \/
        +-------------------> パラメータ
              ^
              |
         局所最適（ここで停止）
```

最急降下法は勾配に従って下るだけなので、最初に見つけた谷（局所最適）から抜け出せない。

### SAの動作

```
高温時：
        ^
        |    /\
        |   /  \  ←→ /\        改悪方向にも
        |  /    \←→/  \        ジャンプ可能
        | /      \/    \____
        |/
        +-------------------->

低温時：
        ^
        |    /\
        |   /  \      /\
        |  /    \    /  \
        | /      \  /    \____  この谷に収束
        |/        \/     ↓
        +-------------------->
```

高温時は「登り」も許容するため、局所解を脱出してより良い解を探索できる。

## パラメータ調整のコツ

### 初期温度 T0

- **高すぎる**: 収束が遅い
- **低すぎる**: 局所解に陥る

目安: 初期のコスト変動量の数倍程度

### 冷却率 α

- **α ≈ 1（例: 0.99）**: ゆっくり冷却 → 精度高いが遅い
- **α ≈ 0.9**: 速く冷却 → 速いが局所解のリスク

### イテレーション数

温度が十分下がるまで必要:
```
T_final = T0 * alpha^N
N = log(T_final/T0) / log(alpha)
```

例: T0=100, T_final=0.01, α=0.99 → N ≈ 920回

## 発展的トピック

### 1. 適応的冷却

収束状況に応じて冷却率を調整:
```python
if 改善が続いている:
    alpha = 0.99  # ゆっくり冷却
else:
    alpha = 0.95  # 速く冷却
```

### 2. 再加熱（Reheating）

局所解に陥ったら温度を上げて脱出:
```python
if 改善がN回続かない:
    T = T * 2  # 再加熱
```

### 3. 並列SA

複数の初期点から同時に実行し、最良解を選択。

### 4. Dual Annealing

scipy.optimize.dual_annealing は、SAに局所探索を組み合わせた高性能版:
- グローバル探索（SA）で有望領域を発見
- 局所探索（L-BFGS-B）で精密に収束

## 今回の最適化結果

### 比較

| 手法 | Cost | Roll | dist |
|------|------|------|------|
| 最急降下法 | 1.74 | 3.58° | 0.66cm |
| **SA** | **1.01** | **3.35°** | **0.00cm** |

### 発見されたパラメータの違い

最急降下法は `gyro_bias_noise = 0.00005`（デフォルト）から動かなかったが、SAは `gyro_bias_noise = 0.00331` という全く異なる最適解を発見した。

これは、最急降下法がこのパラメータに関して平坦な領域（勾配≈0）に居たため動けなかったのに対し、SAはランダムなジャンプでこの平坦領域を脱出できたことを示している。

## まとめ

1. **SAは局所解を脱出できる**: 高温時に改悪解も受け入れることで、局所解に陥らない
2. **温度スケジュールが重要**: 適切な初期温度と冷却率の設定が必要
3. **計算コストは高い**: 最急降下法より多くの評価が必要
4. **多峰性問題に有効**: 複数の局所解がある場合に特に効果的

## 参考文献

- Kirkpatrick, S., Gelatt, C. D., & Vecchi, M. P. (1983). Optimization by Simulated Annealing. Science, 220(4598), 671-680.
- scipy.optimize.dual_annealing documentation
