# ESKFセンサーヘルスチェック設計

## 背景と動機

### 問題の発見

WiFi WebSocketテレメトリ実装後、Teleplotでリアルタイムデータを監視していたところ、**静止状態でもESKFが発散する**問題が発見された。

症状:
- 静止状態でPosition Z が約8秒で100mに到達
- 100m到達時にリセットされるが、再び発散を繰り返す
- バイナリログ（binlog on）取得中は安定

### 根本原因の分析

調査の結果、以下の問題が判明した:

1. **初期化タイミングの問題**
   - タスク開始直後、センサーが安定する前にESKF処理が開始
   - センサーノイズや初期化中の異常値がESKF共分散を破壊
   - `binlog on`コマンドがESKFをリセットするため、バイナリログ中は安定していた

2. **発散検出後のリセット不完全**
   - 発散検出時のリセットが100回に1回しか実行されていなかった（バグ）
   - リセット時に`setMagReferenceFromBuffer()`が呼ばれていなかった

3. **センサー異常時の観測更新**
   - センサーが異常値を返しても観測更新が実行される
   - 異常な観測値がESKFの共分散を破壊し、発散につながる

### 修正と今後の課題

初期化タイミングとリセットバグは修正済み:
- `g_eskf_ready`フラグでセンサー安定後にESKF処理を開始
- 発散時は常にリセット実行

しかし、運用中のセンサー異常に対する対策が必要:
- センサーのヘルスチェックによる観測更新のスキップ
- ESKFモードの動的切り替え

---

## 1. センサーと観測更新の関係

ESKFは15状態量を推定する:
- 位置 (3): X, Y, Z [m]
- 速度 (3): Vx, Vy, Vz [m/s]
- 姿勢誤差 (3): Roll, Pitch, Yaw [rad]
- ジャイロバイアス (3): [rad/s]
- 加速度バイアス (3): [m/s²]

各センサーの役割:

| センサー | 更新関数 | 更新頻度 | 推定状態への影響 |
|---------|---------|---------|----------------|
| **IMU (加速度+ジャイロ)** | `predict()` | 400Hz | 全状態量（必須） |
| **IMU (加速度計姿勢補正)** | `updateAccelAttitude()` | 400Hz | Roll/Pitch、加速度バイアス |
| **ToF (下方距離)** | `updateToF()` | 30Hz | 高度(Z)、速度Z |
| **オプティカルフロー** | `updateFlowRaw()` | 100Hz | 水平速度(Vx, Vy)、水平位置(X, Y) |
| **磁気センサー** | `updateMag()` | 10Hz | Yaw |
| **気圧センサー** | `updateBaro()` | 50Hz | 高度(Z) ※現在無効 |

---

## 2. センサーヘルス状態の定義

```cpp
enum class SensorHealth {
    HEALTHY,   // 正常にデータ取得、値が妥当
    STALE,     // タイムアウト（データが古い）
    INVALID,   // データ取得OK だが値が異常（NaN, 範囲外）
    FAILED     // 初期化失敗またはI/O エラー
};
```

---

## 3. センサー別ヘルスチェック条件

| センサー | HEALTHY条件 | STALE検出 | INVALID検出 |
|---------|------------|----------|-------------|
| IMU | `readSensorData() == ESP_OK` | 連続3サイクル失敗 (7.5ms) | NaN または \|accel\| > 160m/s² |
| ToF | `status <= 4` && 距離 > 10mm | 連続10サイクルデータなし (330ms) | 距離 > 4m または 距離 < 10mm |
| OptFlow | `squal >= 30` (OutlierDetector) | 連続10サイクル品質不良 (100ms) | dx/dy > 127 (異常値) |
| Mag | `read() == ESP_OK` | 連続10サイクル失敗 (100ms) | \|mag\| < 10uT または > 100uT |
| Baro | `read() == ESP_OK` | 連続10サイクル失敗 (200ms) | 高度 < -1000m または > 10000m |

### IMUの特殊性

IMUはESKF予測ステップに必須。IMU異常時はESKF処理自体を停止する。

```cpp
// IMUが異常の場合はESKF処理をスキップ
if (!g_imu_task_healthy) {
    // フォールバック: 単純姿勢推定器を使用
    if (g_attitude_est.isInitialized()) {
        g_attitude_est.update(a, g, dt);
    }
    return;
}
```

---

## 4. ESKFモード定義

| モード | 説明 | 必要センサー |
|-------|-----|------------|
| **FULL** | 全機能動作 | IMU + ToF + Flow + Mag |
| **NO_MAG** | Yaw固定（現在のデフォルト） | IMU + ToF + Flow |
| **NO_FLOW** | 水平位置/速度はドリフト | IMU + ToF + Mag |
| **ALTITUDE_ONLY** | 高度のみ信頼 | IMU + ToF |
| **ATTITUDE_ONLY** | 姿勢のみ信頼 | IMU |
| **DEGRADED** | 予測のみ（観測なし） | IMU |
| **FAILED** | ESKF停止 | - |

---

## 5. センサーヘルスとESKF動作の対応表

```
IMU    ToF    Flow   Mag    → ESKFモード         → 有効な観測更新
─────────────────────────────────────────────────────────────────────
OK     OK     OK     OK     → FULL              → ToF, Flow, Mag, AccelAtt
OK     OK     OK     NG     → NO_MAG            → ToF, Flow, AccelAtt
OK     OK     NG     OK     → NO_FLOW           → ToF, Mag, AccelAtt
OK     OK     NG     NG     → ALTITUDE_ONLY     → ToF, AccelAtt
OK     NG     OK     OK     → NO_TOF*           → Flow(Vxy), Mag, AccelAtt
OK     NG     OK     NG     → NO_TOF_NO_MAG*    → Flow(Vxy), AccelAtt
OK     NG     NG     OK     → ATTITUDE_ONLY     → Mag, AccelAtt
OK     NG     NG     NG     → DEGRADED          → AccelAtt のみ
NG     -      -      -      → FAILED            → ESKF停止
```

**注意:** ToFがNGの場合、オプティカルフローの距離スケーリングが不正確になる。
Flow更新時にToF距離を使用するため、ToF異常時はFlow更新も信頼性が低下。

---

## 6. 観測更新スキップの実装

各観測更新関数呼び出し前にヘルスチェックを行う:

```cpp
// === IMUTask内のESKF処理 ===

// 1. 予測ステップ（IMU必須）
if (!g_imu_task_healthy) {
    // ESKF処理をスキップ、フォールバック使用
    return;
}
g_eskf.predict(a, g, 0.0025f);

// 2. 加速度計姿勢補正（常に実行、IMUデータ使用）
g_eskf.updateAccelAttitude(a);

// 3. ToF更新
if (g_tof_task_healthy && g_tof_data_ready) {
    if (g_tof_data_cache > 0.01f && g_tof_data_cache < 4.0f) {
        g_eskf.updateToF(g_tof_data_cache);
    }
    g_tof_data_ready = false;
}

// 4. オプティカルフロー更新（ToFも必要）
if (g_optflow_task_healthy && g_tof_task_healthy) {
    if (OutlierDetector::isFlowValid(squal)) {
        float distance = tof_bottom;
        if (distance > 0.02f && distance < 4.0f) {
            g_eskf.updateFlowRaw(flow_dx, flow_dy, distance, dt, g.x, g.y);
        }
    }
}

// 5. 磁気更新
if (g_mag_task_healthy && g_mag_ref_set && g_mag_data_ready) {
    g_eskf.updateMag(g_mag_data_cache);
    g_mag_data_ready = false;
}
```

---

## 7. 共分散行列への影響

センサー異常時に観測更新をスキップすると:

1. **該当状態量の共分散(P)が増加し続ける**
   - 予測ステップで常にプロセスノイズQが加算される
   - 観測更新がないとカルマンゲインKによる減少がない

2. **推定値の信頼度が低下**
   - 共分散の対角成分が推定誤差の分散を表す
   - 値が大きいほど不確実

3. **センサー復帰時の挙動**
   - 共分散が大きいとカルマンゲインが大きくなる
   - 観測値に急激に追従する可能性

### 対策案

```cpp
// 共分散の上限を設定（発散防止）
constexpr float MAX_POS_VAR = 100.0f;    // 位置 10m σ
constexpr float MAX_VEL_VAR = 25.0f;     // 速度 5m/s σ
constexpr float MAX_ATT_VAR = 0.25f;     // 姿勢 ~30° σ

void ESKF::enforceCovarianceLimits() {
    for (int i = 0; i < 3; i++) {
        P_(POS_X + i, POS_X + i) = std::min(P_(POS_X + i, POS_X + i), MAX_POS_VAR);
        P_(VEL_X + i, VEL_X + i) = std::min(P_(VEL_X + i, VEL_X + i), MAX_VEL_VAR);
        P_(ATT_X + i, ATT_X + i) = std::min(P_(ATT_X + i, ATT_X + i), MAX_ATT_VAR);
    }
}
```

---

## 8. 実装計画

### Phase 1: ヘルスフラグの設定（優先度: 高）

各センサータスクで `g_xxx_task_healthy` フラグを設定:

```cpp
// ToFTask内
static int consecutive_errors = 0;
if (ret == ESP_OK && status <= 4) {
    consecutive_errors = 0;
    g_tof_task_healthy = true;
} else {
    if (++consecutive_errors >= 10) {
        g_tof_task_healthy = false;
    }
}
```

### Phase 2: 観測更新のガード（優先度: 高）

IMUTask内の各 `updateXxx()` 呼び出し前にヘルスチェック追加。
異常データによる共分散破壊を防止。

### Phase 3: ESKFモードの実装（優先度: 中）

```cpp
enum class ESKFMode {
    FULL, NO_MAG, NO_FLOW, ALTITUDE_ONLY, ATTITUDE_ONLY, DEGRADED, FAILED
};

ESKFMode determineESKFMode() {
    if (!g_imu_task_healthy) return ESKFMode::FAILED;
    if (!g_tof_task_healthy && !g_optflow_task_healthy && !g_mag_task_healthy)
        return ESKFMode::DEGRADED;
    // ... 他のモード判定
}
```

### Phase 4: デグレード通知（優先度: 低）

- LED/ブザーでセンサー異常を通知
- Telemetryパケットに現在のESKFモードを含める
- CLIコマンド `sensor health` で状態確認

---

## 9. テスト計画

1. **正常動作確認**
   - 全センサー正常時にFULLモードで動作
   - 各観測更新が正しい頻度で実行

2. **センサー異常シミュレーション**
   - ToFを手で覆って異常値を発生させる
   - オプティカルフローを覆って品質低下
   - 各センサー異常時にモードが正しく遷移

3. **発散耐性確認**
   - センサー異常時にESKFが発散しないこと
   - センサー復帰時に正常に推定再開

4. **長時間安定性**
   - 静止状態で10分以上の安定動作
   - Teleplotで発散がないことを確認

---

## 関連ドキュメント

- [ESKF ワークフロー](eskf_workflow.md)
- [リアルタイム考慮事項](realtime_considerations.md)
- [開発者ガイド](developer_guide.md)
