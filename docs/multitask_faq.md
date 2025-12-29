# マルチタスクに関するFAQ

## 概要

RTOSを使用したマルチタスク環境での疑問に答えるドキュメントです。特にセンサI/Oと制御タスクの関係について解説します。

---

## Q1: I/Oに時間がかかる場合、他のタスクは待たされるのか？

### 質問

> ToFセンサはデータ取得を予約してから読み出し可能になるまで30msかかる。この間は待つ必要はない。ただし、データを読み出す際に6msかかってしまう。33ms周期で距離情報を取得するとして、RTOSを用いても6ms他の処理を待たせることになるのか？

### 回答

**結論: 適切に実装されていれば、他の高優先度タスクは待たされない。**

ただし、これは「ブロッキングの種類」と「優先度の関係」によって異なります。

---

### ブロッキングの種類

#### 1. ビジーウェイト（CPU占有型）← 悪い例

```cpp
// 悪い例: CPUを占有し続ける
while (!i2c_data_ready()) {
    // 何もせずループ → CPUを100%使用
}
```

この場合、**低優先度タスクは完全にブロックされます**。

#### 2. OSブロッキング（CPU解放型）← 良い例

```cpp
// 良い例: I2Cドライバ内部でブロック中に他タスクに譲る
esp_err_t ret = i2c_master_receive(handle, buffer, len, timeout);
// ↑ この間、タスクはブロック状態になり、スケジューラが他のタスクを実行
```

ESP-IDFのI2Cドライバは内部で`xSemaphoreTake()`を使用しており、**待機中は他のタスクにCPUを譲ります**。

---

### 図解: ToFタスク（優先度14）とIMUタスク（優先度24）の関係

```
Time (ms)  0    1    2    3    4    5    6    7    8    9   10
           |----|----|----|----|----|----|----|----|----|----|

IMUTask    [■]      [■]      [■]      [■]      [■]      [■]
(400Hz)     ↑        ↑        ↑        ↑        ↑        ↑
           2.5ms周期で起床

ToFTask    [===========I2C読み出し(6ms)===========]
(30Hz)      ↑ 開始                                ↑ 完了

実際の動作:
                   ↓プリエンプト    ↓プリエンプト
ToFTask    [==][中断][==][中断][==][中断][=====]
IMUTask        [■]       [■]       [■]

→ IMUTaskは2.5ms周期で正常に動作
→ ToFTaskのI2C読み出しは細切れになるが、最終的に完了
```

**重要**: I2Cドライバが適切にブロッキングを実装していれば、高優先度タスク（IMU）は低優先度タスク（ToF）のI/O中でも割り込んで実行できます。

---

### ESP-IDFのI2Cドライバの動作

ESP-IDF v5.xのI2Cマスタードライバは以下のように動作します：

```cpp
// i2c_master_receive() の内部動作（簡略化）
esp_err_t i2c_master_receive(...)
{
    // 1. トランザクションをキューに追加
    // 2. セマフォで完了を待つ（この間、他タスクに譲る）
    xSemaphoreTake(done_sem, timeout);
    // 3. 完了後に戻る
}
```

この`xSemaphoreTake()`の間、ToFTaskは**ブロック状態**となり：
- スケジューラはToFTaskを「実行可能」リストから除外
- 他の実行可能なタスク（IMUTaskなど）にCPUを割り当て
- I2C完了時にISRがセマフォを解放し、ToFTaskが再び実行可能に

---

### 注意が必要なケース

#### ケース1: 同じI2Cバスを共有

```
問題: IMUタスクもI2Cを使用する場合

Time →
ToFTask:  [===I2C読み出し===]
IMUTask:      [起床][I2C待ち][I2C実行]
                    ↑ここで待たされる
```

**対策**:
- IMUはSPIを使用（本プロジェクトの構成）
- I2Cバスを使うタスク間で優先度を考慮

#### ケース2: クリティカルセクション中

```cpp
// I2Cドライバ内部でクリティカルセクションがある場合
portENTER_CRITICAL(&i2c_spinlock);
// この間は割り込み禁止、プリエンプション不可
portEXIT_CRITICAL(&i2c_spinlock);
```

ESP-IDFのI2Cドライバはクリティカルセクションを最小限に抑えていますが、完全にゼロではありません。

---

## Q2: 6msのI/Oを避ける方法はあるのか？

### 方法1: DMA転送（推奨）

```cpp
// DMAを使用すると、CPU介入なしでデータ転送が可能
i2c_master_bus_config_t config = {
    // ...
    .flags.enable_internal_pullup = true,
};
// ESP-IDF v5.x のI2Cドライバは内部でDMA/FIFOを活用
```

DMAを使用すると、転送中はCPUが完全に解放されます。ただし、ToFの6msがI2C転送だけでなく内部処理を含む場合は効果が限定的。

### 方法2: 非同期I/O

```cpp
// 将来的なAPIの例（ESP-IDFでは一部対応）
i2c_master_receive_async(handle, buffer, len, callback);

// コールバックで通知
void i2c_callback(i2c_master_event_t event, void* arg) {
    xSemaphoreGiveFromISR(tof_data_ready_sem, NULL);
}
```

### 方法3: 低優先度での実行（現在の実装）

```cpp
// ToFTask を低優先度で実行
xTaskCreatePinnedToCore(ToFTask, "ToF", STACK_SIZE, nullptr,
                        PRIORITY_TOF_TASK,  // 14（低優先度）
                        &g_tof_task_handle, 1);
```

高優先度タスク（IMU: 24、Control: 23）は、ToFのI/O中でもプリエンプトして実行できます。

### 方法4: 別コアで実行

```cpp
// Core 0でToFを実行（IMU/ControlはCore 1）
xTaskCreatePinnedToCore(ToFTask, "ToF", STACK_SIZE, nullptr,
                        PRIORITY_TOF_TASK,
                        &g_tof_task_handle, 0);  // Core 0
```

これにより、ToFのI/Oが**完全に独立**して実行されます。

---

## Q3: 現在のプロジェクト構成でToFは問題ないか？

### 現在の構成

| タスク | 優先度 | Core | I/O |
|--------|--------|------|-----|
| IMUTask | 24 | 1 | SPI |
| ControlTask | 23 | 1 | なし |
| ToFTask | 14 | 1 | I2C |

### 分析

```
ToFの6ms読み出し中に何が起こるか：

1. ToFTask が I2C 読み出しを開始
2. I2C ドライバがセマフォ待ちに入る（ブロック状態）
3. スケジューラが次に優先度の高い実行可能タスクを選択
4. 2.5ms後、ESP Timer が IMUTask を起床
5. IMUTask（優先度24）が ToFTask（優先度14）をプリエンプト
6. IMUTask が完了し、ControlTask を起床
7. ControlTask が完了
8. ToFTask が再開（まだI2C待ち中なら継続待機）
```

**結論: 問題なし。** IMUTask/ControlTaskは400Hzで正常に動作します。

### 検証方法

```cpp
// IMUTaskで実際の周期を監視
static void IMUTask(void* pvParameters)
{
    static uint32_t last_us = 0;

    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        uint32_t now_us = esp_timer_get_time();
        if (last_us != 0) {
            uint32_t period = now_us - last_us;
            if (period > 2600 || period < 2400) {  // ±100us以上のずれ
                ESP_LOGW(TAG, "IMU period jitter: %lu us", period);
            }
        }
        last_us = now_us;

        // 処理...
    }
}
```

---

## Q4: I2Cの6msは本当にブロックを解放しているのか？

### 確認方法

```cpp
static void ToFTask(void* pvParameters)
{
    while (true) {
        // 測定開始
        tof.startMeasurement();

        // 30ms待機（データ準備中）
        vTaskDelay(pdMS_TO_TICKS(30));

        // データ読み出し（6ms）
        uint32_t start = esp_timer_get_time();
        tof.readDistance();  // ← ここが6msかかる
        uint32_t elapsed = esp_timer_get_time() - start;

        ESP_LOGI(TAG, "ToF read took %lu us", elapsed);
    }
}
```

### 確認すべき点

1. **6msの内訳を調べる**
   - I2C転送時間（データ量 × ボーレートから計算）
   - センサ内部処理時間
   - ドライバのオーバーヘッド

2. **I2Cクロックの確認**
   ```cpp
   // 400kHz (Fast Mode) の場合
   // 100バイトの転送時間 = 100 × 8bit / 400kHz = 2ms
   // 6msかかるなら、4msはセンサ内部処理 or 複数転送
   ```

3. **VL53L3CXの仕様確認**
   - データシートによると、結果レジスタの読み出しは複数回のI2C転送を含む
   - 内部で`vTaskDelay()`が呼ばれていないか確認

---

## Q5: より確実に高優先度タスクを守る方法は？

### 方法1: 割り込みレベルの活用

```cpp
// 高優先度割り込みでIMUを直接処理
// （タスクよりさらに優先度が高い）

void IRAM_ATTR imu_data_ready_isr(void* arg)
{
    // 最小限の処理
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_imu_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### 方法2: タスクの分離

```cpp
// Core 0: I/O集約タスク
ToFTask, BaroTask, PowerTask → Core 0

// Core 1: リアルタイムタスク
IMUTask, ControlTask → Core 1
```

### 方法3: I/Oタイムアウトの設定

```cpp
// I/Oが長すぎる場合はタイムアウト
esp_err_t ret = i2c_master_receive(handle, buffer, len,
                                    pdMS_TO_TICKS(10));  // 10msでタイムアウト
if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGW(TAG, "I2C timeout, skip this cycle");
}
```

---

## Q6: 6msの読み出しを分割できないか？

### 分割読み出しの例

```cpp
// VL53L3CXの結果レジスタを分割読み出し
static void ToFTask(void* pvParameters)
{
    while (true) {
        tof.startMeasurement();
        vTaskDelay(pdMS_TO_TICKS(30));

        // 分割1: ステータス読み出し（1ms）
        tof.readStatus();
        taskYIELD();  // 明示的に他タスクに譲る

        // 分割2: 距離データ読み出し（2ms）
        tof.readDistanceData();
        taskYIELD();

        // 分割3: シグナル品質読み出し（2ms）
        tof.readSignalData();

        // データ処理
        processToFData();
    }
}
```

**注意**: VL53L3CXのドライバAPIがこのような分割を許容するかは要確認。

---

## まとめ

| 疑問 | 回答 |
|------|------|
| 6msのI/O中、他タスクは待たされるか？ | 高優先度タスクは待たされない（プリエンプション） |
| 低優先度タスクは待たされるか？ | はい、ToFより低いタスクは待たされる |
| この6msを避ける方法は？ | 別コア実行、DMA、非同期I/O |
| 現在の構成で問題はあるか？ | なし（IMU/Controlは高優先度） |

---

## 本プロジェクトへの推奨事項

### 現状で問題ない理由

1. **優先度分離**: IMU(24) > Control(23) >> ToF(14)
2. **I/O分離**: IMU=SPI、ToF=I2C（バス競合なし）
3. **OSブロッキング**: ESP-IDFのI2Cドライバは待機中にCPUを解放

### さらに安心を得るために

1. **Core分離を検討**
   ```cpp
   // ToFをCore 0に移動
   xTaskCreatePinnedToCore(ToFTask, ..., 0);
   ```

2. **実行時間監視を実装**
   - `realtime_considerations.md` の監視コードを追加

3. **ジッター計測**
   - 実機で400Hz周期のジッターを計測し、問題がないことを確認

---

*最終更新: 2025-12-29*
