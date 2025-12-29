# リアルタイム処理の考慮事項

## 概要

飛行制御システムでは、状態推定と制御演算が**決まった周期で確実に実行される**ことが必須です。RTOSを使用しているからといって、リアルタイム性が自動的に保証されるわけではありません。

本ドキュメントでは、RTOSを使用したマルチタスクシステムでリアルタイム処理を実現するための注意点と監視方法について説明します。

---

## なぜRTOSだけでは安心できないのか

### 1. RTOSの「リアルタイム」の意味

RTOSの「リアルタイム」とは、**決定論的なスケジューリング**を意味します。つまり：

- 高優先度タスクが低優先度タスクに割り込める
- スケジューリングの挙動が予測可能

しかし、これは**処理が時間内に完了することを保証しない**。

```
❌ 誤解: RTOSを使えば処理は必ず間に合う
✅ 正解: RTOSは優先度に基づくスケジューリングを提供するだけ
```

### 2. デッドラインミスの原因

RTOSを使用していても、以下の原因でデッドラインミスが発生します：

#### a) 処理時間超過
```
タスク周期: 2.5ms (400Hz)
実際の処理時間: 3.0ms → デッドラインミス
```

#### b) 優先度逆転
低優先度タスクがミューテックスを保持している間、高優先度タスクがブロックされる。

```
Time →
Task H (高優先度):  [---wait for mutex---][run]
Task M (中優先度):  [========running========]
Task L (低優先度):  [hold mutex][-----preempted-----]

問題: Task Mが実行されている間、Task Hは待機し続ける
```

#### c) 割り込み遅延
ISR（割り込みサービスルーチン）が長時間実行されると、全タスクが遅延する。

#### d) メモリアロケーション
`malloc()`/`new`は非決定論的な時間がかかる可能性がある。

#### e) ブロッキングI/O
I2C/SPI通信のタイムアウト待ちがタスクをブロックする。

### 3. 本プロジェクトでの具体例

```cpp
// IMUTask - 400Hz (2.5ms周期)
static void IMUTask(void* pvParameters)
{
    while (true) {
        // セマフォ待ち（ESP Timerから起床）
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        // ここからの処理が2.5ms以内に完了する必要がある
        g_imu.read();           // SPI通信: ~200μs
        g_eskf.predict();       // ESKF予測: ~500μs
        g_eskf.updateXXX();     // 各種更新: ~300μs
        // ...

        xSemaphoreGive(g_control_semaphore);  // ControlTaskを起床
    }
}
```

もし処理が2.5msを超えると：
1. 次のESP Timerコールバックが待たされる
2. 制御周期がジッターを持つ
3. 状態推定の積分誤差が蓄積
4. 最悪の場合、機体が不安定に

---

## マルチタスクでリアルタイム処理を実現するための注意点

### 1. タスク設計の原則

#### a) 優先度の適切な割り当て

| 優先度 | タスク | 理由 |
|--------|--------|------|
| 最高 (24) | IMUTask | 状態推定の基盤、最も時間に敏感 |
| 高 (23) | ControlTask | 制御出力、IMU直後に実行 |
| 中 (15-20) | センサタスク | データ収集、多少の遅延は許容 |
| 低 (5-10) | LED/CLI | ユーザーインターフェース |

#### b) レートモノトニックスケジューリング (RMS)
```
原則: 周期が短いタスクに高い優先度を与える
```

400Hz > 100Hz > 50Hz > 30Hz > 10Hz

#### c) CPUコアの分離

```cpp
// Core 1: リアルタイムクリティカル
xTaskCreatePinnedToCore(IMUTask, "IMU", ..., 1);      // Core 1
xTaskCreatePinnedToCore(ControlTask, "Control", ..., 1);  // Core 1

// Core 0: 非クリティカル
xTaskCreatePinnedToCore(CLITask, "CLI", ..., 0);      // Core 0
xTaskCreatePinnedToCore(LEDTask, "LED", ..., 0);      // Core 0
```

### 2. 共有リソースの管理

#### a) ミューテックスの使用を最小限に

```cpp
// 悪い例: 長時間ロック
xSemaphoreTake(mutex, portMAX_DELAY);
expensive_computation();  // 長い処理
shared_data = result;
xSemaphoreGive(mutex);

// 良い例: 短時間ロック
expensive_computation();  // ロック外で計算
xSemaphoreTake(mutex, portMAX_DELAY);
shared_data = result;     // データコピーのみ
xSemaphoreGive(mutex);
```

#### b) 優先度継承ミューテックスの使用

```cpp
// FreeRTOSでは標準ミューテックスが優先度継承をサポート
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
```

#### c) ロックフリーデータ構造の検討

```cpp
// アトミック操作で単純なフラグを管理
volatile bool g_data_ready = false;

// ISRで
g_data_ready = true;

// タスクで
if (g_data_ready) {
    g_data_ready = false;
    process_data();
}
```

### 3. メモリ管理

#### a) 静的割り当ての推奨

```cpp
// 悪い例: 動的割り当て
void processData() {
    float* buffer = new float[1000];  // 非決定論的
    // ...
    delete[] buffer;
}

// 良い例: 静的割り当て
static float s_buffer[1000];  // コンパイル時に確保
void processData() {
    // s_bufferを使用
}
```

#### b) クラスメンバ変数の活用

```cpp
// ESKFの一時行列はメンバ変数として保持
class ESKF {
private:
    Matrix<15, 15> F_;      // 状態遷移行列
    Matrix<15, 15> Q_;      // プロセスノイズ
    Matrix<15, 15> temp1_;  // 一時計算用
    Matrix<15, 15> temp2_;  // 一時計算用
};
```

### 4. I/O処理の分離

#### a) DMAの活用

```cpp
// SPI通信をDMAで実行し、CPUを解放
spi_device_transmit(spi_handle, &transaction);  // DMA転送
```

#### b) タイムアウトの設定

```cpp
// 無限待機を避ける
if (xSemaphoreTake(sem, pdMS_TO_TICKS(10)) != pdTRUE) {
    // タイムアウト処理
    ESP_LOGW(TAG, "Sensor read timeout");
    return;
}
```

### 5. 割り込み処理

#### a) ISRは最小限に

```cpp
// ISR内では最小限の処理
void IRAM_ATTR timer_isr(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_imu_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

#### b) 割り込み優先度の管理

ESP32では割り込みレベルが設定可能。クリティカルな割り込みは高いレベルに。

---

## 処理時間の監視方法

### 1. 実行時間の計測

#### a) 基本的な計測

```cpp
static void IMUTask(void* pvParameters)
{
    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        uint32_t start_us = esp_timer_get_time();

        // === 処理 ===
        processIMU();
        runESKF();
        // ============

        uint32_t elapsed_us = esp_timer_get_time() - start_us;

        // デッドラインチェック (2500μs = 2.5ms)
        if (elapsed_us > 2500) {
            ESP_LOGW(TAG, "IMUTask overrun: %lu us", elapsed_us);
        }

        // 統計更新
        updateTaskStats(elapsed_us);

        xSemaphoreGive(g_control_semaphore);
    }
}
```

#### b) 統計情報の収集

```cpp
struct TaskStats {
    uint32_t exec_count;
    uint32_t overrun_count;
    uint32_t max_exec_us;
    uint32_t min_exec_us;
    uint64_t total_exec_us;

    float getAverageUs() const {
        return exec_count > 0 ? (float)total_exec_us / exec_count : 0;
    }

    float getOverrunRate() const {
        return exec_count > 0 ? (float)overrun_count / exec_count * 100 : 0;
    }
};

static TaskStats g_imu_stats = {0, 0, 0, UINT32_MAX, 0};

void updateTaskStats(uint32_t elapsed_us) {
    g_imu_stats.exec_count++;
    g_imu_stats.total_exec_us += elapsed_us;

    if (elapsed_us > g_imu_stats.max_exec_us) {
        g_imu_stats.max_exec_us = elapsed_us;
    }
    if (elapsed_us < g_imu_stats.min_exec_us) {
        g_imu_stats.min_exec_us = elapsed_us;
    }
    if (elapsed_us > 2500) {
        g_imu_stats.overrun_count++;
    }
}
```

### 2. CLIコマンドでの監視

```cpp
// CLI: taskstats コマンド
static void cmd_taskstats(int argc, char** argv, void* context)
{
    CLI* cli = static_cast<CLI*>(context);

    cli->print("=== Task Statistics ===\r\n");
    cli->print("IMUTask (400Hz, deadline 2500us):\r\n");
    cli->print("  Executions: %lu\r\n", g_imu_stats.exec_count);
    cli->print("  Overruns:   %lu (%.2f%%)\r\n",
               g_imu_stats.overrun_count,
               g_imu_stats.getOverrunRate());
    cli->print("  Exec time:  min=%lu, avg=%.1f, max=%lu us\r\n",
               g_imu_stats.min_exec_us,
               g_imu_stats.getAverageUs(),
               g_imu_stats.max_exec_us);
}
```

### 3. FreeRTOSランタイム統計

```cpp
// sdkconfig.defaults に追加
// CONFIG_FREERTOS_USE_TRACE_FACILITY=y
// CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y

// ランタイム統計の取得
char stats_buffer[1024];
vTaskGetRunTimeStats(stats_buffer);
printf("%s\n", stats_buffer);
```

### 4. ジッターの監視

```cpp
static void IMUTask(void* pvParameters)
{
    static uint32_t last_wakeup_us = 0;
    const uint32_t expected_period_us = 2500;

    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        uint32_t now_us = esp_timer_get_time();

        if (last_wakeup_us != 0) {
            uint32_t actual_period = now_us - last_wakeup_us;
            int32_t jitter = (int32_t)actual_period - (int32_t)expected_period_us;

            // ジッターが許容範囲外なら警告
            if (abs(jitter) > 100) {  // ±100μs以上
                ESP_LOGW(TAG, "IMU jitter: %+ld us", jitter);
            }

            updateJitterStats(jitter);
        }

        last_wakeup_us = now_us;

        // 処理...
    }
}
```

### 5. ウォッチドッグによる監視

```cpp
#include "esp_task_wdt.h"

static void IMUTask(void* pvParameters)
{
    // タスクウォッチドッグに登録 (タイムアウト5秒)
    esp_task_wdt_add(NULL);

    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        // 処理...

        // ウォッチドッグをリセット
        esp_task_wdt_reset();
    }
}
```

### 6. GPIO出力による外部計測

オシロスコープやロジックアナライザで計測する場合：

```cpp
#define DEBUG_GPIO 48  // 未使用GPIOピン

static void IMUTask(void* pvParameters)
{
    gpio_set_direction(DEBUG_GPIO, GPIO_MODE_OUTPUT);

    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);

        gpio_set_level(DEBUG_GPIO, 1);  // HIGH: 処理開始

        // 処理...

        gpio_set_level(DEBUG_GPIO, 0);  // LOW: 処理終了

        xSemaphoreGive(g_control_semaphore);
    }
}
```

オシロスコープで：
- パルス幅 = 処理時間
- パルス間隔 = タスク周期
- パルス幅のばらつき = ジッター

---

## マルチレート制御への対応

異なる周波数で動作するセンサと制御を統合する場合：

### 1. 本プロジェクトの構成

| コンポーネント | 周波数 | 役割 |
|---------------|--------|------|
| IMU | 400Hz | 姿勢推定の基盤 |
| ESKF predict | 400Hz | 状態予測 |
| ControlTask | 400Hz | 制御出力 |
| OpticalFlow | 100Hz | 速度観測 |
| Magnetometer | 100Hz | Yaw観測 |
| Barometer | 50Hz | 高度観測 |
| ToF | 30Hz | 高度観測 |

### 2. 低レートセンサの統合方法

```cpp
// センサタスク: data_readyフラグを立てる
static void BaroTask(void* pvParameters)
{
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));  // 50Hz

        g_baro.read();
        g_baro_data_cache = baro_altitude;
        g_baro_data_ready = true;  // フラグを立てるだけ
    }
}

// IMUTask: フラグを確認してESKF更新
static void IMUTask(void* pvParameters)
{
    while (true) {
        // 400Hz周期

        // IMU読み取りとESKF predict (400Hz)
        g_imu.read();
        g_eskf.predict(imu_data, dt);

        // Baro更新 (50Hz - フラグが立っている場合のみ)
        if (g_baro_data_ready) {
            g_baro_data_ready = false;
            g_eskf.updateBaro(g_baro_data_cache);
        }

        // Flow更新 (100Hz - フラグが立っている場合のみ)
        if (g_flow_data_ready) {
            g_flow_data_ready = false;
            g_eskf.updateFlow(g_flow_data_cache);
        }
    }
}
```

### 3. 補間と外挿

低レートセンサのデータを高レート制御で使用する場合：

```cpp
// 線形補間の例
float interpolate(float prev_value, float curr_value,
                  uint32_t prev_time, uint32_t curr_time,
                  uint32_t target_time)
{
    float alpha = (float)(target_time - prev_time) / (curr_time - prev_time);
    return prev_value + alpha * (curr_value - prev_value);
}
```

---

## 本プロジェクトでの実装状況

### 現在の対策

| 対策 | 実装状況 | 備考 |
|------|---------|------|
| ESP Timer (400Hz) | ✅ | 正確なタイミング生成 |
| セマフォ同期 | ✅ | IMU→Controlの連携 |
| コア分離 | ✅ | Core1=リアルタイム, Core0=その他 |
| data_readyフラグ | ✅ | センサ→ESKFの非同期統合 |
| メンバ変数による行列管理 | ✅ | スタック節約 |

### 追加推奨事項

| 対策 | 優先度 | 状態 |
|------|--------|------|
| 実行時間計測 | 高 | 未実装 |
| タスク統計CLI | 高 | 未実装 |
| ジッター監視 | 中 | 未実装 |
| GPIO出力デバッグ | 低 | 未実装 |

---

## 参考資料

1. **FreeRTOS公式ドキュメント**
   - [Task Priorities](https://www.freertos.org/RTOS-task-priority.html)
   - [Rate Monotonic Scheduling](https://www.freertos.org/implementation/a00006.html)

2. **ESP-IDF FreeRTOS**
   - [ESP-IDF FreeRTOS (SMP)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/freertos_idf.html)

3. **リアルタイムシステム設計**
   - "Real-Time Systems Design and Analysis" by Phillip A. Laplante
   - "Hard Real-Time Computing Systems" by Giorgio Buttazzo

---

*最終更新: 2025-12-29*
