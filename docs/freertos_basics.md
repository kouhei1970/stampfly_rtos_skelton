# FreeRTOS基礎知識

## 概要

本ドキュメントでは、FreeRTOSを使用する上で必要な基礎知識を解説します。マルチタスク、セマフォ、ミューテックスなどの概念を理解することで、飛行制御システムの設計と実装がより深く理解できるようになります。

---

## 目次

1. [マルチタスクとは](#1-マルチタスクとは)
2. [タスクの状態](#2-タスクの状態)
3. [優先度とスケジューリング](#3-優先度とスケジューリング)
4. [セマフォ](#4-セマフォ)
5. [ミューテックス](#5-ミューテックス)
6. [キュー](#6-キュー)
7. [イベントグループ](#7-イベントグループ)
8. [タイマー](#8-タイマー)
9. [クリティカルセクション](#9-クリティカルセクション)
10. [よくある問題と対策](#10-よくある問題と対策)

---

## 1. マルチタスクとは

### シングルタスク vs マルチタスク

#### シングルタスク（スーパーループ）

```cpp
// Arduino的なアプローチ
void loop() {
    readIMU();        // 2ms
    readBaro();       // 5ms
    readToF();        // 10ms
    calculateControl(); // 1ms
    outputMotors();   // 0.5ms
    // 合計: 18.5ms → 最大54Hz
}
```

**問題点:**
- 全処理が直列実行
- 最も遅い処理がシステム全体の速度を決める
- IMUを400Hzで読みたくても、他の処理が邪魔

#### マルチタスク（RTOS）

```cpp
// 独立したタスクとして実行
void IMUTask()     { while(1) { readIMU(); delay(2.5ms); }}      // 400Hz
void BaroTask()    { while(1) { readBaro(); delay(20ms); }}      // 50Hz
void ToFTask()     { while(1) { readToF(); delay(33ms); }}       // 30Hz
void ControlTask() { while(1) { calculate(); delay(2.5ms); }}    // 400Hz
```

**利点:**
- 各タスクが独立した周期で実行
- 重い処理が他の処理をブロックしない
- 優先度により重要な処理を優先

### タスクとは

タスクは「独立した実行の流れ」です。各タスクは：

- **独自のスタック**: ローカル変数、関数呼び出し履歴
- **独自のコンテキスト**: CPUレジスタの値
- **独自の優先度**: 実行順序の決定に使用

```cpp
// タスクの作成
void MyTask(void* pvParameters)
{
    // 初期化（1回だけ実行）
    initSomething();

    // メインループ（永久に実行）
    while (true) {
        doWork();
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms待機
    }

    // ここには到達しない（タスクは通常終了しない）
}

// タスクの登録
xTaskCreate(
    MyTask,         // タスク関数
    "MyTask",       // 名前（デバッグ用）
    4096,           // スタックサイズ（バイト）
    NULL,           // パラメータ
    10,             // 優先度
    &taskHandle     // ハンドル（オプション）
);
```

### コンテキストスイッチ

OSがタスクを切り替えることを**コンテキストスイッチ**と呼びます。

```
Time →

TaskA: [実行中][保存]          [復元][実行中]
TaskB:              [復元][実行中][保存]
         ↑                    ↑
    コンテキストスイッチ    コンテキストスイッチ

保存: CPUレジスタ、スタックポインタをメモリに保存
復元: メモリからCPUレジスタ、スタックポインタを復元
```

ESP32-S3では、コンテキストスイッチに約1〜2μsかかります。

---

## 2. タスクの状態

タスクは4つの状態を遷移します：

```
                   ┌─────────────┐
        イベント発生 │   Ready     │ スケジューラ選択
        ┌──────────│ （実行可能）  │──────────┐
        │          └─────────────┘          │
        │                                   ▼
┌─────────────┐                      ┌─────────────┐
│  Blocked    │◄─────────────────────│  Running    │
│ (ブロック中)  │    待機開始           │  (実行中)    │
└─────────────┘                      └─────────────┘
        │                                   │
        │          ┌─────────────┐          │
        └─────────►│  Suspended  │◄─────────┘
           suspend │  (停止中)    │  suspend
                   └─────────────┘
```

### 各状態の説明

| 状態 | 説明 | 例 |
|------|------|-----|
| **Running** | 現在CPUで実行中 | `readIMU();` を実行中 |
| **Ready** | 実行可能だがCPU待ち | 優先度が低くて待機 |
| **Blocked** | イベントを待機中 | `vTaskDelay()`, `xSemaphoreTake()` |
| **Suspended** | 明示的に停止 | `vTaskSuspend()` で停止 |

### ブロッキングの種類

```cpp
// 1. 時間によるブロック
vTaskDelay(pdMS_TO_TICKS(100));  // 100ms待機

// 2. セマフォ待ちによるブロック
xSemaphoreTake(sem, portMAX_DELAY);  // 解放されるまで待機

// 3. キュー待ちによるブロック
xQueueReceive(queue, &data, portMAX_DELAY);  // データが来るまで待機

// 4. 通知待ちによるブロック
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 通知が来るまで待機
```

**重要**: ブロック中のタスクはCPUを消費しません。

---

## 3. 優先度とスケジューリング

### 優先度

FreeRTOSでは数値が大きいほど優先度が高いです。

```cpp
// 本プロジェクトの優先度設定
#define PRIORITY_IMU     24  // 最高（状態推定の基盤）
#define PRIORITY_CONTROL 23  // 高（制御出力）
#define PRIORITY_OPTFLOW 20  // 中高（速度観測）
#define PRIORITY_MAG     18  // 中（Yaw観測）
#define PRIORITY_BARO    16  // 中（高度観測）
#define PRIORITY_TOF     14  // 中低（高度観測）
#define PRIORITY_LED      8  // 低（表示）
#define PRIORITY_CLI      5  // 最低（ユーザーインターフェース）
```

### スケジューリングアルゴリズム

FreeRTOSは**プリエンプティブ優先度スケジューリング**を使用：

1. 最も優先度の高い実行可能タスクが実行される
2. 同じ優先度のタスクはラウンドロビン（時分割）
3. 高優先度タスクが実行可能になると、低優先度タスクを中断（プリエンプション）

```
Time →

TaskH (優先度20): [Blocked]    [■■■Ready■■■]    [Blocked]
TaskL (優先度10): [■■■Running■■■][中断][■■Running■■]

                              ↑
                    TaskHが実行可能になった瞬間
                    TaskLはプリエンプトされる
```

### タイムスライス

同じ優先度のタスクが複数ある場合、一定時間（デフォルト1ms）で切り替わります。

```cpp
// sdkconfig で設定
CONFIG_FREERTOS_HZ=1000  // 1ms tick
```

---

## 4. セマフォ

### セマフォとは

セマフォは「カウンタ付きの信号」です。タスク間の同期に使用します。

```
セマフォ = 整数カウンタ + 待ちタスクリスト

Give(): カウンタを+1（上限まで）
Take(): カウンタが0なら待機、0より大きければ-1
```

### バイナリセマフォ

カウンタが0か1のセマフォ。イベント通知に最適。

```cpp
// 作成
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

// タスクA: イベント発生を通知
void TaskA(void* pvParameters)
{
    while (true) {
        // 何かのイベントを検出
        if (eventDetected()) {
            xSemaphoreGive(sem);  // セマフォを「与える」
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// タスクB: イベントを待つ
void TaskB(void* pvParameters)
{
    while (true) {
        // セマフォを「取る」（イベントを待つ）
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
            handleEvent();  // イベント処理
        }
    }
}
```

### 本プロジェクトでの使用例

```cpp
// ESP Timer → IMUTask → ControlTask の連携

// ESP Timer callback (ISR)
void IRAM_ATTR timer_callback(void* arg)
{
    xSemaphoreGiveFromISR(g_imu_semaphore, NULL);  // IMUを起床
}

// IMUTask
void IMUTask(void* pvParameters)
{
    while (true) {
        xSemaphoreTake(g_imu_semaphore, portMAX_DELAY);  // 起床を待つ

        readIMU();
        runESKF();

        xSemaphoreGive(g_control_semaphore);  // Controlを起床
    }
}

// ControlTask
void ControlTask(void* pvParameters)
{
    while (true) {
        xSemaphoreTake(g_control_semaphore, portMAX_DELAY);  // 起床を待つ

        calculateControl();
        outputMotors();
    }
}
```

### カウンティングセマフォ

カウンタが0〜N のセマフォ。リソースプールの管理に使用。

```cpp
// 最大3つの同時アクセスを許可
SemaphoreHandle_t sem = xSemaphoreCreateCounting(3, 3);

void Task(void* pvParameters)
{
    while (true) {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
            // リソースを使用（最大3タスクまで同時実行可能）
            useSharedResource();

            xSemaphoreGive(sem);  // リソースを返却
        }
    }
}
```

---

## 5. ミューテックス

### ミューテックスとは

ミューテックス（Mutual Exclusion）は「相互排他」を実現する仕組みです。共有リソースを一度に1つのタスクだけがアクセスできるようにします。

```
ミューテックス ≒ バイナリセマフォ + 所有権 + 優先度継承
```

### セマフォとミューテックスの違い

| 項目 | セマフォ | ミューテックス |
|------|---------|--------------|
| 用途 | イベント通知 | リソース保護 |
| 所有権 | なし（誰でもGive可能）| あり（Takeした人だけGive可能）|
| 優先度継承 | なし | あり |
| ISRからのGive | 可能 | 不可 |

### 基本的な使い方

```cpp
// 作成
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// 共有データ
float shared_data = 0.0f;

// タスクA
void TaskA(void* pvParameters)
{
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // ロック取得

        // クリティカルセクション開始
        shared_data = calculateNewValue();
        // クリティカルセクション終了

        xSemaphoreGive(mutex);  // ロック解放
    }
}

// タスクB
void TaskB(void* pvParameters)
{
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // ロック取得

        float local_copy = shared_data;  // データを読み取り

        xSemaphoreGive(mutex);  // ロック解放

        processData(local_copy);  // ロック外で処理
    }
}
```

### 優先度継承

ミューテックスは**優先度逆転**を防ぐ機能があります。

```
優先度逆転の問題:

TaskH (優先度20): [Blocked(Mutex)][============待機============]
TaskM (優先度15): [Ready]        [■■■■■■■■Running■■■■■■■■■]
TaskL (優先度10): [Hold Mutex][Preempted by M][■■■Running■■■]

問題: TaskLがMutexを持っているのに、TaskMに邪魔されてTaskHが待ち続ける
```

```
優先度継承による解決:

TaskH (優先度20): [Blocked(Mutex)][■■Running■■]
TaskM (優先度15): [Ready]         [■■■Runnig■■■]
TaskL (優先度10→20): [Hold Mutex][■■■Running■■■][Done][優先度10に戻る]
                     ↑
                TaskHがMutex待ちになった瞬間
                TaskLの優先度が一時的に20に上昇
```

### 本プロジェクトでの使用例

```cpp
// StampFlyState クラスでの使用
class StampFlyState {
private:
    SemaphoreHandle_t mutex_;
    float roll_, pitch_, yaw_;

public:
    void updateAttitude(float roll, float pitch, float yaw)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        roll_ = roll;
        pitch_ = pitch;
        yaw_ = yaw;
        xSemaphoreGive(mutex_);
    }

    void getAttitudeEuler(float& roll, float& pitch, float& yaw) const
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
        xSemaphoreGive(mutex_);
    }
};
```

---

## 6. キュー

### キューとは

キューはタスク間でデータを受け渡すための「待ち行列」です。

```
Producer → [Data1][Data2][Data3] → Consumer
           └────── キュー ──────┘
```

### 基本的な使い方

```cpp
// キューの作成（最大10個のfloat）
QueueHandle_t queue = xQueueCreate(10, sizeof(float));

// 送信側タスク
void ProducerTask(void* pvParameters)
{
    while (true) {
        float data = readSensor();
        xQueueSend(queue, &data, portMAX_DELAY);  // キューに追加
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 受信側タスク
void ConsumerTask(void* pvParameters)
{
    while (true) {
        float data;
        if (xQueueReceive(queue, &data, portMAX_DELAY) == pdTRUE) {
            processData(data);
        }
    }
}
```

### キューの使用例: コマンドパケット

```cpp
// コマンド構造体
struct Command {
    uint8_t type;
    float value;
};

QueueHandle_t cmd_queue = xQueueCreate(5, sizeof(Command));

// コマンド送信
Command cmd = {CMD_SET_THROTTLE, 0.5f};
xQueueSend(cmd_queue, &cmd, 0);  // ブロックしない

// コマンド受信
void ControlTask(void* pvParameters)
{
    Command cmd;
    while (true) {
        if (xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            executeCommand(cmd);
        }
        // その他の処理...
    }
}
```

---

## 7. イベントグループ

### イベントグループとは

複数のイベントを1つにまとめて待機できる仕組みです。

```cpp
// イベントグループの作成
EventGroupHandle_t events = xEventGroupCreate();

// イベントビットの定義
#define EVENT_IMU_READY   (1 << 0)
#define EVENT_BARO_READY  (1 << 1)
#define EVENT_TOF_READY   (1 << 2)
#define EVENT_ALL_READY   (EVENT_IMU_READY | EVENT_BARO_READY | EVENT_TOF_READY)

// 各タスクがイベントをセット
void IMUTask(void* pvParameters)
{
    while (true) {
        readIMU();
        xEventGroupSetBits(events, EVENT_IMU_READY);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// すべてのイベントを待つタスク
void FusionTask(void* pvParameters)
{
    while (true) {
        // すべてのセンサが準備完了するまで待機
        EventBits_t bits = xEventGroupWaitBits(
            events,
            EVENT_ALL_READY,    // 待つビット
            pdTRUE,             // 自動クリア
            pdTRUE,             // すべてのビットを待つ（AND）
            portMAX_DELAY
        );

        fuseSensorData();
    }
}
```

---

## 8. タイマー

### ソフトウェアタイマー

一定時間後または周期的にコールバックを実行します。

```cpp
// タイマーの作成
TimerHandle_t timer = xTimerCreate(
    "MyTimer",              // 名前
    pdMS_TO_TICKS(100),     // 周期（100ms）
    pdTRUE,                 // 自動リロード（周期実行）
    NULL,                   // パラメータ
    timerCallback           // コールバック関数
);

// コールバック関数
void timerCallback(TimerHandle_t xTimer)
{
    // 100msごとに実行される
    toggleLED();
}

// タイマー開始
xTimerStart(timer, 0);
```

### ESP Timer（ハードウェアタイマー）

より高精度なタイミングが必要な場合：

```cpp
// ESP Timer の作成
esp_timer_create_args_t timer_args = {
    .callback = timer_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "imu_timer"
};

esp_timer_handle_t timer;
esp_timer_create(&timer_args, &timer);

// 2.5ms (400Hz) 周期で開始
esp_timer_start_periodic(timer, 2500);  // マイクロ秒単位

// コールバック（ISRコンテキストで実行）
void IRAM_ATTR timer_callback(void* arg)
{
    xSemaphoreGiveFromISR(g_imu_semaphore, NULL);
}
```

---

## 9. クリティカルセクション

### クリティカルセクションとは

割り込みやタスク切り替えを一時的に禁止する区間です。

```cpp
// 方法1: タスクレベル（プリエンプション禁止）
taskENTER_CRITICAL(&spinlock);
// この間、他のタスクに切り替わらない
critical_operation();
taskEXIT_CRITICAL(&spinlock);

// 方法2: 割り込みレベル（割り込み禁止）
portDISABLE_INTERRUPTS();
// この間、割り込みも発生しない
very_critical_operation();
portENABLE_INTERRUPTS();
```

### 使用上の注意

```cpp
// 悪い例: 長いクリティカルセクション
taskENTER_CRITICAL(&spinlock);
readSensorData();     // I/O待ち → 長時間ブロック
processData();        // 計算 → 時間がかかる
writeOutput();
taskEXIT_CRITICAL(&spinlock);

// 良い例: 最小限のクリティカルセクション
readSensorData();     // クリティカルセクション外
processData();

taskENTER_CRITICAL(&spinlock);
shared_variable = result;  // データコピーのみ
taskEXIT_CRITICAL(&spinlock);

writeOutput();
```

**原則**: クリティカルセクションは可能な限り短くする。

---

## 10. よくある問題と対策

### デッドロック

2つ以上のタスクが互いにロックを待って永久に進まない状態。

```cpp
// デッドロックの例
// TaskA: mutex1 → mutex2 の順でロック
// TaskB: mutex2 → mutex1 の順でロック

void TaskA() {
    xSemaphoreTake(mutex1, portMAX_DELAY);
    xSemaphoreTake(mutex2, portMAX_DELAY);  // TaskBがmutex2を持っていると待機
    // ...
}

void TaskB() {
    xSemaphoreTake(mutex2, portMAX_DELAY);
    xSemaphoreTake(mutex1, portMAX_DELAY);  // TaskAがmutex1を持っていると待機
    // ...
}
// → 両方が永久に待機
```

**対策**:
- ロックの取得順序を統一
- タイムアウト付きでロックを取得
- ネストしたロックを避ける

### スタックオーバーフロー

タスクのスタックが不足して他のメモリを破壊。

```cpp
// スタックオーバーフローの原因
void Task(void* pvParameters)
{
    float large_array[1000];  // 4000バイト！
    // ...
}

// スタックサイズが4096バイトだと危険
xTaskCreate(Task, "Task", 4096, NULL, 10, NULL);
```

**対策**:
- 大きな配列は静的変数かヒープに
- スタック使用量の監視

```cpp
// スタック残量の確認
UBaseType_t remaining = uxTaskGetStackHighWaterMark(NULL);
ESP_LOGI(TAG, "Stack remaining: %u bytes", remaining * sizeof(StackType_t));
```

### 優先度逆転

低優先度タスクがリソースを保持し、高優先度タスクが待たされる。

**対策**:
- ミューテックス（優先度継承あり）を使用
- クリティカルセクションは短く
- ロックを持ったまま長い処理をしない

### レースコンディション

複数タスクが同時にデータにアクセスして不整合が発生。

```cpp
// 問題のあるコード
float position = 0.0f;

void UpdateTask() {
    position = calculateNewPosition();  // 書き込み
}

void ReadTask() {
    float p = position;  // 読み込み（書き込み途中かも）
}
```

**対策**:
- ミューテックスで保護
- アトミック操作を使用
- データのコピーを作成

---

## まとめ

| 概念 | 用途 | 注意点 |
|------|------|--------|
| タスク | 独立した処理の流れ | スタックサイズに注意 |
| セマフォ | イベント通知、同期 | ISRからはGiveFromISRを使用 |
| ミューテックス | 共有リソース保護 | 優先度継承あり、短時間で解放 |
| キュー | データ受け渡し | サイズとブロック時間を考慮 |
| クリティカルセクション | 割り込み禁止 | 最小限の時間で |

### 本プロジェクトでの適用

```
ESP Timer (2.5ms周期)
    ↓ セマフォ Give
IMUTask (優先度24)
    ├── センサ読み取り
    ├── ESKF更新
    └── セマフォ Give
        ↓
ControlTask (優先度23)
    ├── 制御計算
    └── モーター出力

StampFlyState
    └── ミューテックスで保護された共有データ
```

---

## 参考資料

- [FreeRTOS公式ドキュメント](https://www.freertos.org/Documentation/RTOS_book.html)
- [ESP-IDF FreeRTOS API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/freertos.html)
- [Mastering the FreeRTOS Real Time Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel-Book) (Github)

---

*最終更新: 2025-12-29*
