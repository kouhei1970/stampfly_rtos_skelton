# stampfly_logger コンポーネント設計書

## 1. 概要

### 1.1 目的
バイナリログ出力機能をCLIコンポーネントから分離し、独立したロガーコンポーネントとして実装する。
これにより、任意の周波数（100Hz〜400Hz）でのログ出力、ESKFタスクとのノンブロッキング連携を実現する。

### 1.2 現状の問題点
- CLIコンポーネントが「コマンド処理」と「ログ出力」の2つの責務を持つ
- ログのタイミング制御がmain.cppに散在（FreeRTOSティック精度に制限）
- ESKFタスク（400Hz）とログ出力の連携が困難
- USB書き込みのブロッキングが推定処理に影響する可能性

### 1.3 目標
- 単一責務の原則に基づくコンポーネント分離
- ESP Timerによる高精度タイミング（μs単位）
- リングバッファ/キューによるノンブロッキングログ
- 任意周波数の設定（100Hz, 200Hz, 400Hz等）

---

## 2. アーキテクチャ

### 2.1 システム構成図

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.cpp                                │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │ ESKF Task    │    │ Sensor Tasks │    │  CLI Task    │      │
│  │   (400Hz)    │    │              │    │              │      │
│  └──────┬───────┘    └──────────────┘    └──────┬───────┘      │
│         │                                        │              │
│         │ pushData()                  start()/stop() commands   │
│         ▼                                        ▼              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              stampfly_logger (新規)                       │  │
│  │  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐     │  │
│  │  │ ESP Timer   │──▶│ Log Queue   │──▶│ Writer Task │     │  │
│  │  │ (任意Hz)    │   │(Ring Buffer)│   │   (USB出力) │     │  │
│  │  └─────────────┘   └─────────────┘   └─────────────┘     │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 データフロー

1. **ESKFタスク（400Hz）**: センサデータと推定結果を`Logger::pushData()`でキューに投入
2. **ESP Timer（設定周波数）**: タイマー割り込みでセマフォを通知
3. **Writer Task**: セマフォ待機→キューからデータ取得→USB送信

---

## 3. ファイル構成

```
components/stampfly_logger/
├── include/
│   └── logger.hpp          # Logger クラス定義、LogPacket構造体
├── logger.cpp              # Logger 実装
└── CMakeLists.txt          # ビルド設定

components/stampfly_cli/
├── cli.cpp                 # ログ関連コード削除、Logger呼び出しに変更
└── include/cli.hpp         # BinaryLogPacketV2 → logger.hppへ移動
```

---

## 4. API設計

### 4.1 Logger クラス

```cpp
namespace stampfly {

/**
 * @brief バイナリログパケット構造体（128バイト）
 *
 * CLIから移動、変更なし
 */
#pragma pack(push, 1)
struct LogPacket {
    uint8_t header[2];      // 0xAA, 0x56
    uint32_t timestamp_ms;
    // ... (既存のBinaryLogPacketV2と同じ)
};
#pragma pack(pop)

/**
 * @brief 高精度ロガークラス
 */
class Logger {
public:
    /**
     * @brief ロガー初期化
     * @param frequency_hz ログ出力周波数（100, 200, 400等）
     * @param queue_size キューサイズ（デフォルト: 16パケット）
     * @return ESP_OK on success
     */
    esp_err_t init(uint32_t frequency_hz, size_t queue_size = 16);

    /**
     * @brief ログ出力開始
     */
    void start();

    /**
     * @brief ログ出力停止
     */
    void stop();

    /**
     * @brief ログデータをキューに投入（ノンブロッキング）
     * @param packet ログパケット
     * @return true: 成功, false: キュー満杯
     *
     * ESKFタスクから呼び出される。ブロックしない。
     */
    bool pushData(const LogPacket& packet);

    /**
     * @brief 現在のログ周波数を取得
     */
    uint32_t getFrequency() const { return frequency_hz_; }

    /**
     * @brief ログ周波数を変更
     * @param frequency_hz 新しい周波数
     */
    void setFrequency(uint32_t frequency_hz);

    /**
     * @brief ログカウンタを取得
     */
    uint32_t getLogCount() const { return log_count_; }

    /**
     * @brief ログカウンタをリセット
     */
    void resetLogCount() { log_count_ = 0; }

    /**
     * @brief ログ出力中か確認
     */
    bool isRunning() const { return running_; }

    /**
     * @brief 開始時コールバック設定
     */
    void setStartCallback(void (*callback)()) { start_callback_ = callback; }

private:
    static void timerCallback(void* arg);
    static void writerTask(void* arg);

    uint32_t frequency_hz_ = 100;
    bool initialized_ = false;
    bool running_ = false;
    uint32_t log_count_ = 0;

    esp_timer_handle_t timer_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t write_sem_ = nullptr;
    TaskHandle_t writer_task_ = nullptr;

    void (*start_callback_)() = nullptr;
};

}  // namespace stampfly
```

### 4.2 使用例

```cpp
// main.cpp
#include "logger.hpp"

stampfly::Logger g_logger;

void app_main() {
    // 400Hzでロガー初期化
    g_logger.init(400);

    // 開始コールバック設定（mag_ref設定等）
    g_logger.setStartCallback([]() {
        g_eskf.setMagReference(...);
    });
}

// CLIコマンドから
void cmd_binlog(int argc, char** argv, void* context) {
    if (strcmp(argv[1], "on") == 0) {
        g_logger.start();
    } else if (strcmp(argv[1], "off") == 0) {
        g_logger.stop();
    } else if (strcmp(argv[1], "freq") == 0) {
        uint32_t freq = atoi(argv[2]);
        g_logger.setFrequency(freq);  // 100, 200, 400Hz
    }
}

// ESKFタスク内（400Hz）
void eskf_timer_callback(...) {
    // ... ESKF処理 ...

    if (g_logger.isRunning()) {
        LogPacket pkt = buildLogPacket();  // パケット構築
        g_logger.pushData(pkt);            // ノンブロッキング
    }
}
```

---

## 5. 実装詳細

### 5.1 ESP Timer設定

```cpp
esp_err_t Logger::init(uint32_t frequency_hz, size_t queue_size) {
    frequency_hz_ = frequency_hz;

    // キュー作成
    queue_ = xQueueCreate(queue_size, sizeof(LogPacket));

    // セマフォ作成
    write_sem_ = xSemaphoreCreateBinary();

    // Writerタスク作成
    xTaskCreatePinnedToCore(
        writerTask, "log_writer",
        4096, this,
        5,  // 優先度（ESKFより低く）
        &writer_task_,
        0   // Core 0（ESKFはCore 1）
    );

    // ESP Timer作成
    esp_timer_create_args_t timer_args = {
        .callback = timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "logger_timer"
    };
    esp_timer_create(&timer_args, &timer_);

    initialized_ = true;
    return ESP_OK;
}
```

### 5.2 タイマーコールバック

```cpp
void Logger::timerCallback(void* arg) {
    Logger* self = static_cast<Logger*>(arg);
    // ISRコンテキストなのでセマフォを通知するだけ
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(self->write_sem_, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### 5.3 Writerタスク

```cpp
void Logger::writerTask(void* arg) {
    Logger* self = static_cast<Logger*>(arg);
    LogPacket pkt;

    while (true) {
        // タイマーからの通知を待つ
        if (xSemaphoreTake(self->write_sem_, portMAX_DELAY) == pdTRUE) {
            // キューから最新データを取得
            if (xQueueReceive(self->queue_, &pkt, 0) == pdTRUE) {
                // USB出力（ブロッキング可）
                fwrite(&pkt, sizeof(pkt), 1, stdout);
                fflush(stdout);
                self->log_count_++;
            }
        }
    }
}
```

### 5.4 周波数設定

```cpp
void Logger::start() {
    if (!initialized_ || running_) return;

    if (start_callback_) start_callback_();

    // μs単位で周期設定
    uint64_t period_us = 1000000 / frequency_hz_;
    esp_timer_start_periodic(timer_, period_us);

    running_ = true;
}

void Logger::setFrequency(uint32_t frequency_hz) {
    bool was_running = running_;
    if (was_running) stop();

    frequency_hz_ = frequency_hz;

    if (was_running) start();
}
```

---

## 6. CLI変更点

### 6.1 削除するコード（cli.cpp）
- `outputBinaryLogV2()` 関数全体
- `BinaryLogPacketV2` 構造体（logger.hppへ移動）
- `binlog_v2_enabled_`, `binlog_counter_` 変数

### 6.2 変更するコード（cli.cpp）

```cpp
// cmd_binlog() 変更
static void cmd_binlog(int argc, char** argv, void* context) {
    extern Logger g_logger;  // main.cppで定義

    if (argc < 2) {
        cli->print("Usage: binlog [on|off|freq <hz>]\r\n");
        cli->print("Current: %s, freq=%luHz, count=%lu\r\n",
                   g_logger.isRunning() ? "on" : "off",
                   g_logger.getFrequency(),
                   g_logger.getLogCount());
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        g_logger.resetLogCount();
        g_logger.start();
        cli->print("Logging started at %luHz\r\n", g_logger.getFrequency());
    } else if (strcmp(argv[1], "off") == 0) {
        g_logger.stop();
        cli->print("Logging stopped, count=%lu\r\n", g_logger.getLogCount());
    } else if (strcmp(argv[1], "freq") == 0 && argc >= 3) {
        uint32_t freq = atoi(argv[2]);
        if (freq >= 10 && freq <= 1000) {
            g_logger.setFrequency(freq);
            cli->print("Frequency set to %luHz\r\n", freq);
        } else {
            cli->print("Invalid frequency (10-1000Hz)\r\n");
        }
    }
}
```

---

## 7. main.cpp変更点

### 7.1 削除するコード
```cpp
// 削除: CLIタスク内のログ出力
// if (g_cli.isBinlogV2Enabled() && (now - last_binlog) >= binlog_period) {
//     g_cli.outputBinaryLogV2();
//     last_binlog = now;
// }
```

### 7.2 追加するコード
```cpp
#include "logger.hpp"

stampfly::Logger g_logger;

void app_main() {
    // ... 既存の初期化 ...

    // ロガー初期化（デフォルト400Hz）
    g_logger.init(400);

    // 開始コールバック
    g_logger.setStartCallback([]() {
        // ESKFのmag_refを現在の値に設定など
    });
}

// ESKFタスク内（eskf_timer_callback）
void eskf_timer_callback(...) {
    // ... ESKF処理 ...

    // ログ出力（ESKFタスクから直接pushData）
    if (g_logger.isRunning()) {
        LogPacket pkt;
        // パケット構築...
        g_logger.pushData(pkt);
    }
}
```

---

## 8. 実装スケジュール

| ステップ | 内容 | 所要時間 |
|---------|------|----------|
| 1 | stampfly_logger ディレクトリ作成 | 5分 |
| 2 | logger.hpp 作成（API定義） | 15分 |
| 3 | logger.cpp 実装 | 30分 |
| 4 | CMakeLists.txt 作成 | 5分 |
| 5 | cli.cpp 変更（ログコード削除、Logger呼び出し） | 20分 |
| 6 | main.cpp 変更（Logger初期化、pushData追加） | 20分 |
| 7 | ビルド・デバッグ | 30分 |
| **合計** | | **約2時間** |

---

## 9. テスト計画

### 9.1 単体テスト
- [ ] Logger初期化成功
- [ ] 周波数設定（100, 200, 400Hz）
- [ ] start/stop動作
- [ ] pushDataのノンブロッキング確認

### 9.2 統合テスト
- [ ] CLIコマンド `binlog on/off/freq` 動作
- [ ] 各周波数でのログ出力確認
- [ ] ESKFの処理遅延がないこと確認（400Hz維持）
- [ ] PC側replayツールでの読み取り確認

### 9.3 性能テスト
- [ ] 400Hz × 128B = 51.2KB/s の連続出力
- [ ] キューオーバーフロー発生しないこと
- [ ] CPU使用率の確認

---

## 10. 将来の拡張

- SDカードへのログ保存
- WiFi経由のリモートログ
- ログレベル（debug/info/error）の実装
- 複数ログストリームの同時出力
