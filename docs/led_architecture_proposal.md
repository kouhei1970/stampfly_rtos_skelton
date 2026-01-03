# LED アーキテクチャ設計提案

## 0. ハードウェア構成

StampFlyには**3つのWS2812 LED**が搭載されている：

| LED | GPIO | 数量 | 位置 | 用途（提案） |
|-----|------|------|------|-------------|
| MCU LED | GPIO21 | 1個 | M5Stamp S3上 | システム状態（起動/エラー） |
| Body LED 0 | GPIO39 | 1個 | ドローン本体（前） | 飛行状態 |
| Body LED 1 | GPIO39 | 1個 | ドローン本体（後） | センサー/バッテリー |

**注意**: Body LED 0/1はGPIO39で直列接続（WS2812デイジーチェーン）

### 各LEDの役割分担（提案）

```
┌─────────────────────────────────────────────────────────────┐
│  MCU LED (GPIO21)     │ システム状態                        │
│  ・起動中: 白 Breathe  │ ・エラー: 赤 Blink                  │
│  ・Ready: 緑 Solid     │ ・ペアリング: 青 Blink              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Body LED 0 (前)      │ 飛行状態                            │
│  ・IDLE: 緑 Solid      │ ・ARMED: 緑 Blink                   │
│  ・FLYING: 黄 Solid    │ ・LANDING: 緑 Fast Blink            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Body LED 1 (後)      │ センサー/バッテリー                 │
│  ・正常: 緑 Solid      │ ・低電圧: シアン Blink              │
│  ・センサー異常: 赤    │ ・デバッグ: 黄                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 1. 現状の問題点

### 1.1 責任の分散と混在

現在、LED制御が複数箇所に分散しており、一貫性がない：

| 場所 | 役割 | 問題 |
|------|------|------|
| `led.cpp` | ハードウェア制御 + 状態ヘルパー | showIdle()等の状態表示ロジックが混在 |
| `led_task.cpp` | 状態監視 + update()呼び出し | 特殊ケース（INIT状態、yaw_alert）の例外処理 |
| `main.cpp` | 起動シーケンスのLED制御 | 手動update()呼び出しが必要 |
| `control_task.cpp` | デバッグ表示（yaw_alert） | グローバル変数で制御 |
| `power_task.cpp` | 低電圧警告 | 直接setPattern()呼び出し |

### 1.2 場当たり的な対処

```cpp
// led_task.cpp - INIT状態の例外処理
case stampfly::FlightState::INIT:
    // INIT状態では手動で設定したパターンを尊重（上書きしない）
    break;

// main.cpp - 手動LED更新
for (int j = 0; j < 30; j++) {
    g_led.update();
    vTaskDelay(pdMS_TO_TICKS(33));
}

// control_task.cpp - グローバル変数による制御
volatile int g_yaw_alert_counter = 0;
if (g_yaw_alert_counter > 0) {
    // led_taskで特別扱い
}
```

### 1.3 優先度の概念がない

複数の制御元が同時にLEDを制御しようとした場合、「最後に呼んだ者勝ち」になる：

- 低電圧警告 vs 飛行状態
- デバッグ表示 vs 通常表示
- 起動シーケンス vs LEDタスク

### 1.4 起動時の問題

LEDタスクが他のタスクと同時に起動するため、起動シーケンス中は手動でupdate()を呼ぶ必要がある。

---

## 2. 提案アーキテクチャ

### 2.1 設計原則

1. **優先度ベースの表示システム** - 高優先度の表示要求が低優先度を上書き
2. **LEDタスクの早期起動** - アクチュエータ初期化直後に起動
3. **単一の制御点** - すべてのLED制御要求は一箇所を通る
4. **レイヤー分離** - ハードウェア制御と表示ポリシーを分離

### 2.2 レイヤー構成

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│  main.cpp, control_task, power_task, etc.                   │
│  → LEDManager::request(source, pattern, color, timeout)     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   sf_svc_led (Service Layer)                │
│  LEDManager - 優先度管理、タイムアウト、状態遷移            │
│  ・優先度に基づく表示決定                                   │
│  ・一時的表示のタイムアウト管理                             │
│  ・フライト状態に応じたデフォルト表示                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   sf_hal_led (HAL Layer)                    │
│  LED - ハードウェア制御のみ                                 │
│  ・RMT/WS2812制御                                          │
│  ・パターンアニメーション                                   │
│  ・輝度制御                                                 │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 優先度定義

```cpp
enum class LEDPriority : uint8_t {
    BOOT = 0,           // 最高: 起動シーケンス
    CRITICAL_ERROR = 1, // 致命的エラー（センサー故障等）
    LOW_BATTERY = 2,    // 低電圧警告
    DEBUG_ALERT = 3,    // デバッグ表示（yaw_alert等）
    PAIRING = 4,        // ペアリングモード
    FLIGHT_STATE = 5,   // 通常の飛行状態表示
    DEFAULT = 6,        // 最低: デフォルト表示
};
```

### 2.4 LEDインデックス定義

```cpp
enum class LEDIndex : uint8_t {
    MCU = 0,      // M5Stamp S3内蔵 (GPIO21)
    BODY_FRONT,   // ドローン前部 (GPIO39, index 0)
    BODY_REAR,    // ドローン後部 (GPIO39, index 1)
    ALL,          // 全LED一括
    NUM_LEDS = 3
};

// 各LEDの担当チャンネル
enum class LEDChannel : uint8_t {
    SYSTEM,       // MCU LED: システム状態（起動/エラー/ペアリング）
    FLIGHT,       // BODY_FRONT: 飛行状態
    STATUS,       // BODY_REAR: センサー/バッテリー状態
};
```

### 2.5 表示要求API

```cpp
class LEDManager {
public:
    // シングルトン
    static LEDManager& getInstance();

    // 初期化（2つのLEDストリップを管理）
    esp_err_t init();

    // === チャンネルベースAPI（推奨）===
    // 各チャンネルに表示を要求（優先度で自動管理）
    void requestChannel(LEDChannel channel, LEDPriority priority,
                        LED::Pattern pattern, uint32_t color,
                        uint32_t timeout_ms = 0);
    void releaseChannel(LEDChannel channel, LEDPriority priority);

    // === 直接制御API（デバッグ用）===
    // 特定のLEDを直接制御（優先度システムをバイパス）
    void setDirect(LEDIndex led, LED::Pattern pattern, uint32_t color);

    // === イベント通知API ===
    // フライト状態変更（内部でFLIGHTチャンネルを更新）
    void onFlightStateChanged(FlightState state);

    // バッテリー状態変更（内部でSTATUSチャンネルを更新）
    void onBatteryStateChanged(float voltage, bool low_battery);

    // センサー状態変更（内部でSTATUSチャンネルを更新）
    void onSensorHealthChanged(bool all_healthy);

    // 更新（LEDタスクから呼び出し）
    void update();

private:
    // 各チャンネルの表示要求キュー
    struct ChannelState {
        struct Request {
            bool active;
            LEDPriority priority;
            LED::Pattern pattern;
            uint32_t color;
            uint32_t expire_time_ms;
        };
        Request requests[8];  // 優先度数分
        LEDIndex target_led;
    };

    ChannelState channels_[3];  // SYSTEM, FLIGHT, STATUS

    // HAL（2つのLEDストリップ）
    LEDStrip mcu_led_;   // GPIO21, 1個
    LEDStrip body_led_;  // GPIO39, 2個
};
```

### 2.6 使用例

```cpp
auto& led = LEDManager::getInstance();

// =====================================================
// 起動シーケンス（SYSTEM チャンネル = MCU LED）
// =====================================================

// Phase 1: Place on ground（白 Breathe）
led.requestChannel(LEDChannel::SYSTEM, LEDPriority::BOOT,
                   LED::Pattern::BREATHE, 0xFFFFFF);

// Phase 2: Sensor init（青 Breathe）
led.requestChannel(LEDChannel::SYSTEM, LEDPriority::BOOT,
                   LED::Pattern::BREATHE, 0x0000FF);

// Phase 3: Stabilization（マゼンタ Blink）
led.requestChannel(LEDChannel::SYSTEM, LEDPriority::BOOT,
                   LED::Pattern::BLINK_SLOW, 0xFF00FF);

// Phase 4: Ready（緑 Solid → BOOT解除で自動表示）
led.releaseChannel(LEDChannel::SYSTEM, LEDPriority::BOOT);

// =====================================================
// 飛行状態（FLIGHT チャンネル = BODY_FRONT LED）
// =====================================================

// FlightState変更時に自動更新（onFlightStateChangedから呼ばれる）
led.onFlightStateChanged(FlightState::ARMED);
// → BODY_FRONT が緑点滅になる

// =====================================================
// センサー/バッテリー（STATUS チャンネル = BODY_REAR LED）
// =====================================================

// 低電圧警告
led.onBatteryStateChanged(3.3f, true);
// → BODY_REAR がシアン点滅になる

// デバッグ表示（1秒後に自動解除）
led.requestChannel(LEDChannel::STATUS, LEDPriority::DEBUG_ALERT,
                   LED::Pattern::BLINK_FAST, 0xFFFF00, 1000);

// =====================================================
// 全LED一括制御（緊急時）
// =====================================================

// 致命的エラー（全LED赤点滅）
led.setDirect(LEDIndex::ALL, LED::Pattern::BLINK_FAST, 0xFF0000);
```

---

## 3. LEDタスクの早期起動

### 3.1 現状の起動順序

```
app_main()
├── Phase 1: Place on ground (3秒) ← 手動update()が必要
├── Phase 2: Sensor init           ← 手動update()が必要
├── startTasks() ← ここでLEDタスク起動
├── Phase 3: Stabilization         ← LEDタスクが動いているが例外処理
└── Phase 4: Ready
```

### 3.2 提案する起動順序

```
app_main()
├── NVS初期化
├── LEDManager初期化（GPIO21 + GPIO39）
├── LEDタスク起動（最優先で起動）  ← 他の全タスクより前
│   └── 以降、手動update()不要
│
├── Phase 1: Place on ground (MCU=白 Breathe)
│   └── requestChannel(SYSTEM, BOOT, BREATHE, WHITE)
│
├── Phase 2: Sensor init (MCU=青 Breathe)
│   └── requestChannel(SYSTEM, BOOT, BREATHE, BLUE)
│
├── 他のタスク起動
│
├── Phase 3: Stabilization (MCU=マゼンタ Blink)
│   └── requestChannel(SYSTEM, BOOT, BLINK_SLOW, MAGENTA)
│
├── Phase 4: Ready
│   ├── releaseChannel(SYSTEM, BOOT)
│   │   → MCU LED: 自動的に緑 Solidに
│   ├── BODY_FRONT: IDLE状態 → 緑 Solid
│   └── BODY_REAR: センサー正常 → 緑 Solid
│
└── メインループ
    └── 各タスクからonXxxChanged()でLED自動更新
```

### 3.3 利点

- 手動update()呼び出しが不要
- 起動シーケンス中も滑らかなアニメーション
- タスク間の調整不要（優先度で自動解決）

---

## 4. 実装計画

### Phase 1: sf_svc_led コンポーネント作成

1. `components/sf_svc_led/` 作成
2. `LEDManager` クラス実装
3. 優先度管理、タイムアウト管理

### Phase 2: sf_hal_led の簡素化

1. 状態ヘルパー関数（showIdle等）を削除
2. 純粋なハードウェア制御のみに

### Phase 3: LEDタスクのリファクタリング

1. `sf_task_led/` に移動
2. `LEDManager::update()` 呼び出しのみに簡素化
3. 早期起動の実装

### Phase 4: 各所の修正

1. main.cpp: `LEDManager::request()` に置き換え
2. control_task.cpp: デバッグ表示を `LEDManager::request()` に
3. power_task.cpp: 低電圧警告を `LEDManager::request()` に
4. グローバル変数 `g_yaw_alert_counter` 削除

---

## 5. 新旧対応表

| 現在のコード | 新しいコード |
|-------------|-------------|
| `g_led.setPattern(BREATHE, WHITE)` | `LEDManager::request(BOOT, BREATHE, WHITE)` |
| `g_led.showIdle()` | `LEDManager::onFlightStateChanged(IDLE)` |
| `g_led.update()` (手動) | 不要（タスクが自動で呼び出し） |
| `g_yaw_alert_counter = 400` | `LEDManager::request(DEBUG_ALERT, ..., 1000)` |
| INIT状態の例外処理 | 不要（優先度で自動解決） |

---

## 6. 代替案の検討

### 案A: 現状維持 + 最小限の修正

- 利点: 変更量が少ない
- 欠点: 技術的負債が残る

### 案B: 提案アーキテクチャ（推奨）

- 利点: 拡張性、保守性、明確な設計
- 欠点: 変更量が多い

### 案C: イベントドリブン方式

- LED表示要求をFreeRTOSキューで管理
- 利点: 完全な非同期化
- 欠点: 複雑すぎる、オーバーヘッド

**推奨**: 案B（提案アーキテクチャ）

---

## 7. ファイル構成（最終形）

```
components/
├── sf_hal_led/                    # HAL層（ハードウェア制御のみ）
│   ├── include/
│   │   └── led_strip.hpp          # 単一GPIOのLEDストリップ制御
│   └── led_strip.cpp
│
├── sf_svc_led/                    # サービス層（新規）
│   ├── include/
│   │   ├── led_manager.hpp        # LEDManager クラス
│   │   ├── led_types.hpp          # LEDIndex, LEDChannel, LEDPriority
│   │   └── led_patterns.hpp       # パターン定義
│   └── led_manager.cpp
│
main/
├── config.hpp                     # GPIO定義追加
│   # GPIO_LED_MCU = 21
│   # GPIO_LED_BODY = 39
│   # NUM_LEDS_MCU = 1
│   # NUM_LEDS_BODY = 2
│
└── tasks/
    └── led_task.cpp               # 簡素化（update()呼び出しのみ）
```

### 7.1 config.hpp への追加

```cpp
namespace led {
// M5Stamp S3 内蔵LED
inline constexpr int GPIO_MCU = 21;
inline constexpr int NUM_LEDS_MCU = 1;

// StampFly ボード上LED（デイジーチェーン）
inline constexpr int GPIO_BODY = 39;
inline constexpr int NUM_LEDS_BODY = 2;

// LED インデックス
inline constexpr int IDX_BODY_FRONT = 0;
inline constexpr int IDX_BODY_REAR = 1;
}  // namespace led
```

---

## 8. 結論

現在のLED制御は場当たり的な修正が積み重なり、保守性が低下しています。
また、3つのLEDが存在するにも関わらず1つしか活用していません。

新アーキテクチャを導入することで：

1. **3つのLEDの有効活用**
   - MCU LED (GPIO21): システム状態（起動/エラー）
   - BODY_FRONT (GPIO39-0): 飛行状態
   - BODY_REAR (GPIO39-1): センサー/バッテリー

2. **チャンネルベースの制御**
   - 各LEDが担当する情報カテゴリを明確化
   - 複数情報を同時に表示可能

3. **優先度ベースの表示管理**
   - 各チャンネル内で優先度により自動解決
   - タイムアウト付き一時表示

4. **早期起動**
   - LEDタスクを最初に起動
   - 手動update()呼び出しが不要

5. **イベント駆動**
   - onFlightStateChanged(), onBatteryStateChanged()
   - 各タスクからの通知で自動更新

これにより、パイロットはLEDを見るだけで「システム状態」「飛行状態」「センサー/バッテリー状態」を一目で把握できるようになります。
