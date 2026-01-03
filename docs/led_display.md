# LED表示仕様

StampFlyの3つのLEDによる状態表示システム。

## ハードウェア構成

| LED | GPIO | 位置 | チャンネル |
|-----|------|------|-----------|
| MCU LED | 21 | M5Stamp S3基板上 | SYSTEM |
| BODY_TOP | 39 (index 0) | ドローン上面/表 | FLIGHT |
| BODY_BOTTOM | 39 (index 1) | ドローン下面/裏 | STATUS |

## 優先度システム

複数の表示要求がある場合、優先度の高いものが表示される。

| 優先度 | 用途 | 説明 |
|--------|------|------|
| 0: BOOT | 起動シーケンス | 最高優先度、起動中のみ |
| 1: CRITICAL_ERROR | 致命的エラー | センサー異常等 |
| 2: LOW_BATTERY | 低バッテリー | 電圧警告 |
| 3: DEBUG_ALERT | デバッグ通知 | ヨー急変検出等 |
| 4: PAIRING | ペアリング中 | コントローラ接続待ち |
| 5: FLIGHT_STATE | 飛行状態 | 通常の状態表示 |
| 6: DEFAULT | デフォルト | 待機状態 |

---

## SYSTEM チャンネル (MCU LED)

システム全体の状態を表示。

### 起動シーケンス

| フェーズ | 色 | パターン | 意味 |
|---------|-----|---------|------|
| Phase 1 | 白 (0xFFFFFF) | ブリーズ | ハードウェア初期化中 |
| Phase 2 | 青 (0x0000FF) | ブリーズ | センサータスク起動待ち |
| Phase 3 | マゼンタ (0xFF00FF) | 遅い点滅 | センサー安定待ち |
| Phase 4 | 緑 (0x00FF00) | 点灯 | 準備完了 |

### ペアリング

| 状態 | 色 | パターン |
|------|-----|---------|
| ペアリング中 | 青 (0x0000FF) | 高速点滅 |

---

## FLIGHT チャンネル (BODY_TOP)

飛行状態を表示。

| FlightState | 色 | パターン | 説明 |
|-------------|-----|---------|------|
| INIT | 青 | ブリーズ | 初期化中 |
| CALIBRATING | 黄 | 高速点滅 | キャリブレーション中 |
| IDLE | 緑 | 点灯 | 待機中（アーム可能） |
| ARMED | 緑 | 遅い点滅 | アーム済み（離陸準備完了） |
| FLYING | 黄 | 点灯 | 飛行中 |
| LANDING | 緑 | 高速点滅 | 着陸中 |
| ERROR | 赤 | 高速点滅 | エラー発生 |

---

## STATUS チャンネル (BODY_BOTTOM)

センサー・バッテリー等の状態を表示。

| 状態 | 色 | パターン | 優先度 |
|------|-----|---------|--------|
| センサー異常 | 赤 (0xFF0000) | 高速点滅 | CRITICAL_ERROR |
| 低バッテリー | シアン (0x00FFFF) | 遅い点滅 | LOW_BATTERY |
| ヨー指令急変 | 黄 (0xFFFF00) | 高速点滅 | DEBUG_ALERT |
| ヨージャイロ急変 | 黄 (0xFFFF00) | 点灯 | DEBUG_ALERT |
| 正常 | 緑 (0x00FF00) | 点灯 | DEFAULT |

---

## パターン定義

| パターン | 動作 |
|----------|------|
| OFF | 消灯 |
| SOLID | 点灯 |
| BLINK_SLOW | 遅い点滅 (1秒周期) |
| BLINK_FAST | 高速点滅 (200ms周期) |
| BREATHE | ふわっと明滅 (2秒周期) |
| RAINBOW | 虹色サイクル (5秒周期) |

---

## CLIコマンド

```bash
# 現在の明るさを確認
led

# 明るさを変更（0-255、NVSに保存）
led brightness 64
```

---

## API使用例

```cpp
#include "led_manager.hpp"

auto& led_mgr = stampfly::LEDManager::getInstance();

// チャンネルに表示要求（優先度付き）
led_mgr.requestChannel(
    stampfly::LEDChannel::SYSTEM,
    stampfly::LEDPriority::BOOT,
    stampfly::LEDPattern::BREATHE,
    0xFFFFFF  // 白
);

// タイムアウト付き表示要求（1秒後に自動解除）
led_mgr.requestChannel(
    stampfly::LEDChannel::STATUS,
    stampfly::LEDPriority::DEBUG_ALERT,
    stampfly::LEDPattern::BLINK_FAST,
    0xFFFF00,  // 黄
    1000       // 1000ms
);

// 優先度を解放
led_mgr.releaseChannel(
    stampfly::LEDChannel::SYSTEM,
    stampfly::LEDPriority::BOOT
);

// イベント通知（内部で適切な表示を設定）
led_mgr.onFlightStateChanged(stampfly::FlightState::ARMED);
led_mgr.onBatteryStateChanged(voltage, is_low);
led_mgr.onSensorHealthChanged(all_healthy);

// 明るさ変更
led_mgr.setBrightness(64, true);  // NVSに保存
```

---

## 関連ファイル

- `components/sf_svc_led/` - LEDManager実装
- `components/sf_hal_led/` - LED HALドライバ
- `main/tasks/led_task.cpp` - LED更新タスク (30Hz)
- `main/config.hpp` - GPIO定義
