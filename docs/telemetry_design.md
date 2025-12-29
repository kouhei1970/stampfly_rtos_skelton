# WiFiテレメトリ設計ドキュメント

## 概要

StampFlyにWiFiベースのテレメトリ機能を追加するための技術調査と設計方針をまとめる。

## 要件

1. **テレメトリ送信**: 機体の状態（姿勢、位置、センサー値等）をリアルタイムで送信
2. **パラメータ変更**: 飛行中にPIDゲイン等を調整可能
3. **受信側**: PCブラウザベースのアプリでデータ保存・可視化・分析
4. **操縦統合**: コントローラもWiFi経由で操縦（ESP-NOWとの混在を避ける）

## 通信方式の選択肢

### 1. WebSocket over WiFi

| 項目 | 評価 |
|------|------|
| ブラウザ対応 | ◎ 標準対応 |
| 双方向通信 | ◎ ネイティブサポート |
| レイテンシ | ○ 典型15-30ms |
| 実装難易度 | ○ esp_http_server拡張 |

**ESP-IDFサポート**: `esp_http_server`にWebSocketサポートが組み込まれている。menuconfig で有効化が必要。

参考: [ESP-IDF WebSocket Echo Server Example](https://github.com/espressif/esp-idf/tree/v5.2.1/examples/protocols/http_server/ws_echo_server)

### 2. UDP over WiFi

| 項目 | 評価 |
|------|------|
| ブラウザ対応 | × 直接不可（中継サーバー必要） |
| 双方向通信 | ○ 可能 |
| レイテンシ | ◎ 最小（再送なし） |
| 実装難易度 | ◎ シンプル |

操縦のような低レイテンシが必要な用途には有利だが、ブラウザから直接接続できない。

### 3. ESP-NOW

| 項目 | 評価 |
|------|------|
| ブラウザ対応 | × 不可 |
| 双方向通信 | ○ 可能 |
| レイテンシ | ◎ 1-2ms典型 |
| 到達距離 | ◎ 最大1km |

**WiFiとの同時使用**: 可能だが制約あり
- `WIFI_MODE_APSTA`モードを使用
- 同一チャンネルに制限される
- パケットロスが発生しやすいという報告あり

参考: [ESP32: WiFi and ESP-Now simultaneously](https://www.electrosoftcloud.com/en/esp32-wifi-and-esp-now-simultaneously/)

## レイテンシの実態

### 典型値と最悪値

| 方式 | 典型値 | 最悪値 | 備考 |
|------|--------|--------|------|
| ESP-NOW | 1-2ms | ~10ms | 専用プロトコル |
| WiFi UDP | 7-15ms | 200-388ms | 干渉環境で悪化 |
| WiFi WebSocket | 15-30ms | 数百ms | TCP再送含む |

### 最悪値が大きくなる要因

1. **2.4GHz帯の干渉**: 他のWiFi、Bluetooth、電子レンジ等
2. **パケット再送**: 破損パケットのリカバリ
3. **電力節約モード**: デフォルトで有効、数百ms遅延の原因
4. **チャンネル混雑**: 同一チャンネルの他デバイス

参考: [ESP32 Forum - UDP packet delays](https://esp32.com/viewtopic.php?t=27665)

## レイテンシ最適化

### 必須設定

```c
// WiFi電力節約モードを無効化（必須）
esp_wifi_set_ps(WIFI_PS_NONE);
```

これだけで典型レイテンシが数百msから15ms程度に改善される。

### 推奨設定

```c
// menuconfigで設定
CONFIG_ESP_WIFI_IRAM_OPT=y           // WiFi関数をIRAMに配置
CONFIG_LWIP_IRAM_OPTIMIZATION=y      // LwIPをIRAMに配置
CONFIG_ESPTOOLPY_FLASHFREQ="80m"     // フラッシュ速度を80MHzに
```

### APモード vs STAモード

- **APモード（SoftAP）**: ESP32自身がアクセスポイントになる
  - レイテンシ: ~17ms
  - ルーター不要
  - 接続範囲は限定的

- **STAモード**: 既存のルーターに接続
  - レイテンシ: ~200ms以上になることも
  - ルーター経由の遅延が追加

**推奨**: テレメトリにはAPモードを使用

参考: [ESP32 Forum - Slow WiFi after migrating](https://www.esp32.com/viewtopic.php?t=6073)

## コア割り当て

ESP32-S3はデュアルコア。WiFiとリアルタイム制御を分離する。

| コア | タスク |
|------|--------|
| Core 0 | WiFi/テレメトリ（ESP-IDFのWiFiスタックはCore 0で動作） |
| Core 1 | IMU読み取り、ESKF、制御ループ |

WiFiがブロックしても制御に影響しない設計が重要。

## 既存プロジェクト参考

### DroneBridge for ESP32

[DroneBridge/ESP32](https://github.com/DroneBridge/ESP32) - ドローン向けテレメトリの実績あるプロジェクト

特徴:
- WiFi、WiFi-LR、ESP-NOW対応
- MAVLink、MSP、LTMプロトコル対応
- 暗号化（AES-256-GCM）
- 到達距離: WiFi ~150m、ESP-NOW ~1km

参考になるポイント:
- パケットサイズとレイテンシのトレードオフ
- 複数通信方式の切り替え設計

## 推奨アーキテクチャ

### 構成図

```
[StampFly (ESP32-S3)]          [PC]
        │                        │
   WiFi AP Mode              ブラウザ
   192.168.4.1               │
        │                    │
        └── WebSocket ───────┘
            (テレメトリ + 操縦)
```

### 通信プロトコル

1. **テレメトリ下り（機体→PC）**: WebSocket Binary
   - 50-100Hz程度（400Hzからデシメーション）
   - バイナリフォーマットで帯域節約

2. **操縦上り（PC→機体）**: WebSocket Binary
   - 50Hz程度
   - スロットル、Roll/Pitch/Yawコマンド

3. **パラメータ設定**: WebSocket Text (JSON)
   - 低頻度
   - PIDゲイン、設定値の読み書き

### コントローラ統合

```
[コントローラ (ESP32-S3)]
        │
   WiFi STA Mode
        │
        └── WebSocket ──→ [StampFly AP]
```

コントローラもWebSocketクライアントとして接続。
PC（ブラウザ）と同じプロトコルで操縦データを送信。

## フェイルセーフ

WiFi通信が不安定な場合の安全機構:

1. **ハートビート監視**: 操縦パケットが一定時間（例: 500ms）途絶したら検知
2. **自動着陸**: 通信途絶時にスロットルを徐々に下げる
3. **ホールド**: GPSがあれば位置保持（将来）

## 実装フェーズ

### Phase 1: 基本テレメトリ
- WiFi APモードでWebSocketサーバー起動
- 姿勢データ（Roll/Pitch/Yaw）を50Hzで送信
- ブラウザで受信・表示

### Phase 2: 双方向通信
- パラメータ読み書き
- 操縦コマンド受信

### Phase 3: コントローラ対応
- コントローラ側ファームウェア
- WebSocketクライアント実装

## 帯域見積もり

### テレメトリデータ（50Hz）

| データ | バイト数 |
|--------|----------|
| タイムスタンプ | 4 |
| Roll/Pitch/Yaw | 12 |
| 位置 (x,y,z) | 12 |
| 速度 (vx,vy,vz) | 12 |
| モーター出力 (4ch) | 8 |
| バッテリー電圧 | 4 |
| **合計** | ~52 bytes |

50Hz × 52 bytes = 2,600 bytes/s ≒ **21 kbps**

WebSocketオーバーヘッド込みでも **~30 kbps** 程度。WiFiの帯域的には余裕。

## 結論と推奨

1. **WebSocket over WiFi APモード**を採用
2. **操縦もWiFi統合**でESP-NOWとの混在を避ける
3. **電力節約モード無効化**でレイテンシ改善
4. **Core分離**で制御への影響を最小化
5. **フェイルセーフ**を必ず実装

レイテンシの最悪値（100ms超）は懸念だが、以下で緩和:
- APモード使用（ルーター経由の遅延排除）
- 電力節約無効化
- 操縦コマンドにタイムスタンプ付与、古いコマンドは破棄

## 参考リンク

- [DroneBridge for ESP32](https://github.com/DroneBridge/ESP32)
- [ESP-IDF HTTP Server (WebSocket)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/esp_http_server.html)
- [ESP-IDF RF Coexistence](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/coexist.html)
- [ESP32 WiFi Driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html)
- [Electric UI - Latency Comparison](https://electricui.com/blog/latency-comparison)
