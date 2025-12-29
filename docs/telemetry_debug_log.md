# WiFi WebSocketテレメトリ実装 - デバッグログ

## 概要

StampFlyにWiFi APモード + WebSocketによるテレメトリ機能を実装した際に遭遇した問題と解決策の記録。

- **実装日**: 2025-12-29
- **ESP-IDF**: v5.4.1
- **通信方式**: WiFi AP (SoftAP) + WebSocket
- **テレメトリレート**: 50Hz

---

## 実装構成

```
[StampFly (ESP32-S3)]              [PC]
     WiFi AP                    ブラウザ
   "StampFly"                      │
   192.168.4.1                     │
        │                          │
        └── WebSocket (/ws) ───────┘
              50Hz バイナリデータ
```

### 関連ファイル

| ファイル | 役割 |
|---------|------|
| `components/stampfly_telemetry/` | WebSocketサーバー、HTMLページ |
| `components/stampfly_comm/controller_comm.cpp` | WiFi AP初期化 |
| `main/main.cpp` | TelemetryTask (50Hz送信) |

---

## 問題1: WiFi接続後、HTTPアクセス不可

### 症状

- PCからWiFi「StampFly」に接続成功（WiFiアイコン表示）
- ブラウザで `http://192.168.4.1/` にアクセス
- 「インターネットに接続されていません」と表示

### 原因

WiFi APモードを使用する際に **ネットワークインターフェース (esp_netif)** を作成していなかった。

```cpp
// 問題のあるコード
esp_wifi_set_mode(WIFI_MODE_APSTA);
esp_wifi_start();
// → WiFiは動作するが、DHCPサーバーが起動しない
// → PCにIPアドレスが割り当てられない
```

### 解決策

`esp_netif_create_default_wifi_ap()` を追加。

```cpp
// controller_comm.cpp
#include "esp_netif.h"

esp_err_t ControllerComm::init(const Config& config)
{
    // ...

    // ネットワークインターフェース作成（APモード用）
    esp_netif_create_default_wifi_ap();  // ← 追加

    // WiFi初期化
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    // ...
}
```

### 解説

`esp_netif_create_default_wifi_ap()` は以下を行う：

1. APモード用のTCP/IPスタックを初期化
2. DHCPサーバーを起動
3. デフォルトIP `192.168.4.1` を設定
4. 接続クライアントに `192.168.4.2〜` を割り当て

WiFi接続（L2）とIP通信（L3）は別レイヤーであり、WiFiが動作してもIP通信には別途設定が必要。

### 参考

- [ESP-IDF Wi-Fi Driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html)
- [ESP-NETIF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_netif.html)

---

## 問題2: HTMLページ表示後、WebSocket通信が開始しない

### 症状

- HTMLページ（テレメトリUI）は正常に表示
- 姿勢データが更新されない（初期値「---」のまま）
- ブラウザの開発者ツールでWebSocket接続は成功している

### 原因

WebSocketハンドシェイク時にクライアントを登録していなかった。

```cpp
// 問題のあるコード
esp_err_t Telemetry::ws_handler(httpd_req_t* req)
{
    // ハンドシェイク（HTTP GET）
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake from client");
        return ESP_OK;  // 何もせずに返す
    }
    // → クライアントが登録されない
    // → broadcast()で送信先がない
}
```

### 解決策

ハンドシェイク成功時にクライアントfdを登録。

```cpp
esp_err_t Telemetry::ws_handler(httpd_req_t* req)
{
    int fd = httpd_req_to_sockfd(req);

    // ハンドシェイク（HTTP GET）
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake from client (fd=%d)", fd);
        s_instance->addClient(fd);  // ← クライアント登録
        return ESP_OK;
    }
    // ...
}
```

### 解説

ESP-IDFのWebSocketハンドラのライフサイクル：

```
1. クライアントが /ws に接続
   ↓
2. ws_handler() が HTTP GET として呼ばれる
   → ここでクライアントを登録すべき
   ↓
3. ハンドラが ESP_OK を返す
   → WebSocketハンドシェイク完了
   ↓
4. 以降、同じfdでデータ送受信
   → httpd_ws_send_frame_async(server, fd, &frame)
   ↓
5. クライアント切断時
   → ws_handler() が CLOSE フレームで呼ばれる
```

### 参考

- [ESP-IDF HTTP Server (WebSocket)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/esp_http_server.html)
- [WebSocket Echo Server Example](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server)

---

## 問題3: 通信が数秒後に途切れる

### 症状

- 最初は正常にデータが更新される
- 数秒後に更新が停止
- シリアルログ：
  ```
  I (169645) Telemetry: HTTP GET /
  I (169669) Telemetry: Client disconnected (fd=59)
  I (169669) Telemetry: Client disconnected (fd=59, total=0)
  ```

### 原因

`broadcast()` 内で `httpd_ws_get_fd_info()` を使ってクライアント状態をチェックしていたが、HTTPリクエスト処理中にこのチェックが実行されると、WebSocketクライアントが誤って「無効」と判定されていた。

```cpp
// 問題のあるコード
int Telemetry::broadcast(const void* data, size_t len)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        int fd = client_fds_[i];
        if (fd == -1) continue;

        // クライアント状態チェック
        httpd_ws_client_info_t client_info = httpd_ws_get_fd_info(server_, fd);
        if (client_info != HTTPD_WS_CLIENT_WEBSOCKET) {
            // HTTPリクエスト処理中にここに入ってしまう
            client_fds_[i] = -1;
            client_count_--;
            continue;
        }
        // ...
    }
}
```

### トリガー条件

1. ブラウザがHTMLページを表示中
2. 何らかの理由でHTMLを再リクエスト（リロード、フォーカス復帰等）
3. HTTPリクエスト処理中に `broadcast()` が実行される
4. `httpd_ws_get_fd_info()` が不正確な値を返す
5. WebSocketクライアントが削除される

### 解決策

`httpd_ws_get_fd_info()` による事前チェックを削除し、送信結果のエラーコードのみで判定。

```cpp
int Telemetry::broadcast(const void* data, size_t len)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        int fd = client_fds_[i];
        if (fd == -1) continue;

        // 事前チェックなしで送信
        httpd_ws_frame_t ws_pkt = {
            .payload = (uint8_t*)data,
            .len = len,
            .type = HTTPD_WS_TYPE_BINARY,
            .final = true
        };

        esp_err_t ret = httpd_ws_send_frame_async(server_, fd, &ws_pkt);
        if (ret == ESP_OK) {
            sent_count++;
        } else if (ret == ESP_ERR_INVALID_ARG) {
            // fdが無効な場合のみ削除
            client_fds_[i] = -1;
            client_count_--;
        }
        // 他のエラー（EAGAIN等）は無視して次回再試行
    }
}
```

### 解説

ESP-IDFのHTTPサーバーは**シングルスレッド**で動作する。HTTPリクエストとWebSocket通信が同じコンテキストで処理されるため、タイミングによっては状態が不整合になる。

送信エラーコードでの判定が確実：

| エラーコード | 意味 | 対応 |
|-------------|------|------|
| `ESP_OK` | 送信成功 | カウントアップ |
| `ESP_ERR_INVALID_ARG` | fdが無効 | クライアント削除 |
| `ESP_ERR_HTTPD_INVALID_REQ` | 一時的エラー | 無視（再試行） |
| その他 | 不明なエラー | 無視（再試行） |

---

## 最終的なアーキテクチャ

### コード構成

```
components/stampfly_telemetry/
├── CMakeLists.txt
├── include/
│   └── telemetry.hpp      # Telemetryクラス、パケット定義
├── telemetry.cpp          # WebSocketサーバー実装
└── www/
    └── index.html         # ブラウザUI（埋め込み）

components/stampfly_comm/
└── controller_comm.cpp    # WiFi APSTA初期化

main/
└── main.cpp               # TelemetryTask (50Hz)
```

### 初期化フロー

```
app_main()
  │
  ├─ esp_netif_init()           # ネットワークスタック初期化
  ├─ esp_event_loop_create()    # イベントループ
  │
  ├─ ControllerComm::init()
  │    ├─ esp_netif_create_default_wifi_ap()  # AP用netif
  │    ├─ esp_wifi_init()
  │    ├─ esp_wifi_set_mode(APSTA)
  │    ├─ esp_wifi_set_config(AP)  # SSID: StampFly
  │    ├─ esp_wifi_set_ps(NONE)    # 電力節約OFF
  │    ├─ esp_wifi_start()
  │    └─ esp_now_init()           # ESP-NOW共存
  │
  ├─ Telemetry::init()
  │    ├─ httpd_start()
  │    ├─ httpd_register_uri_handler("/")   # HTML配信
  │    └─ httpd_register_uri_handler("/ws") # WebSocket
  │
  └─ xTaskCreatePinnedToCore(TelemetryTask, Core0)
```

### データフロー

```
[Core 1: IMUTask 400Hz]
        │
        ▼
[StampFlyState] ←── 他センサタスク
        │
        │ getAttitudeEuler()
        │ getPosition()
        │ getVelocity()
        ▼
[Core 0: TelemetryTask 50Hz]
        │
        │ broadcast()
        ▼
[WebSocket clients]
        │
        ▼
[ブラウザ JavaScript]
```

---

## 教訓まとめ

### 1. WiFi APモードの必須設定

```cpp
// 必ず両方必要
esp_netif_create_default_wifi_ap();  // DHCPサーバー起動
esp_wifi_set_mode(WIFI_MODE_AP);     // または WIFI_MODE_APSTA
```

### 2. WebSocketのライフサイクル理解

```
HTTP GET → ハンドシェイク → データ送受信 → CLOSE
    │
    └─ ここでクライアント登録
```

### 3. HTTPサーバーのスレッドモデル

- ESP-IDFのHTTPサーバーはシングルスレッド
- HTTPとWebSocketが同一コンテキストで処理される
- 状態チェックはタイミング依存で不安定になりうる
- 送信結果のエラーコードで判断するのが確実

### 4. デバッグのポイント

1. `idf.py monitor` でシリアルログを常時確認
2. ログに十分な情報を出力（fd番号、カウント等）
3. ブラウザの開発者ツールでWebSocket状態を確認
4. 問題の再現条件を特定（リロード、フォーカス等）

---

## 参考リンク

- [ESP-IDF HTTP Server](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/esp_http_server.html)
- [ESP-IDF Wi-Fi Driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html)
- [ESP-NETIF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_netif.html)
- [WebSocket Echo Server Example](https://github.com/espressif/esp-idf/tree/master/examples/protocols/http_server/ws_echo_server)
