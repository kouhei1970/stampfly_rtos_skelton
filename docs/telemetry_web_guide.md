# WebSocket Telemetry 可視化システム入門ガイド

ESP32からブラウザへリアルタイムでテレメトリデータを送信し、可視化するシステムの構築方法を解説します。

## 1. システム構成

```
┌─────────────────┐     WiFi AP      ┌─────────────────┐
│     ESP32       │  ─────────────>  │    Browser      │
│  (WebSocket     │    WebSocket     │  (HTML/CSS/JS)  │
│   Server)       │    Binary Data   │                 │
└─────────────────┘                  └─────────────────┘
```

### 通信フロー

1. ESP32がWiFi APモードで起動（例: SSID "StampFly"）
2. ブラウザがWiFi APに接続
3. ブラウザが `ws://192.168.4.1/ws` にWebSocket接続
4. ESP32が定期的にバイナリデータを送信（例: 50Hz）
5. ブラウザがデータを受信・パース・表示

## 2. ESP32側の実装

### 2.1 必要なコンポーネント

```cmake
# CMakeLists.txt
idf_component_register(
    SRCS "telemetry.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server freertos
    EMBED_FILES "www/index.html"  # HTMLをファームウェアに埋め込み
)
```

### 2.2 HTTPサーバーとWebSocketの設定

```cpp
#include "esp_http_server.h"

// HTTPサーバー起動
httpd_config_t config = HTTPD_DEFAULT_CONFIG();
config.server_port = 80;
httpd_handle_t server;
httpd_start(&server, &config);

// HTMLページを提供するハンドラ
httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_get_handler,
    .user_ctx = nullptr
};
httpd_register_uri_handler(server, &uri_root);

// WebSocketエンドポイント
httpd_uri_t uri_ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = nullptr,
    .is_websocket = true,              // WebSocketを有効化
    .handle_ws_control_frames = true
};
httpd_register_uri_handler(server, &uri_ws);
```

### 2.3 埋め込みファイルの提供

```cpp
// ファームウェアに埋め込まれたHTMLファイルへの参照
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

esp_err_t http_get_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}
```

### 2.4 データパケット構造（バイナリ）

```cpp
// パケット構造体（パディングなし）
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  header;        // 0xAA（パケット識別子）
    uint8_t  type;          // パケットタイプ
    uint32_t timestamp_ms;  // タイムスタンプ
    float    roll;          // 姿勢データ [rad]
    float    pitch;
    float    yaw;
    // ... その他のデータ
    uint8_t  checksum;      // チェックサム
};
#pragma pack(pop)
```

**重要**: `#pragma pack(push, 1)` でパディングを無効化し、構造体のバイト配置を制御。

### 2.5 WebSocketでデータ送信

```cpp
void broadcast_telemetry(httpd_handle_t server, int client_fd) {
    TelemetryPacket pkt;
    pkt.header = 0xAA;
    pkt.timestamp_ms = esp_timer_get_time() / 1000;
    pkt.roll = current_roll;
    // ... データ設定

    // WebSocketフレームとして送信
    httpd_ws_frame_t ws_pkt = {
        .payload = (uint8_t*)&pkt,
        .len = sizeof(pkt),
        .type = HTTPD_WS_TYPE_BINARY,  // バイナリフレーム
        .final = true
    };

    httpd_ws_send_frame_async(server, client_fd, &ws_pkt);
}
```

## 3. ブラウザ側の実装

### 3.1 基本的なHTML構造

```html
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Telemetry</title>
</head>
<body>
    <div id="roll">---</div>
    <div id="pitch">---</div>
    <div id="yaw">---</div>

<script>
// JavaScriptコードはここに
</script>
</body>
</html>
```

### 3.2 WebSocket接続

```javascript
// WebSocket接続
const ws = new WebSocket('ws://192.168.4.1/ws');
ws.binaryType = 'arraybuffer';  // バイナリデータを受信するために必須

ws.onopen = () => {
    console.log('Connected');
};

ws.onclose = () => {
    console.log('Disconnected');
    // 再接続ロジック
    setTimeout(() => location.reload(), 2000);
};

ws.onerror = (err) => {
    console.error('Error:', err);
};

ws.onmessage = (event) => {
    // event.data はArrayBuffer
    processPacket(event.data);
};
```

### 3.3 バイナリデータのパース

```javascript
function processPacket(buffer) {
    // ArrayBufferが必要
    if (!(buffer instanceof ArrayBuffer)) return;

    // DataViewでバイナリデータにアクセス
    const data = new DataView(buffer);

    // サイズチェック
    if (data.byteLength < 48) return;

    // ヘッダーチェック
    if (data.getUint8(0) !== 0xAA) return;

    // データ抽出（リトルエンディアン = true）
    const timestamp = data.getUint32(2, true);  // オフセット2, 4バイト
    const roll = data.getFloat32(6, true);      // オフセット6, 4バイト
    const pitch = data.getFloat32(10, true);    // オフセット10, 4バイト
    const yaw = data.getFloat32(14, true);      // オフセット14, 4バイト

    // 表示更新
    const RAD_TO_DEG = 180 / Math.PI;
    document.getElementById('roll').textContent = (roll * RAD_TO_DEG).toFixed(1);
    document.getElementById('pitch').textContent = (pitch * RAD_TO_DEG).toFixed(1);
    document.getElementById('yaw').textContent = (yaw * RAD_TO_DEG).toFixed(1);
}
```

### 3.4 DataViewのオフセット計算

パケット構造体とJavaScriptのオフセットを一致させる：

```
C++ struct:                    JavaScript offset:
─────────────────────────────────────────────────
uint8_t  header      (1byte)   getUint8(0)
uint8_t  type        (1byte)   getUint8(1)
uint32_t timestamp   (4bytes)  getUint32(2, true)
float    roll        (4bytes)  getFloat32(6, true)
float    pitch       (4bytes)  getFloat32(10, true)
float    yaw         (4bytes)  getFloat32(14, true)
...
```

**注意**: `true`はリトルエンディアン（ESP32のバイトオーダー）

## 4. Three.js 3D可視化

### 4.1 Three.jsの基本概念

```javascript
// シーン: 3Dオブジェクトを配置する空間
const scene = new THREE.Scene();

// カメラ: シーンを見る視点
const camera = new THREE.PerspectiveCamera(
    60,                          // 視野角 (degrees)
    width / height,              // アスペクト比
    0.1,                         // near clip
    1000                         // far clip
);
camera.position.set(2, 2, 2);    // カメラ位置
camera.lookAt(0, 0, 0);          // 注視点

// レンダラー: シーンを描画
const renderer = new THREE.WebGLRenderer({ canvas: canvasElement });
renderer.setSize(width, height);

// アニメーションループ
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();
```

### 4.2 座標系の変換

**NED座標系（ドローン）**:
- X: 北（前方）
- Y: 東（右）
- Z: 下

**Three.js座標系**:
- X: 右
- Y: 上
- Z: 手前（画面外へ）

変換:
```javascript
// NED → Three.js
threeX = ned_X;      // 北 → X
threeY = -ned_Z;     // 上 = -下
threeZ = ned_Y;      // 東 → Z
```

### 4.3 オブジェクトの作成と更新

```javascript
// ジオメトリ（形状）
const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.02);

// マテリアル（材質）
const material = new THREE.MeshLambertMaterial({ color: 0x00ff88 });

// メッシュ（ジオメトリ + マテリアル）
const drone = new THREE.Mesh(geometry, material);
scene.add(drone);

// 位置と回転の更新
function updateDrone(roll, pitch, yaw, pos_x, pos_y, pos_z) {
    // 位置（NED → Three.js変換）
    drone.position.set(pos_x, -pos_z, pos_y);

    // 回転（オイラー角）
    drone.rotation.set(roll, -yaw, pitch, 'YXZ');
}
```

## 5. よくある問題と対策

### 5.1 表示が止まる

**原因**: JavaScript例外が発生している

**対策**: try-catchでエラーをキャッチ
```javascript
ws.onmessage = (event) => {
    try {
        processPacket(event.data);
    } catch (e) {
        console.error('Error:', e);
        // エラーカウンタを表示
        errorCount++;
        document.getElementById('errors').textContent = errorCount;
    }
};
```

### 5.2 Three.jsでメモリ問題

**原因**: 毎フレーム新しいオブジェクトを生成

**悪い例**:
```javascript
function update() {
    // 毎回新しい配列を生成 → メモリリーク
    const positions = new Float32Array(count * 3);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
}
```

**良い例**:
```javascript
// 事前に確保
const positions = new Float32Array(MAX_COUNT * 3);
const positionAttribute = new THREE.BufferAttribute(positions, 3);
geometry.setAttribute('position', positionAttribute);

function update() {
    // 既存の配列を更新
    positions[0] = newX;
    positions[1] = newY;
    positions[2] = newZ;
    positionAttribute.needsUpdate = true;  // 更新フラグ
}
```

### 5.3 CDN依存の問題

ESP32のWiFi APはインターネット接続がないため、CDNからライブラリを読み込めない。

**対策**: ライブラリをファームウェアに埋め込む
```cmake
EMBED_FILES "www/index.html" "www/three.min.js"
```

### 5.4 ブラウザキャッシュ

ファームウェア更新後も古いHTMLが表示される。

**対策**: ハードリフレッシュ
- Windows/Linux: `Ctrl+Shift+R`
- Mac: `Cmd+Shift+R`

## 6. デバッグのコツ

### 6.1 段階的に実装

1. まずWebSocket接続のみ確認
2. テキストデータ送受信を確認
3. バイナリデータに移行
4. 数値表示を追加
5. グラフを追加
6. 最後に3D表示

### 6.2 ブラウザコンソールを活用

```javascript
ws.onmessage = (event) => {
    console.log('Received:', event.data.byteLength, 'bytes');
    // ...
};
```

ブラウザでF12キー → Console タブで確認。

### 6.3 エラー表示をUIに組み込む

開発中はエラーを画面に表示すると便利：
```javascript
catch (e) {
    document.getElementById('status').textContent = 'ERR: ' + e.message;
}
```

## 7. 参考リンク

- [ESP-IDF HTTP Server](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html)
- [Three.js Documentation](https://threejs.org/docs/)
- [MDN WebSocket API](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [MDN DataView](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/DataView)
