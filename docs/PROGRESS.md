# StampFly RTOS Skeleton 実装進捗

## 最終更新: 2025-11-28

---

## 完了済みフェーズ

### Phase 1: プロジェクト基盤 ✅ 完了
- ESP-IDF v5.4 プロジェクト構成
- 全コンポーネントのスタブ作成
- CMakeLists.txt 設定

### Phase 2: ドライバ層実装 ✅ 完了
- stampfly_imu (BMI270) - 既存リポジトリ利用
- stampfly_tof (VL53L3CX) - 既存リポジトリ利用
- stampfly_opticalflow (PMW3901) - 既存リポジトリ利用
- stampfly_mag (BMM150) - 新規実装
- stampfly_baro (BMP280) - 新規実装
- stampfly_power (INA3221) - 新規実装

### Phase 3: サービス層実装 ✅ 完了
- stampfly_motor - LEDC PWM、X-quad構成ミキサー
- stampfly_led - WS2812 RMT制御、状態表示パターン
- stampfly_buzzer - LEDC PWM、プリセット音
- stampfly_button - GPIO0デバウンス、長押し検出
- stampfly_filter - LowPassFilter、MedianFilter、OutlierDetector、Vec3
- stampfly_math - Vec3、Quaternion、Matrix テンプレート
- stampfly_eskf - ESKF統合推定器、AttitudeEstimator、AltitudeEstimator、VelocityEstimator
- stampfly_state - StampFlyState (シングルトン)、SystemManager
- stampfly_comm - ESP-NOW通信、ControlPacket、TelemetryPacket、ペアリング機能
- stampfly_cli - USB CDCコンソール、コマンド処理

### Phase 4: タスク統合 ✅ 完了
main.cppに以下を統合:

**FreeRTOSタスク:**
| タスク名 | 周波数 | 優先度 | Core | スタック |
|----------|--------|--------|------|---------|
| IMUTask | 400Hz | 24 | 1 | 8192 |
| OptFlowTask | 100Hz | 20 | 1 | 8192 |
| MagTask | 100Hz | 18 | 1 | 4096 |
| BaroTask | 50Hz | 16 | 1 | 4096 |
| ToFTask | 30Hz | 14 | 1 | 4096 |
| PowerTask | 10Hz | 12 | 0 | 4096 |
| LEDTask | 30Hz | 8 | 0 | 4096 |
| ButtonTask | 100Hz | 10 | 0 | 4096 |
| CommTask | 50Hz | 15 | 0 | 4096 |
| CLITask | - | 5 | 0 | 4096 |

**初期化関数:**
- initI2C() - I2Cバス初期化
- initSensors() - 全センサ初期化
- initActuators() - モーター、LED、ブザー、ボタン
- initEstimators() - ESKF、姿勢/高度推定器
- initCommunication() - ESP-NOW
- initCLI() - USBシリアルCLI

**イベントハンドラ:**
- onButtonEvent() - ARM/DISARM、ペアリング、リセット
- onControlPacket() - コントローラからの制御入力

**ビルド状態:** 成功 (891KB, 72%空き)

---

## 実機テスト結果 (2025-11-28)

### 動作確認済み
| 項目 | 通信 | 値検証 | 備考 |
|------|------|--------|------|
| ブート・起動音 | ✅ | - | 正常に起動 |
| IMU (BMI270) | ✅ | 未確認 | SPI通信成功、値の妥当性は未検証 |
| Mag (BMM150) | ✅ | 未確認 | I2C通信成功、値の妥当性は未検証 |
| Baro (BMP280) | ✅ | 未確認 | I2C通信成功、値の妥当性は未検証 |
| ToF Bottom (VL53L3CX) | ✅ | 未確認 | 通信成功、距離値の妥当性は未検証 |
| ToF Front (VL53L3CX) | ✅ | 未確認 | 通信成功、距離値の妥当性は未検証 |
| OptFlow (PMW3901) | ✅ | 未確認 | SPI通信成功、burst read動作、値の妥当性は未検証 |
| Power (INA3221) | ✅ | 未確認 | 電圧・電流取得、値の妥当性は未検証 |
| LED (WS2812) | ✅ | ✅ | 状態表示パターン動作確認 |
| Buzzer | ✅ | ✅ | 起動音・警告音動作確認 |
| Button | ✅ | ✅ | イベント検出動作確認 |
| ESP-NOW | ✅ | 未確認 | 初期化成功、通信は未テスト |
| CLI | ✅ | ✅ | helpコマンド動作確認、エコーバック正常 |
| CLI teleplot | ✅ | - | Teleplotストリーミング機能追加 |

### CLI コマンド一覧
| コマンド | 説明 |
|---------|------|
| `help` | 利用可能コマンド表示 |
| `status` | システム状態表示 |
| `sensor [imu\|mag\|baro\|tof\|flow\|power\|all]` | センサ値表示（実データ） |
| `teleplot [on\|off]` | Teleplotストリーミング開始/停止 |
| `calib [gyro\|accel\|mag]` | キャリブレーション（stub） |
| `motor [arm\|disarm\|test <id> <throttle>]` | モーター制御（stub） |
| `pair` | ペアリングモード開始 |
| `unpair` | ペアリング解除 |
| `gain <name> <value>` | 制御ゲイン設定（stub） |
| `attitude` | 姿勢表示（stub） |
| `version` | バージョン情報表示 |
| `reset` | システムリセット |

### 次回確認事項: センサ値の妥当性検証
- IMU: 静止時にZ軸加速度≈9.8m/s²、ジャイロ≈0
- Mag: 日本での地磁気≈45μT程度
- Baro: 気圧≈101325Pa、温度≈室温
- ToF: 実測距離との比較
- OptFlow: 移動時のdelta値変化
- Power: テスターでの電圧比較

### 実機テスト中に修正した問題

1. **WiFi/ESP-NOW初期化エラー**
   - 原因: `esp_netif_init()`と`esp_event_loop_create_default()`が未呼び出し
   - 修正: main.cpp で初期化追加

2. **USB CDCコンソール問題**
   - 原因: USB CDCがログ出力と競合
   - 修正: `sdkconfig.defaults`で`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`に変更

3. **スタックオーバーフロー**
   - 原因: タスクスタックサイズ不足 (2048→4096/8192)
   - 修正: 全タスクのスタックサイズ増加

4. **バッテリーチャンネル誤り**
   - 原因: INA3221のCH0→CH1に変更が必要
   - 修正: `battery_channel = 1`に修正

5. **SPI重複初期化エラー**
   - 原因: BMI270とPMW3901でSPIバスを2回初期化
   - 修正: PMW3901に`skip_bus_init`フラグ追加

6. **CLI std::map/ラムダクラッシュ**
   - 原因: 動的メモリ割り当て問題
   - 修正: 静的配列と関数ポインタに変更

7. **起動時の誤低電圧警告**
   - 原因: `last_voltage_v_`初期値が0.0f
   - 修正: 初期値を4.2fに変更

8. **CLIエコーバック遅延**
   - 原因: stdioバッファリングによりESP_LOGと同期して出力
   - 修正: `write()`システムコール直接使用でバッファをバイパス

9. **ToFセンサーI2Cエラー連発**
   - 原因: センサー切断時にエラーログが大量出力
   - 修正: エラー10回連続で該当センサーを自動無効化

---

## 次のステップ: Phase 5

### 残りの Phase 4 作業 (テスト)

#### 単体テスト
| 対象 | テスト内容 | 状態 |
|------|----------|------|
| BMI270Wrapper | 初期化・読み出し | ✅ 実機確認済 |
| BMM150 | 初期化・読み出し | ✅ 実機確認済 |
| BMP280 | 初期化・読み出し | ✅ 実機確認済 |
| VL53L3CXWrapper | 初期化・距離取得 | ✅ 実機確認済 |
| PMW3901 | 初期化・モーション取得 | ✅ 実機確認済 |
| PowerMonitor | 初期化・電圧取得 | ✅ 実機確認済 |
| MotorDriver | PWM出力 | 未実施 |
| LED | パターン表示 | ✅ 実機確認済 |
| Buzzer | 音出力 | ✅ 実機確認済 |
| LowPassFilter | 数値検証 | 未実施 |
| StampFlyState | 状態遷移 | 未実施 |

#### 統合テスト
| テスト名 | 内容 | 状態 |
|---------|------|------|
| 全センサ同時動作 | 全センサタスク起動、データ取得 | ✅ 実機確認済 |
| ESP-NOW通信 | コントローラとの双方向通信 | 未実施 |
| タスク優先度検証 | 全タスク同時稼働時の遅延計測 | 未実施 |
| メモリリーク検出 | 長時間稼働 (1時間) | 未実施 |
| 状態遷移統合 | INIT→CALIBRATING→IDLE→ARMED | 未実施 |
| 低電圧シミュレーション | 3.4V以下検出 | 未実施 |
| ペアリング機能 | ペアリング実行・NVS保存・復元 | 未実施 |

### Phase 5: ドキュメント作成 (未着手)

| ファイル名 | 内容 | 状態 |
|-----------|------|------|
| `docs/api_reference.md` | 全クラスAPI、関数シグネチャ、使用例 | 未着手 |
| `docs/hardware_setup.md` | ハードウェア接続図、GPIO割り当て | 未着手 |
| `docs/calibration_guide.md` | キャリブレーション手順 | 未着手 |
| `docs/control_implementation_guide.md` | 制御実装ガイド | 未着手 |

---

## ファイル構成

```
stampfly_rtos_skelton/
├── main/
│   └── main.cpp                 # ✅ 全タスク統合済み
├── components/
│   ├── stampfly_imu/            # ✅ BMI270 (既存)
│   ├── stampfly_tof/            # ✅ VL53L3CX (既存)
│   ├── stampfly_opticalflow/    # ✅ PMW3901 (既存)
│   ├── stampfly_mag/            # ✅ BMM150 (新規)
│   ├── stampfly_baro/           # ✅ BMP280 (新規)
│   ├── stampfly_power/          # ✅ INA3221 (新規)
│   ├── stampfly_motor/          # ✅ LEDC PWM (新規)
│   ├── stampfly_led/            # ✅ WS2812 (新規)
│   ├── stampfly_buzzer/         # ✅ LEDC PWM (新規)
│   ├── stampfly_button/         # ✅ GPIO0 (新規)
│   ├── stampfly_filter/         # ✅ LPF, Median, Outlier (新規)
│   ├── stampfly_math/           # ✅ Vec3, Quat, Matrix (新規)
│   ├── stampfly_eskf/           # ✅ ESKF (新規)
│   ├── stampfly_state/          # ✅ State, SystemManager (新規)
│   ├── stampfly_comm/           # ✅ ESP-NOW (新規)
│   └── stampfly_cli/            # ✅ CLI (新規)
└── docs/
    ├── implementation_plan.md   # 実装計画
    └── PROGRESS.md              # この進捗ファイル
```

---

## 次回作業への引き継ぎ事項

### 推奨する次のアクション

1. **実機テスト準備**
   - StampFly実機でファームウェアを書き込み
   - センサ接続確認
   - シリアルモニタでログ確認

2. **CLIによる動作確認**
   - `help` コマンドで利用可能コマンド表示
   - `sensor imu` でIMUデータ確認 (現在はstub)
   - `status` でシステム状態確認 (現在はstub)

3. **CLIのstub実装を実データに置き換え**
   - cli.cpp の各センサコマンドで StampFlyState から実データ取得
   - `sensor imu` → state.getIMUData()
   - `sensor mag` → state.getMagData()
   - など

4. **単体テスト実施**
   - test/unit/ ディレクトリ作成
   - 各センサの初期化・読み出しテスト

5. **ドキュメント作成**
   - api_reference.md
   - hardware_setup.md
   - calibration_guide.md
   - control_implementation_guide.md

### 注意点

- PMW3901 は例外ベースのAPI (try-catch必須)
- I2Cセンサは共通の `g_i2c_bus` ハンドルを使用
- ESP-NOW通信は for_tdma ブランチ互換のパケット形式

### ビルド方法

```bash
cd /Users/kouhei/Dropbox/01教育研究/20マルチコプタ/stampfly_rtos_skelton
source /Users/kouhei/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

---

## 変更履歴

| 日付 | 内容 |
|------|------|
| 2025-11-28 | CLI teleplotコマンド追加、sensorコマンドを実データ対応 |
| 2025-11-28 | 実機テスト完了、各種バグ修正、CLI動作確認 |
| 2025-11-27 | Phase 4 タスク統合完了、ビルド成功 |
