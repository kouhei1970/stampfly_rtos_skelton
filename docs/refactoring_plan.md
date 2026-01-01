# main.cpp リファクタリング - 全体設計

## 現状分析

### main.cpp で行われている処理（コンポーネントとの関係）

| 処理 | 行数 | 現在の場所 | 関連コンポーネント |
|------|------|-----------|-------------------|
| **座標変換** | ~30行 | main.cpp各タスク | stampfly_imu等 |
| BMI270→NED | | IMUTask | stampfly_imu |
| BMM150→NED | | MagTask | stampfly_mag |
| PMW3901→中間座標 | | OptFlowTask | stampfly_opticalflow |
| **センサーフュージョン呼び出し** | ~100行 | IMUTask | stampfly_eskf |
| predict(400Hz) | | | |
| updateAccelAttitude(400Hz) | | | |
| updateFlowRaw(100Hz) | | | |
| updateBaro(50Hz) | | | |
| updateToF(30Hz) | | | |
| updateMag(10Hz) | | | |
| **ヘルスチェック** | ~60行 | 各タスク分散 | なし |
| 連続成功/失敗カウント | | | |
| healthy/unhealthyフラグ | | | |
| **LPF適用** | ~10行 | IMUTask | stampfly_filter |
| **バイナリログ構築** | ~60行 | IMUTask | stampfly_logger |
| **発散検出・リセット** | ~30行 | IMUTask | stampfly_eskf |

### 既存コンポーネント（19個）

```
センサー系:   stampfly_imu, stampfly_mag, stampfly_baro, stampfly_tof, stampfly_opticalflow
推定系:       stampfly_eskf, stampfly_filter
状態管理:     stampfly_state
通信系:       stampfly_comm, stampfly_telemetry, stampfly_logger, stampfly_cli
アクチュエータ: stampfly_motor, stampfly_led, stampfly_buzzer, stampfly_button
その他:       stampfly_power, stampfly_math, stampfly_pid
```

---

## 私の意見

### 現状の問題点

1. **センサードライバが「生データ」を返す**
   - BMI270Wrapper::readSensorData() は**センサー座標系**で返す
   - 機体座標系(NED)への変換はmain.cppで実装
   - 学習者：「このセンサーの値はどの方向？」が分かりにくい

2. **ESKFコンポーネントが「数学ライブラリ」にとどまっている**
   - ESKF.predict(), ESKF.updateXXX() は純粋な計算
   - 「どのセンサーをどの周期で融合するか」はmain.cpp
   - 学習者：「センサーフュージョンってどうやるの？」が分かりにくい

3. **ヘルスチェックが各タスクに分散**
   - 同じパターン（連続成功/失敗カウント）が繰り返し実装
   - 学習者：全体のセンサー状態を把握しにくい

### 設計思想の選択

**選択肢A: センサードライバは薄く、フュージョンは別コンポーネント**
- センサードライバ：ハードウェアアクセスのみ
- 座標変換・融合は上位レイヤー（stampfly_fusionなど）

**選択肢B: センサードライバを厚くする**
- 各ドライバが座標変換済みデータを返す
- 融合ロジックはmain/かstampfly_eskfに統合

私の推奨は **選択肢A** です。理由：
- センサードライバは汎用的に保てる（座標系は機体依存）
- 融合ロジックを1箇所にまとめた方が学習しやすい
- 責務が明確（ドライバ＝ハード、フュージョン＝アルゴリズム）

---

## リファクタリング案

### 案1: main/内ファイル分割（最小変更）

```
main/
├── main.cpp          (~300行) - app_main、初期化呼び出し、タスク起動
├── config.hpp                 - GPIO、優先度、定数
├── globals.hpp/cpp            - グローバル変数
├── init.cpp                   - 初期化関数群
├── tasks/
│   ├── sensor_tasks.cpp       - IMU/Mag/Baro/ToF/OptFlow タスク
│   ├── control_task.cpp       - ControlTask
│   ├── peripheral_tasks.cpp   - LED/Button/Power タスク
│   └── comm_tasks.cpp         - Comm/CLI/Telemetry タスク
└── fusion.cpp                 - センサーフュージョンロジック
```

**メリット**:
- 変更が局所的、既存動作への影響小
- main.cppを見れば全体像が分かる

**デメリット**:
- コンポーネントには戻らない（main専用のまま）
- 他プロジェクトで再利用しにくい

---

### 案2: 新コンポーネント追加（中規模）

```
components/
├── stampfly_fusion/     (新規) - センサーフュージョン統合
│   ├── sensor_fusion.cpp      - ESKF呼び出しパターン
│   ├── sensor_fusion.hpp
│   └── CMakeLists.txt
├── stampfly_health/     (新規) - センサーヘルスチェック
│   ├── health_monitor.cpp     - 共通ヘルスチェックロジック
│   ├── health_monitor.hpp
│   └── CMakeLists.txt

main/
├── main.cpp          (~500行) - 初期化、タスク起動、全体フロー
├── config.hpp                 - GPIO、優先度
├── tasks.cpp                  - タスク関数群（簡素化）
└── CMakeLists.txt
```

**メリット**:
- 責務がコンポーネント単位で明確
- stampfly_fusionを見れば融合の全体像が分かる
- 再利用可能

**デメリット**:
- 依存関係が増える
- グローバル変数の扱いに工夫が必要

---

### 案3: センサードライバに座標変換を統合（大規模）

```
components/
├── stampfly_imu/
│   └── bmi270_wrapper.cpp
│       + setFrameTransform(...)  - 座標変換行列を設定
│       + readBodyFrame(...)      - 機体座標系で返す
├── stampfly_mag/
│   └── bmm150.cpp
│       + readBodyFrame(...)
├── stampfly_fusion/     (新規)
└── stampfly_health/     (新規)

main/
├── main.cpp          (~400行)
├── config.hpp
└── tasks.cpp
```

**メリット**:
- 座標変換がセンサー初期化時に確定
- タスクコードが最もシンプル
- 「センサー→機体座標」が各ドライバで完結

**デメリット**:
- 既存APIの変更（破壊的変更）
- センサードライバが機体依存になる（汎用性低下）

---

### 案4: stampfly_appコンポーネント新設（最大規模）

```
components/
├── stampfly_app/        (新規) - アプリケーション全体
│   ├── app.cpp                - 初期化、タスク起動
│   ├── tasks/
│   │   ├── sensor_tasks.cpp
│   │   ├── control_task.cpp
│   │   └── comm_tasks.cpp
│   ├── fusion/
│   │   └── sensor_fusion.cpp
│   └── config.hpp
├── stampfly_fusion/     (新規)
└── stampfly_health/     (新規)

main/
└── main.cpp          (~30行)
    extern "C" void app_main() {
        stampfly::App::run();
    }
```

**メリット**:
- main.cppが最小（エントリポイントのみ）
- 全アプリロジックがコンポーネント化
- 最も再利用性が高い

**デメリット**:
- 大幅な構造変更
- 開発工数が大きい
- 初学者には構造が複雑に見える可能性

---

## 推奨案

**案2（新コンポーネント追加）を推奨**

理由：
1. **学習目的に最適**:
   - stampfly_fusionを見れば「センサーフュージョンとは何か」が分かる
   - main.cppを見れば「RTOSタスク構造」が分かる
   - 責務が分離されて追いやすい

2. **変更規模が適切**:
   - 既存センサードライバは変更不要
   - main.cppは縮小するが消滅しない
   - 段階的に実装可能

3. **将来性**:
   - 案4への移行も容易
   - コンポーネント単位でのテストが可能

---

## 案2の詳細設計

### stampfly_fusion コンポーネント

```cpp
// sensor_fusion.hpp
namespace stampfly {

class SensorFusion {
public:
    struct Config {
        float imu_dt = 0.0025f;      // 400Hz
        float flow_dt = 0.01f;       // 100Hz
        // センサー周期設定
    };

    void init(const Config& config);

    // IMU更新（400Hz、メインループ）
    void predictIMU(const Vec3& accel_body, const Vec3& gyro_body);

    // 各センサー更新（非同期）
    void updateOpticalFlow(int16_t dx, int16_t dy, float distance,
                           float gyro_x, float gyro_y);
    void updateBarometer(float altitude);
    void updateToF(float distance);
    void updateMagnetometer(const Vec3& mag_body);

    // 状態取得
    ESKFState getState() const;
    bool isHealthy() const;
    void reset();

private:
    ESKF eskf_;
    HealthMonitor health_;  // or inline

    bool checkDivergence();
};

} // namespace stampfly
```

### stampfly_health コンポーネント（オプション）

```cpp
// health_monitor.hpp
namespace stampfly {

class SensorHealth {
public:
    void recordSuccess();
    void recordFailure();
    bool isHealthy() const;

private:
    int consecutive_success_ = 0;
    int consecutive_failure_ = 0;
    static constexpr int HEALTHY_THRESHOLD = 10;
    static constexpr int UNHEALTHY_THRESHOLD = 10;
};

class HealthMonitor {
public:
    SensorHealth imu, mag, baro, tof_bottom, tof_front, optflow;

    uint32_t getHealthyMask() const;  // ビットマスクで返す
};

} // namespace stampfly
```

### main.cpp（リファクタリング後）

```cpp
extern "C" void app_main() {
    // 1. 基本初期化
    initNVS();
    initI2C();

    // 2. センサー初期化
    initSensors();      // IMU, Mag, Baro, ToF, OptFlow

    // 3. 推定器初期化
    initFusion();       // SensorFusion (ESKFを内包)

    // 4. アクチュエータ初期化
    initActuators();    // Motor, LED, Buzzer

    // 5. 通信初期化
    initCommunication();

    // 6. タスク起動
    startTasks();

    // 7. メインループ（ウォッチドッグ等）
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## 次のステップ

方針を決定後、以下の順序で実装：

1. **stampfly_fusion コンポーネント作成**
2. **stampfly_health コンポーネント作成**（オプション）
3. **main.cpp からフュージョンロジック移行**
4. **main/ 内ファイル分割**（tasks.cpp, init.cpp, config.hpp）
5. **ビルド・動作確認**

---

---

# ソフトウェアアーキテクチャ調査

## 分野別アーキテクチャパターン

### 1. ロボット開発（ROS 2）

**構造**: レイヤー + Pub/Sub

```
Application Layer     : ユーザーノード
Client Library Layer  : rclcpp, rclpy（言語別API）
RCL Layer            : 共通C API
RMW Layer            : DDS抽象化（ミドルウェア交換可能）
DDS Layer            : 通信ミドルウェア
```

**通信パターン**:
- **Topics**: 非同期Pub/Sub（センサーデータ等）
- **Services**: 同期リクエスト/レスポンス
- **Actions**: 長時間タスク（フィードバック付き）

**特徴**:
- 分散システム前提の設計
- Managed Nodes（ライフサイクル管理）
- Composition（複数ノードを1プロセスに統合）

**参考**: [ROS 2 Architecture](https://medium.com/software-architecture-foundations/robot-operating-system-2-ros-2-architecture-731ef1867776)

---

### 2. ドローン（PX4）

**構造**: モジュール + Pub/Sub（uORB）

```
┌─────────────────────────────────────────┐
│           Application Modules           │
│  (Navigator, Commander, MC_POS_CONTROL) │
└─────────────────┬───────────────────────┘
                  │ uORB (Pub/Sub)
┌─────────────────▼───────────────────────┐
│           Flight Stack                  │
│  Estimator ─→ Controller ─→ Actuators   │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│           Middleware (uORB)             │
│  共有メモリベース、非同期メッセージング      │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│           Drivers / HAL                 │
└─────────────────────────────────────────┘
```

**uORBの特徴**:
- **トピック**: 意味的なチャネル（"attitude", "position"等）
- **非同期**: Publisher/Subscriberは独立動作
- **共有メモリ**: 全モジュールが同一アドレス空間
- **リアクティブ**: 機能は交換可能コンポーネントに分割

**学習ポイント**:
- 各モジュールは`main()`を持つスタンドアロンアプリ
- 起動・停止がランタイムで可能
- センサー→推定器→制御器→アクチュエータの明確なパイプライン

**参考**: [PX4 uORB](https://docs.px4.io/main/en/middleware/uorb), [PX4 Architecture](https://docs.px4.io/main/en/concept/architecture.html)

---

### 3. ドローン（ArduPilot）

**構造**: HAL + 共有ライブラリ

```
┌─────────────────────────────────────────┐
│         Vehicle Code                    │
│  (Copter, Plane, Rover, Sub, etc.)      │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│         Shared Libraries                │
│  AP_AHRS, AP_NavEKF, AP_Motors,        │
│  AP_GPS, AP_Baro, AP_InertialSensor... │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│         AP_HAL (Hardware Abstraction)   │
│  純粋仮想クラス（インターフェース定義）    │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│    AP_HAL_ChibiOS / AP_HAL_Linux / ...  │
│  プラットフォーム固有実装                 │
└─────────────────────────────────────────┘
```

**HALの特徴**:
- **純粋仮想クラス**: AP_HALは具象コードを持たない
- **単一インスタンス**: `hal`という名前で参照
- **ボード定義**: hwdef.datファイルでハード構成を記述

**主要HALクラス**:
- `AP_HAL::GPIO` - GPIO制御
- `AP_HAL::I2CDriver` - I2Cタイムアウト対応
- `AP_HAL::Scheduler` - 非同期スケジューリング
- `AP_HAL::SPIDriver` - SPI管理

**参考**: [ArduPilot Libraries](https://ardupilot.org/dev/docs/apmcopter-programming-libraries.html)

---

### 4. 自動車（AUTOSAR Classic）

**構造**: 厳格なレイヤー

```
┌─────────────────────────────────────────┐
│      Application Layer (SWC)            │
│  ハードウェア非依存のアプリケーション       │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│      RTE (Runtime Environment)          │
│  Virtual Functional Bus実装             │
│  SWC間通信を抽象化                       │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│      Basic Software (BSW)               │
│  ├─ Services Layer                      │
│  ├─ ECU Abstraction Layer               │
│  └─ Microcontroller Abstraction Layer   │
└─────────────────────────────────────────┘
```

**特徴**:
- **VFB (Virtual Functional Bus)**: ハード非依存開発を支援
- **静的構成**: コンパイル時に決定
- **リアルタイム重視**: パワートレイン、シャシー制御向け
- **CAN/FlexRay**: 従来プロトコル

**参考**: [AUTOSAR Classic Platform](https://www.autosar.org/standards/classic-platform)

---

### 5. 自動車（AUTOSAR Adaptive）

**構造**: SOA（サービス指向）

```
┌─────────────────────────────────────────┐
│      Adaptive Applications              │
│  自動運転、インフォテイメント等           │
└─────────────────┬───────────────────────┘
                  │ Service API
┌─────────────────▼───────────────────────┐
│      ARA (AUTOSAR Runtime Adaptive)     │
│  Functional Clusters                    │
│  (Communication, Diagnostics, etc.)     │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│      OS (POSIX PSE51)                   │
│  Ethernet / SOME/IP                     │
└─────────────────────────────────────────┘
```

**特徴**:
- **動的構成**: ランタイムで変更可能
- **OTA更新**: 無線アップデート対応
- **高性能計算**: CPU負荷の高い処理向け
- **Ethernet/SOME-IP**: 高速通信

**参考**: [AUTOSAR Adaptive](https://www.windriver.com/solutions/learning/autosar-adaptive-software-platform)

---

### 6. 組み込み一般

**主要パターン**:

| パターン | 特徴 | 用途 |
|---------|------|-----|
| **Layered** | HAL/BSP → Drivers → Business Logic | 汎用組み込み |
| **Event-Driven** | イベント/メッセージに反応 | 非同期処理 |
| **Message-Passing** | メッセージキュー経由で通信 | 疎結合、分散 |
| **Pipeline** | データを段階的に処理 | 信号処理 |
| **RTOS-Based** | タスクスケジューリング | リアルタイム |

**センサーフュージョンアーキテクチャ**:

| モデル | 特徴 |
|-------|------|
| **JDL Fusion** | 米国防総省標準、レベル0-5の階層 |
| **LAAS** | 機能レベル/決定レベルの分離 |
| **Data-Centric** | 仮想中央データベース |

**参考**: [Embedded Architecture Patterns](https://arjunkalsi13.medium.com/10-most-common-embedded-software-architecture-84769d549017)

---

### 7. Webアプリケーション

**主要パターン**:

| パターン | 核心概念 | レイヤー |
|---------|---------|---------|
| **Hexagonal (Ports & Adapters)** | ビジネスロジックを外部から分離 | Core ← Ports ← Adapters |
| **Clean Architecture** | 依存方向は内向き | Entities → Use Cases → Interface Adapters → Frameworks |
| **DDD (Domain-Driven Design)** | ドメインモデル中心 | Domain → Application → Infrastructure |

**共通原則**:
- **依存性逆転**: 内側は外側に依存しない
- **ポート**: インターフェース（抽象）
- **アダプター**: 実装（具象）

**参考**: [DDD, Hexagonal, Onion, Clean](https://herbertograca.com/2017/11/16/explicit-architecture-01-ddd-hexagonal-onion-clean-cqrs-how-i-put-it-all-together/)

---

## パターン比較表

| 分野 | 代表例 | 構造 | 通信 | 特徴 |
|------|-------|------|------|------|
| ロボット | ROS 2 | レイヤー+Pub/Sub | Topics/Services/Actions | 分散、ミドルウェア抽象化 |
| ドローン | PX4 | モジュール+Pub/Sub | uORB | リアクティブ、共有メモリ |
| ドローン | ArduPilot | HAL+ライブラリ | 関数呼び出し | 移植性、共有コード |
| 自動車 | AUTOSAR CP | 厳格レイヤー | VFB/RTE | 静的、リアルタイム |
| 自動車 | AUTOSAR AP | SOA | SOME/IP | 動的、高性能 |
| 組み込み | - | 様々 | イベント/メッセージ | 用途に応じて選択 |
| Web | Clean/Hex | ポート&アダプター | API/依存注入 | テスタビリティ |

---

## StampFlyへの示唆

### PX4に近いアプローチ

現状のStampFlyはPX4に構造が似ている：
- タスクがモジュール的に分離
- グローバル変数（≒共有メモリ）で通信
- センサー→推定→制御のパイプライン

**PX4を参考にするなら**:
1. **uORB的な仕組み**: トピックベースのPub/Sub
2. **モジュール独立性**: 各機能を交換可能に
3. **明確なデータフロー図**: モジュール間の依存を可視化

### ArduPilotに近いアプローチ

**ArduPilotを参考にするなら**:
1. **HAL強化**: センサードライバをより抽象化
2. **共有ライブラリ**: 推定・制御を再利用可能なコンポーネントに
3. **ボード定義**: ハード構成を外部ファイルで管理

### Clean Architecture的アプローチ

**Webアーキテクチャを参考にするなら**:
1. **ポート/アダプター**: センサーはアダプター、融合ロジックはコア
2. **依存性逆転**: 上位は下位のインターフェースのみ知る
3. **テスタビリティ**: モック注入でユニットテスト容易

---

## 次のステップ

どのアプローチが StampFly の学習目的に最適か検討が必要：

1. **PX4風**: Pub/Sub導入（実装コスト高だが将来性あり）
2. **ArduPilot風**: HAL/ライブラリ強化（段階的に実装可能）
3. **Clean風**: ポート/アダプター導入（テスタビリティ向上）
4. **ハイブリッド**: 各パターンの良い部分を組み合わせ

---

---

# 決定事項と実装計画

## 採用した設計方針

ユーザーの設計ドキュメント（`knote/note_about_software_desigen.md`）に基づき、以下を採用：

### 命名規則

| レイヤー | prefix | 役割 | FreeRTOS依存 |
|---------|--------|------|-------------|
| HAL | `sf_hal_*` | ハードウェアドライバ | 最小限（mutex程度） |
| Algorithm | `sf_algo_*` | 純粋アルゴリズム | **なし** |
| Service | `sf_svc_*` | 状態管理・ロギング等 | あり |
| Task | `sf_task_*` | FreeRTOSタスク | あり |
| App | `sf_app_*` | アプリ固有機能 | あり |

- `stampfly_` プレフィックスは長いため `sf_` に短縮
- 既存コンポーネント（`stampfly_*`）は段階的にリネーム予定

### 設計原則

1. **main.cppは薄く**：構成図（配線図）として読むだけで全体が分かる
2. **FreeRTOS依存の分離**：`algo_*` は純粋アルゴリズム、PCシミュレーション移植可能
3. **一度に全てを変えない**：一つづつmainから抽出、単体テスト・動作確認を繰り返す

---

## 実装計画

### フェーズ1: sf_algo_fusion 土台作り ✅ 完了

**目標**: ESKFをラップするsf_algo_fusionコンポーネントを作成し、main.cppから使用

| # | 作業 | 状態 |
|---|------|------|
| 1 | `sf_algo_fusion` コンポーネント作成 | ✅ 完了 |
| 2 | 各センサー有効/無効スイッチ追加 | ✅ 完了 |
| 3 | main.cppから `sf_algo_fusion` を使用 | ✅ 完了 |
| 4 | ビルド確認 | ✅ 完了 |

**結果**: main.cppのメソッド呼び出しが `g_eskf.xxx()` → `g_fusion.xxx()` に変更。
ただし、main.cppのコード量はほぼ変わらず（土台作りのみ）。

---

### フェーズ1.5: センサー更新ロジックの移動 🔄 進行中

**目標**: main.cpp内のセンサー更新ロジックをsf_algo_fusionに移動し、main.cppを縮小

**完了した作業**:

| センサー | 移動したロジック | main.cpp削減 |
|---------|-----------------|-------------|
| OpticalFlow | squal品質チェック、距離範囲チェック | 17行 → 10行 |
| ToF | 距離範囲チェック (0.01m〜4.0m) | 削除 |
| Baro | (コメントアウト中) | - |
| Mag | チェック不要（元々シンプル） | - |

**変更後のmain.cpp (IMUTask内)**:
```cpp
// OpticalFlow - 品質/距離チェックはSensorFusion内部で実行
g_fusion.updateOpticalFlow(flow_dx, flow_dy, flow_squal, tof_bottom, dt, g.x, g.y);

// ToF - 距離範囲チェックはSensorFusion内部で実行
g_fusion.updateToF(g_tof_data_cache);

// Mag - シンプルな呼び出し（チェック不要）
g_fusion.updateMagnetometer(g_mag_data_cache);
```

| # | 作業 | 状態 |
|---|------|------|
| 1 | 移動するロジックの洗い出し | ✅ 完了 |
| 2 | sf_algo_fusion API設計 | ✅ 完了 |
| 3 | OpticalFlow品質/距離チェック移動 | ✅ 完了 |
| 4 | ToF距離範囲チェック移動 | ✅ 完了 |
| 5 | ビルド確認 | ✅ 完了 |

---

### フェーズ2: センサーヘルスチェック統合

- `sf_svc_health` コンポーネント作成
- 各タスクに分散しているヘルスチェックロジックを統合
- ヘルスステータスの一元管理

---

### フェーズ3: main.cppファイル分割

- `main/config.hpp` - GPIO、優先度、定数
- `main/init.cpp` - 初期化関数群
- `main/tasks/` - タスク関数群（IMUTask, BaroTask等を別ファイルに）

---

### フェーズ4: 既存コンポーネントリネーム

- `stampfly_*` → `sf_<layer>_*` への段階的移行
- 例：`stampfly_eskf` → `sf_algo_eskf`

---

## sf_algo_fusion コンポーネント構成

```
components/sf_algo_fusion/
├── include/
│   └── sensor_fusion.hpp    # SensorFusion クラス定義
├── sensor_fusion.cpp        # 実装
├── CMakeLists.txt
└── README.md
```

### 設計決定

- **選択肢A採用**：`sf_algo_fusion` は既存 `stampfly_eskf` のラッパー（ファサード）
- ESKFをそのまま使うのではなく、センサーフュージョンの「使い方」をカプセル化
- 発散検出・リセット機能を内包
- FreeRTOS非依存（algo層設計原則に準拠）
- 各センサーの有効/無効スイッチ（デフォルト: 全て有効）
