/**
 * @file eskf.cpp
 * @brief ESKF Implementation
 *
 * Error-State Kalman Filter (ESKF) による統合推定器
 * 15状態: [位置(3), 速度(3), 姿勢誤差(3), ジャイロバイアス(3), 加速度バイアス(3)]
 */

#include "eskf.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "ESKF";

namespace stampfly {

// エラー状態インデックス
enum StateIdx {
    POS_X = 0, POS_Y = 1, POS_Z = 2,
    VEL_X = 3, VEL_Y = 4, VEL_Z = 5,
    ATT_X = 6, ATT_Y = 7, ATT_Z = 8,
    BG_X = 9, BG_Y = 10, BG_Z = 11,
    BA_X = 12, BA_Y = 13, BA_Z = 14
};

// ============================================================================
// ESKF Implementation
// ============================================================================

esp_err_t ESKF::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ESKF (15-state)");
    config_ = config;
    reset();
    initialized_ = true;
    ESP_LOGI(TAG, "ESKF initialized successfully");

    return ESP_OK;
}

void ESKF::reset()
{
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
    state_.orientation = Quaternion::identity();
    state_.gyro_bias = Vector3::zero();
    state_.accel_bias = Vector3::zero();
    state_.roll = state_.pitch = state_.yaw = 0.0f;

    // 共分散行列の初期化
    P_ = Matrix<15, 15>::zeros();
    float pos_var = config_.init_pos_std * config_.init_pos_std;
    float vel_var = config_.init_vel_std * config_.init_vel_std;
    float att_var = config_.init_att_std * config_.init_att_std;
    float bg_var = config_.init_gyro_bias_std * config_.init_gyro_bias_std;
    float ba_var = config_.init_accel_bias_std * config_.init_accel_bias_std;

    P_(POS_X, POS_X) = pos_var;
    P_(POS_Y, POS_Y) = pos_var;
    P_(POS_Z, POS_Z) = pos_var;
    P_(VEL_X, VEL_X) = vel_var;
    P_(VEL_Y, VEL_Y) = vel_var;
    P_(VEL_Z, VEL_Z) = vel_var;
    P_(ATT_X, ATT_X) = att_var;
    P_(ATT_Y, ATT_Y) = att_var;
    P_(ATT_Z, ATT_Z) = att_var;
    P_(BG_X, BG_X) = bg_var;
    P_(BG_Y, BG_Y) = bg_var;
    P_(BG_Z, BG_Z) = bg_var;
    P_(BA_X, BA_X) = ba_var;
    P_(BA_Y, BA_Y) = ba_var;
    P_(BA_Z, BA_Z) = ba_var;
}

void ESKF::setGyroBias(const Vector3& bias)
{
    state_.gyro_bias = bias;
}

void ESKF::setAccelBias(const Vector3& bias)
{
    state_.accel_bias = bias;
}

void ESKF::setMagReference(const Vector3& mag_ref)
{
    config_.mag_ref = mag_ref;
}

void ESKF::setYaw(float yaw)
{
    state_.yaw = yaw;
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, yaw);
}

void ESKF::predict(const Vector3& accel, const Vector3& gyro, float dt)
{
    if (!initialized_ || dt <= 0) return;

    // バイアス補正
    Vector3 gyro_corrected = gyro - state_.gyro_bias;
    Vector3 accel_corrected = accel - state_.accel_bias;

    // mag_enabled=false時はYawレートを0に固定（ドリフト防止）
    if (!config_.mag_enabled) {
        gyro_corrected.z = 0.0f;
    }

    // 回転行列
    Matrix<3, 3> R = quaternionToRotationMatrix(state_.orientation);

    // ワールド座標系での加速度（重力補正）
    // NEDフレーム: g = (0, 0, +9.81)
    Vector3 accel_world = toVector3(R * toMatrix(accel_corrected));
    accel_world.z += config_.gravity;

    // 名目状態の更新
    state_.position += state_.velocity * dt + accel_world * (0.5f * dt * dt);
    state_.velocity += accel_world * dt;

    // 姿勢: q = q ⊗ exp(ω*dt)
    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // mag_enabled=false時はYaw=0に固定
    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    // 共分散の予測更新
    F_ = Matrix<15, 15>::identity();

    // dp/dv = I*dt
    F_(POS_X, VEL_X) = dt;
    F_(POS_Y, VEL_Y) = dt;
    F_(POS_Z, VEL_Z) = dt;

    // dv/dθ = -R*[a]×*dt
    Matrix<3, 3> skew_a = skewSymmetric(accel_corrected);
    Matrix<3, 3> dv_dtheta = (R * skew_a) * (-dt);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_(VEL_X + i, ATT_X + j) = dv_dtheta(i, j);
        }
    }

    // dv/db_a = -R*dt
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_(VEL_X + i, BA_X + j) = -R(i, j) * dt;
        }
    }

    // dθ/db_g = -I*dt
    F_(ATT_X, BG_X) = -dt;
    F_(ATT_Y, BG_Y) = -dt;
    F_(ATT_Z, BG_Z) = -dt;

    // プロセスノイズ共分散 Q
    Q_ = Matrix<15, 15>::zeros();
    float gyro_var = config_.gyro_noise * config_.gyro_noise * dt;
    float accel_var = config_.accel_noise * config_.accel_noise * dt;
    float bg_var = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
    float ba_var = config_.accel_bias_noise * config_.accel_bias_noise * dt;

    Q_(ATT_X, ATT_X) = gyro_var;
    Q_(ATT_Y, ATT_Y) = gyro_var;
    Q_(ATT_Z, ATT_Z) = gyro_var;

    for (int i = 0; i < 3; i++) {
        Q_(VEL_X + i, VEL_X + i) = accel_var;
    }

    Q_(BG_X, BG_X) = bg_var;
    Q_(BG_Y, BG_Y) = bg_var;
    // mag_enabled=false時はGyro Bias Zの推定を抑制
    Q_(BG_Z, BG_Z) = config_.mag_enabled ? bg_var : 0.0f;
    Q_(BA_X, BA_X) = ba_var;
    Q_(BA_Y, BA_Y) = ba_var;
    Q_(BA_Z, BA_Z) = ba_var;

    // P = F * P * F' + Q
    temp1_ = F_ * P_;
    temp2_ = temp1_ * F_.transpose();
    P_ = temp2_ + Q_;
}

void ESKF::updateBaro(float altitude)
{
    if (!initialized_) return;

    // NED変換: 上昇=マイナス
    Matrix<1, 1> z;
    z(0, 0) = -altitude;

    Matrix<1, 1> h;
    h(0, 0) = state_.position.z;

    Matrix<1, 15> H;
    H(0, POS_Z) = 1.0f;

    Matrix<1, 1> R;
    R(0, 0) = config_.baro_noise * config_.baro_noise;

    measurementUpdate<1>(z, h, H, R);
}

void ESKF::updateToF(float distance)
{
    if (!initialized_) return;

    // 傾きが大きい場合はスキップ
    float tilt = std::sqrt(state_.roll * state_.roll + state_.pitch * state_.pitch);
    if (tilt > config_.tof_tilt_threshold) {
        return;
    }

    // 姿勢に基づく地上距離への変換
    float cos_roll = std::cos(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float height = distance * cos_roll * cos_pitch;

    Matrix<1, 1> z;
    z(0, 0) = -height;

    Matrix<1, 1> h;
    h(0, 0) = state_.position.z;

    Matrix<1, 15> H;
    H(0, POS_Z) = 1.0f;

    Matrix<1, 1> R;
    R(0, 0) = config_.tof_noise * config_.tof_noise;

    measurementUpdate<1>(z, h, H, R);
}

void ESKF::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    // 地磁気ベクトルのノルムチェック（異常値除去）
    float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
    if (mag_norm < 10.0f || mag_norm > 100.0f) {
        return;  // 異常な地磁気読み取り
    }

    // 現在の姿勢から期待される地磁気ベクトル（ボディ座標系）
    // mag_ref はNED座標系での参照地磁気ベクトル（初期化時に取得）
    // mag_expected = R^T * mag_ref (NEDからボディへ変換)
    Matrix<3, 3> R = quaternionToRotationMatrix(state_.orientation);
    Matrix<3, 1> mag_ref_mat = toMatrix(config_.mag_ref);
    Matrix<3, 1> mag_expected_mat = R.transpose() * mag_ref_mat;

    // 観測: 地磁気ベクトル全3成分
    // ピッチが90度に近い場合もZ成分が必要
    Matrix<3, 1> z;
    z(0, 0) = mag.x;
    z(1, 0) = mag.y;
    z(2, 0) = mag.z;

    Matrix<3, 1> h;
    h(0, 0) = mag_expected_mat(0, 0);
    h(1, 0) = mag_expected_mat(1, 0);
    h(2, 0) = mag_expected_mat(2, 0);

    // ヤコビアン: ∂h/∂(roll, pitch, yaw)
    // h = R^T * mag_ref
    // ∂h/∂θ = -[h]× (ESKFの誤差状態での微分)

    // R^T の各列を取得
    float r00 = R(0,0), r01 = R(0,1), r02 = R(0,2);
    float r10 = R(1,0), r11 = R(1,1), r12 = R(1,2);
    float r20 = R(2,0), r21 = R(2,1), r22 = R(2,2);

    // mag_expected = R^T * mag_ref
    float hx = r00*config_.mag_ref.x + r10*config_.mag_ref.y + r20*config_.mag_ref.z;
    float hy = r01*config_.mag_ref.x + r11*config_.mag_ref.y + r21*config_.mag_ref.z;
    float hz = r02*config_.mag_ref.x + r12*config_.mag_ref.y + r22*config_.mag_ref.z;

    // ∂h/∂θ = -[h]×
    // ∂h/∂roll:  [0, hz, -hy]^T
    // ∂h/∂pitch: [-hz, 0, hx]^T
    // ∂h/∂yaw:   [hy, -hx, 0]^T

    Matrix<3, 15> H;
    // Roll (ATT_X)
    H(0, ATT_X) = 0.0f;
    H(1, ATT_X) = hz;
    H(2, ATT_X) = -hy;
    // Pitch (ATT_Y)
    H(0, ATT_Y) = -hz;
    H(1, ATT_Y) = 0.0f;
    H(2, ATT_Y) = hx;
    // Yaw (ATT_Z)
    H(0, ATT_Z) = hy;
    H(1, ATT_Z) = -hx;
    H(2, ATT_Z) = 0.0f;

    // 観測ノイズ
    Matrix<3, 3> R_mat;
    float noise_sq = config_.mag_noise * config_.mag_noise;
    R_mat(0, 0) = noise_sq;
    R_mat(1, 1) = noise_sq;
    R_mat(2, 2) = noise_sq;

    measurementUpdate<3>(z, h, H, R_mat);
}

void ESKF::updateFlow(float flow_x, float flow_y, float height)
{
    updateFlowWithGyro(flow_x, flow_y, height, 0.0f, 0.0f);
}

void ESKF::updateFlowWithGyro(float flow_x, float flow_y, float height,
                               float gyro_x, float gyro_y)
{
    if (!initialized_ || height < config_.flow_min_height) return;

    // ジャイロバイアス補正（predict()と同様）
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;

    // ジャイロ補償係数（回帰分析で取得）
    constexpr float flow_scale = 0.23f;  // 実測キャリブレーション (2024-12-02)
    constexpr float k_xx = 1.35f * flow_scale;
    constexpr float k_xy = 9.30f * flow_scale;
    constexpr float k_yx = -2.65f * flow_scale;
    constexpr float k_yy = 0.0f * flow_scale;

    // フローオフセット補正（動的検討時に再評価）
    constexpr float flow_dx_offset = 0.0f;
    constexpr float flow_dy_offset = 0.0f;

    float flow_x_comp = flow_x - k_xx * gyro_x_corrected - k_xy * gyro_y_corrected - flow_dx_offset;
    float flow_y_comp = flow_y - k_yx * gyro_x_corrected - k_yy * gyro_y_corrected - flow_dy_offset;

    // ボディ座標系での速度
    // フローセンサー軸マッピング（実測から）:
    //   右移動(+Y) → flow_dy > 0
    //   前方移動(+X) → flow_dx > 0
    float vx_body = flow_x_comp * height;
    float vy_body = flow_y_comp * height;

    // Body→NED変換
    float cos_yaw = std::cos(state_.yaw);
    float sin_yaw = std::sin(state_.yaw);

    float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;
    float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;

    // カルマンフィルタ観測更新
    Matrix<2, 1> z;
    z(0, 0) = vx_ned;
    z(1, 0) = vy_ned;

    Matrix<2, 1> h;
    h(0, 0) = state_.velocity.x;
    h(1, 0) = state_.velocity.y;

    Matrix<2, 15> H;
    H(0, VEL_X) = 1.0f;
    H(1, VEL_Y) = 1.0f;

    Matrix<2, 2> R;
    R(0, 0) = config_.flow_noise * config_.flow_noise;
    R(1, 1) = config_.flow_noise * config_.flow_noise;

    measurementUpdate<2>(z, h, H, R);
}

void ESKF::updateFlowRaw(int16_t flow_dx, int16_t flow_dy, float distance,
                          float dt, float gyro_x, float gyro_y)
{
    if (!initialized_ || distance < config_.flow_min_height || dt <= 0) return;

    // ================================================================
    // 1. ピクセル変化 → ピクセル角速度 [rad/s]
    // ================================================================
    // PMW3901: FOV=42°, 35pixels → 0.0209 rad/pixel
    float flow_x_cam = static_cast<float>(flow_dx) * config_.flow_rad_per_pixel / dt;
    float flow_y_cam = static_cast<float>(flow_dy) * config_.flow_rad_per_pixel / dt;

    // ================================================================
    // 2. カメラ座標系 → 機体座標系
    // ================================================================
    // [flow_body_x]   [c2b_xx c2b_xy] [flow_cam_x]
    // [flow_body_y] = [c2b_yx c2b_yy] [flow_cam_y]
    float flow_x_body = config_.flow_cam_to_body[0] * flow_x_cam
                      + config_.flow_cam_to_body[1] * flow_y_cam;
    float flow_y_body = config_.flow_cam_to_body[2] * flow_x_cam
                      + config_.flow_cam_to_body[3] * flow_y_cam;

    // ================================================================
    // 3. 回転成分除去 → 並進由来の角速度
    // ================================================================
    // ジャイロバイアス補正（predict()と同様）
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;

    // 機体角速度 → カメラ角速度（カメラは下向き固定）
    // 機体がroll方向に回転 → カメラはY軸周りに回転 → flow_x変化
    // 機体がpitch方向に回転 → カメラはX軸周りに回転 → flow_y変化
    // flow_rotation_x = gyro_y (pitch回転がflow_xに影響)
    // flow_rotation_y = -gyro_x (roll回転がflow_yに影響、符号反転)
    float flow_rotation_x = gyro_y_corrected;   // pitch → flow_x
    float flow_rotation_y = -gyro_x_corrected;  // roll → flow_y (符号反転)

    float flow_trans_x = flow_x_body - flow_rotation_x;
    float flow_trans_y = flow_y_body - flow_rotation_y;

    // ================================================================
    // 4. 並進速度算出 [m/s]
    // ================================================================
    // v = ω × distance
    float vx_body = flow_trans_x * distance;
    float vy_body = flow_trans_y * distance;

    // ================================================================
    // 5. Body→NED変換
    // ================================================================
    float cos_yaw = std::cos(state_.yaw);
    float sin_yaw = std::sin(state_.yaw);

    float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;
    float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;

    // ================================================================
    // 6. カルマンフィルタ観測更新
    // ================================================================
    Matrix<2, 1> z;
    z(0, 0) = vx_ned;
    z(1, 0) = vy_ned;

    Matrix<2, 1> h;
    h(0, 0) = state_.velocity.x;
    h(1, 0) = state_.velocity.y;

    Matrix<2, 15> H;
    H(0, VEL_X) = 1.0f;
    H(1, VEL_Y) = 1.0f;

    Matrix<2, 2> R;
    R(0, 0) = config_.flow_noise * config_.flow_noise;
    R(1, 1) = config_.flow_noise * config_.flow_noise;

    measurementUpdate<2>(z, h, H, R);
}

void ESKF::updateAccelAttitude(const Vector3& accel)
{
    if (!initialized_) return;

    // 加速度ノルムをチェック（垂直方向の大きな加速を検出）
    float accel_norm = std::sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    float gravity_diff = std::abs(accel_norm - config_.gravity);

    if (gravity_diff > config_.accel_motion_threshold) {
        return;
    }

    // 期待される加速度計出力
    Matrix<3, 3> R = quaternionToRotationMatrix(state_.orientation);
    Vector3 neg_g_world(0.0f, 0.0f, -config_.gravity);
    Vector3 g_expected = toVector3(R.transpose() * toMatrix(neg_g_world));

    // Roll/Pitchのみ補正
    Matrix<2, 1> z;
    z(0, 0) = accel.x;
    z(1, 0) = accel.y;

    Matrix<2, 1> h;
    h(0, 0) = g_expected.x;
    h(1, 0) = g_expected.y;

    Matrix<2, 15> H;
    H(0, ATT_Y) = config_.gravity;
    H(1, ATT_X) = -config_.gravity;

    // Adaptive R: 水平加速度が大きいほど観測ノイズを増加
    // 水平方向の動的加速度があると、それを傾きと誤認するため
    float horiz_accel_sq = accel.x * accel.x + accel.y * accel.y;
    constexpr float k_adaptive = 100.0f;  // 調整パラメータ
    float R_scale = 1.0f + k_adaptive * horiz_accel_sq;

    // 観測ノイズ（動的にスケーリング）
    Matrix<2, 2> R_mat;
    float base_noise_sq = config_.accel_att_noise * config_.accel_att_noise;
    R_mat(0, 0) = base_noise_sq * R_scale;
    R_mat(1, 1) = base_noise_sq * R_scale;

    measurementUpdate<2>(z, h, H, R_mat);
}

void ESKF::injectErrorState(const Matrix<15, 1>& dx)
{
    state_.position.x += dx(POS_X, 0);
    state_.position.y += dx(POS_Y, 0);
    state_.position.z += dx(POS_Z, 0);

    state_.velocity.x += dx(VEL_X, 0);
    state_.velocity.y += dx(VEL_Y, 0);
    state_.velocity.z += dx(VEL_Z, 0);

    Vector3 dtheta(dx(ATT_X, 0), dx(ATT_Y, 0), dx(ATT_Z, 0));
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // mag_enabled=false時はYaw=0に固定
    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx(BG_X, 0);
    state_.gyro_bias.y += dx(BG_Y, 0);
    // mag_enabled=false時はGyro Bias Zを更新しない
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx(BG_Z, 0);
    }

    state_.accel_bias.x += dx(BA_X, 0);
    state_.accel_bias.y += dx(BA_Y, 0);
    state_.accel_bias.z += dx(BA_Z, 0);
}

template<int M>
bool ESKF::measurementUpdate(const Matrix<M, 1>& z,
                              const Matrix<M, 1>& h,
                              const Matrix<M, 15>& H,
                              const Matrix<M, M>& R)
{
    Matrix<M, 1> y = z - h;

    Matrix<M, 15> HP = H * P_;
    Matrix<M, M> S = HP * H.transpose() + R;

    Matrix<15, M> PHT = P_ * H.transpose();
    Matrix<M, M> S_inv = inverse<M>(S);
    Matrix<15, M> K = PHT * S_inv;

    Matrix<15, 1> dx = K * y;
    injectErrorState(dx);

    // Joseph形式共分散更新
    temp1_ = Matrix<15, 15>::identity() - K * H;
    temp2_ = temp1_ * P_;
    F_ = temp2_ * temp1_.transpose();

    Matrix<15, M> KR = K * R;
    Q_ = KR * K.transpose();
    P_ = F_ + Q_;

    return true;
}

// テンプレートの明示的インスタンス化
template bool ESKF::measurementUpdate<1>(const Matrix<1, 1>&, const Matrix<1, 1>&,
                                          const Matrix<1, 15>&, const Matrix<1, 1>&);
template bool ESKF::measurementUpdate<2>(const Matrix<2, 1>&, const Matrix<2, 1>&,
                                          const Matrix<2, 15>&, const Matrix<2, 2>&);
template bool ESKF::measurementUpdate<3>(const Matrix<3, 1>&, const Matrix<3, 1>&,
                                          const Matrix<3, 15>&, const Matrix<3, 3>&);

// ============================================================================
// AttitudeEstimator Implementation (Complementary Filter)
// ============================================================================

esp_err_t AttitudeEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AttitudeEstimator (Complementary Filter)");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void AttitudeEstimator::update(const Vector3& accel, const Vector3& gyro, float dt)
{
    if (!initialized_ || dt <= 0) return;

    Vector3 gyro_corrected = gyro - state_.gyro_bias;

    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    Quaternion q_gyro = state_.orientation * dq;
    q_gyro.normalize();

    float accel_norm = accel.norm();
    if (accel_norm > 0.5f && accel_norm < 3.0f * 9.81f) {
        float roll_acc = std::atan2(accel.y, accel.z);
        float pitch_acc = std::atan2(-accel.x, std::sqrt(accel.y*accel.y + accel.z*accel.z));

        float roll_gyro, pitch_gyro, yaw_gyro;
        q_gyro.toEuler(roll_gyro, pitch_gyro, yaw_gyro);

        float alpha = config_.gyro_weight;
        float roll_fused = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
        float pitch_fused = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

        state_.orientation = Quaternion::fromEuler(roll_fused, pitch_fused, yaw_gyro);
    } else {
        state_.orientation = q_gyro;
    }

    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);
}

void AttitudeEstimator::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    float cos_roll = std::cos(state_.roll);
    float sin_roll = std::sin(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float sin_pitch = std::sin(state_.pitch);

    float mag_x = mag.x * cos_pitch + mag.z * sin_pitch;
    float mag_y = mag.x * sin_roll * sin_pitch + mag.y * cos_roll - mag.z * sin_roll * cos_pitch;

    float yaw_mag = std::atan2(-mag_y, mag_x) + config_.mag_declination;

    float alpha = config_.gyro_weight;
    state_.yaw = alpha * state_.yaw + (1.0f - alpha) * yaw_mag;

    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, state_.yaw);
}

void AttitudeEstimator::reset()
{
    state_.orientation = Quaternion::identity();
    state_.roll = state_.pitch = state_.yaw = 0.0f;
    state_.gyro_bias = Vector3::zero();
}

// ============================================================================
// AltitudeEstimator Implementation (Simple Kalman Filter)
// ============================================================================

esp_err_t AltitudeEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AltitudeEstimator (Kalman Filter)");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void AltitudeEstimator::predict(float accel_z, float dt)
{
    if (!initialized_ || dt <= 0) return;

    state_.altitude += state_.velocity * dt + 0.5f * accel_z * dt * dt;
    state_.velocity += accel_z * dt;

    float F[2][2] = {{1, dt}, {0, 1}};
    float FP[2][2];
    float FPFt[2][2];

    FP[0][0] = F[0][0] * P_[0][0] + F[0][1] * P_[1][0];
    FP[0][1] = F[0][0] * P_[0][1] + F[0][1] * P_[1][1];
    FP[1][0] = F[1][0] * P_[0][0] + F[1][1] * P_[1][0];
    FP[1][1] = F[1][0] * P_[0][1] + F[1][1] * P_[1][1];

    FPFt[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[0][1];
    FPFt[0][1] = FP[0][0] * F[1][0] + FP[0][1] * F[1][1];
    FPFt[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[0][1];
    FPFt[1][1] = FP[1][0] * F[1][0] + FP[1][1] * F[1][1];

    P_[0][0] = FPFt[0][0] + config_.process_noise_alt;
    P_[0][1] = FPFt[0][1];
    P_[1][0] = FPFt[1][0];
    P_[1][1] = FPFt[1][1] + config_.process_noise_vel;
}

void AltitudeEstimator::updateBaro(float altitude)
{
    if (!initialized_) return;

    float y = altitude - state_.altitude;
    float S = P_[0][0] + config_.measurement_noise_baro;
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    float P_new[2][2];
    P_new[0][0] = I_KH[0][0] * P_[0][0] + I_KH[0][1] * P_[1][0];
    P_new[0][1] = I_KH[0][0] * P_[0][1] + I_KH[0][1] * P_[1][1];
    P_new[1][0] = I_KH[1][0] * P_[0][0] + I_KH[1][1] * P_[1][0];
    P_new[1][1] = I_KH[1][0] * P_[0][1] + I_KH[1][1] * P_[1][1];

    P_[0][0] = P_new[0][0];
    P_[0][1] = P_new[0][1];
    P_[1][0] = P_new[1][0];
    P_[1][1] = P_new[1][1];
}

void AltitudeEstimator::updateToF(float distance, float pitch, float roll)
{
    if (!initialized_) return;

    float cos_roll = std::cos(roll);
    float cos_pitch = std::cos(pitch);
    float height = distance * cos_roll * cos_pitch;

    float y = height - state_.altitude;
    float S = P_[0][0] + config_.measurement_noise_tof;
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    float P_new[2][2];
    P_new[0][0] = I_KH[0][0] * P_[0][0] + I_KH[0][1] * P_[1][0];
    P_new[0][1] = I_KH[0][0] * P_[0][1] + I_KH[0][1] * P_[1][1];
    P_new[1][0] = I_KH[1][0] * P_[0][0] + I_KH[1][1] * P_[1][0];
    P_new[1][1] = I_KH[1][0] * P_[0][1] + I_KH[1][1] * P_[1][1];

    P_[0][0] = P_new[0][0];
    P_[0][1] = P_new[0][1];
    P_[1][0] = P_new[1][0];
    P_[1][1] = P_new[1][1];
}

void AltitudeEstimator::reset()
{
    state_.altitude = 0.0f;
    state_.velocity = 0.0f;
    P_[0][0] = 1.0f; P_[0][1] = 0.0f;
    P_[1][0] = 0.0f; P_[1][1] = 1.0f;
}

// ============================================================================
// VelocityEstimator Implementation
// ============================================================================

esp_err_t VelocityEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing VelocityEstimator");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void VelocityEstimator::updateFlow(float flow_x, float flow_y, float height, float gyro_x, float gyro_y)
{
    if (!initialized_ || height < 0.1f) return;

    // ジャイロバイアス補正
    float gyro_x_corrected = gyro_x - gyro_bias_x_;
    float gyro_y_corrected = gyro_y - gyro_bias_y_;

    float flow_x_comp = flow_x - gyro_y_corrected;
    float flow_y_comp = flow_y + gyro_x_corrected;

    state_.vx = flow_y_comp * height * config_.flow_scale;
    state_.vy = -flow_x_comp * height * config_.flow_scale;
}

void VelocityEstimator::reset()
{
    state_.vx = 0.0f;
    state_.vy = 0.0f;
}

}  // namespace stampfly
