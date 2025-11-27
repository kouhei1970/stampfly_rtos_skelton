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
    // 状態リセット
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

    // 位置
    P_(POS_X, POS_X) = pos_var;
    P_(POS_Y, POS_Y) = pos_var;
    P_(POS_Z, POS_Z) = pos_var;
    // 速度
    P_(VEL_X, VEL_X) = vel_var;
    P_(VEL_Y, VEL_Y) = vel_var;
    P_(VEL_Z, VEL_Z) = vel_var;
    // 姿勢
    P_(ATT_X, ATT_X) = att_var;
    P_(ATT_Y, ATT_Y) = att_var;
    P_(ATT_Z, ATT_Z) = att_var;
    // ジャイロバイアス
    P_(BG_X, BG_X) = bg_var;
    P_(BG_Y, BG_Y) = bg_var;
    P_(BG_Z, BG_Z) = bg_var;
    // 加速度バイアス
    P_(BA_X, BA_X) = ba_var;
    P_(BA_Y, BA_Y) = ba_var;
    P_(BA_Z, BA_Z) = ba_var;
}

void ESKF::predict(const Vector3& accel, const Vector3& gyro, float dt)
{
    if (!initialized_ || dt <= 0) return;

    // バイアス補正
    Vector3 gyro_corrected = gyro - state_.gyro_bias;
    Vector3 accel_corrected = accel - state_.accel_bias;

    // 回転行列
    Matrix<3, 3> R = quaternionToRotationMatrix(state_.orientation);

    // ワールド座標系での加速度（重力除去）
    Vector3 accel_world = toVector3(R * toMatrix(accel_corrected));
    accel_world.z -= config_.gravity;

    // 名目状態の更新
    // 位置: p = p + v*dt + 0.5*a*dt^2
    state_.position += state_.velocity * dt + accel_world * (0.5f * dt * dt);

    // 速度: v = v + a*dt
    state_.velocity += accel_world * dt;

    // 姿勢: q = q ⊗ exp(ω*dt)
    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();

    // オイラー角更新
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // 共分散の予測更新
    // 状態遷移行列 F (離散化)
    Matrix<15, 15> F = Matrix<15, 15>::identity();

    // dp/dv = I*dt
    F(POS_X, VEL_X) = dt;
    F(POS_Y, VEL_Y) = dt;
    F(POS_Z, VEL_Z) = dt;

    // dv/dθ = -R*[a]×*dt
    Matrix<3, 3> skew_a = skewSymmetric(accel_corrected);
    Matrix<3, 3> dv_dtheta = (R * skew_a) * (-dt);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F(VEL_X + i, ATT_X + j) = dv_dtheta(i, j);
        }
    }

    // dv/db_a = -R*dt
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F(VEL_X + i, BA_X + j) = -R(i, j) * dt;
        }
    }

    // dθ/db_g = -I*dt
    F(ATT_X, BG_X) = -dt;
    F(ATT_Y, BG_Y) = -dt;
    F(ATT_Z, BG_Z) = -dt;

    // プロセスノイズ共分散 Q
    Matrix<15, 15> Q = Matrix<15, 15>::zeros();
    float gyro_var = config_.gyro_noise * config_.gyro_noise * dt;
    float accel_var = config_.accel_noise * config_.accel_noise * dt;
    float bg_var = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
    float ba_var = config_.accel_bias_noise * config_.accel_bias_noise * dt;

    // 姿勢ノイズ
    Q(ATT_X, ATT_X) = gyro_var;
    Q(ATT_Y, ATT_Y) = gyro_var;
    Q(ATT_Z, ATT_Z) = gyro_var;

    // 速度ノイズ (回転行列経由、対角成分のみ簡略化)
    for (int i = 0; i < 3; i++) {
        Q(VEL_X + i, VEL_X + i) = accel_var;
    }

    // バイアスノイズ
    Q(BG_X, BG_X) = bg_var;
    Q(BG_Y, BG_Y) = bg_var;
    Q(BG_Z, BG_Z) = bg_var;
    Q(BA_X, BA_X) = ba_var;
    Q(BA_Y, BA_Y) = ba_var;
    Q(BA_Z, BA_Z) = ba_var;

    // P = F * P * F' + Q
    Matrix<15, 15> FP = F * P_;
    Matrix<15, 15> FPFT = FP * F.transpose();
    P_ = FPFT + Q;
}

void ESKF::updateBaro(float altitude)
{
    if (!initialized_) return;

    // 観測モデル: z = p_z
    Matrix<1, 1> z;
    z(0, 0) = altitude;

    Matrix<1, 1> h;
    h(0, 0) = state_.position.z;

    // ヤコビアン H: dh/dx = [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0]
    Matrix<1, 15> H;
    H(0, POS_Z) = 1.0f;

    // 観測ノイズ R
    Matrix<1, 1> R;
    R(0, 0) = config_.baro_noise * config_.baro_noise;

    measurementUpdate<1>(z, h, H, R);
}

void ESKF::updateToF(float distance)
{
    if (!initialized_) return;

    // 姿勢に基づく地上距離への変換
    float cos_roll = std::cos(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float height = distance * cos_roll * cos_pitch;

    // 観測モデル: z = -p_z (NEDなので下向きが正)
    Matrix<1, 1> z;
    z(0, 0) = -height;

    Matrix<1, 1> h;
    h(0, 0) = state_.position.z;

    // ヤコビアン
    Matrix<1, 15> H;
    H(0, POS_Z) = 1.0f;

    // 観測ノイズ
    Matrix<1, 1> R;
    R(0, 0) = config_.tof_noise * config_.tof_noise;

    measurementUpdate<1>(z, h, H, R);
}

void ESKF::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    // 参照ベクトル（NED）をボディ座標系に変換
    Matrix<3, 3> R = quaternionToRotationMatrix(state_.orientation);
    Vector3 mag_expected = toVector3(R.transpose() * toMatrix(config_.mag_ref));

    // 水平面での方位角のみ使用（ヨー補正）
    float yaw_meas = std::atan2(mag.y, mag.x);
    float yaw_pred = std::atan2(mag_expected.y, mag_expected.x);

    // イノベーション（角度差を-π〜πに正規化）
    float yaw_err = yaw_meas - yaw_pred;
    while (yaw_err > M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // 観測更新（1次元）
    Matrix<1, 1> z;
    z(0, 0) = yaw_err;

    Matrix<1, 1> h;
    h(0, 0) = 0.0f;

    // ヤコビアン: ヨーに対する感度
    Matrix<1, 15> H;
    H(0, ATT_Z) = 1.0f;

    Matrix<1, 1> R_mat;
    R_mat(0, 0) = config_.mag_noise * config_.mag_noise;

    measurementUpdate<1>(z, h, H, R_mat);
}

void ESKF::updateFlow(float flow_x, float flow_y, float height)
{
    if (!initialized_ || height < 0.1f) return;

    // オプティカルフローから水平速度を推定
    // v = flow * height (簡略モデル)
    float vx_meas = flow_y * height;  // 座標系変換
    float vy_meas = -flow_x * height;

    // 観測モデル
    Matrix<2, 1> z;
    z(0, 0) = vx_meas;
    z(1, 0) = vy_meas;

    Matrix<2, 1> h;
    h(0, 0) = state_.velocity.x;
    h(1, 0) = state_.velocity.y;

    // ヤコビアン
    Matrix<2, 15> H;
    H(0, VEL_X) = 1.0f;
    H(1, VEL_Y) = 1.0f;

    // 観測ノイズ
    Matrix<2, 2> R;
    R(0, 0) = config_.flow_noise * config_.flow_noise;
    R(1, 1) = config_.flow_noise * config_.flow_noise;

    measurementUpdate<2>(z, h, H, R);
}

void ESKF::injectErrorState(const Matrix<15, 1>& dx)
{
    // 位置
    state_.position.x += dx(POS_X, 0);
    state_.position.y += dx(POS_Y, 0);
    state_.position.z += dx(POS_Z, 0);

    // 速度
    state_.velocity.x += dx(VEL_X, 0);
    state_.velocity.y += dx(VEL_Y, 0);
    state_.velocity.z += dx(VEL_Z, 0);

    // 姿勢（回転ベクトルをクォータニオンに変換して乗算）
    Vector3 dtheta(dx(ATT_X, 0), dx(ATT_Y, 0), dx(ATT_Z, 0));
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();

    // オイラー角更新
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // ジャイロバイアス
    state_.gyro_bias.x += dx(BG_X, 0);
    state_.gyro_bias.y += dx(BG_Y, 0);
    state_.gyro_bias.z += dx(BG_Z, 0);

    // 加速度バイアス
    state_.accel_bias.x += dx(BA_X, 0);
    state_.accel_bias.y += dx(BA_Y, 0);
    state_.accel_bias.z += dx(BA_Z, 0);
}

template<int M>
void ESKF::measurementUpdate(const Matrix<M, 1>& z,
                              const Matrix<M, 1>& h,
                              const Matrix<M, 15>& H,
                              const Matrix<M, M>& R)
{
    // イノベーション
    Matrix<M, 1> y = z - h;

    // イノベーション共分散: S = H * P * H' + R
    Matrix<M, 15> HP = H * P_;
    Matrix<M, M> S = HP * H.transpose() + R;

    // カルマンゲイン: K = P * H' * S^{-1}
    Matrix<15, M> PHT = P_ * H.transpose();
    Matrix<M, M> S_inv = inverse<M>(S);
    Matrix<15, M> K = PHT * S_inv;

    // 状態更新
    Matrix<15, 1> dx = K * y;
    injectErrorState(dx);

    // 共分散更新 (Joseph形式: 数値安定)
    // P = (I - K*H) * P * (I - K*H)' + K * R * K'
    Matrix<15, 15> I_KH = Matrix<15, 15>::identity() - K * H;
    Matrix<15, 15> I_KH_P = I_KH * P_;
    Matrix<15, 15> I_KH_P_I_KHT = I_KH_P * I_KH.transpose();

    // K * R * K' を計算 (K: 15xM, R: MxM, K': Mx15)
    Matrix<15, M> KR = K * R;
    Matrix<15, 15> KRKT = KR * K.transpose();
    P_ = I_KH_P_I_KHT + KRKT;
}

// テンプレートの明示的インスタンス化
template void ESKF::measurementUpdate<1>(const Matrix<1, 1>&, const Matrix<1, 1>&,
                                          const Matrix<1, 15>&, const Matrix<1, 1>&);
template void ESKF::measurementUpdate<2>(const Matrix<2, 1>&, const Matrix<2, 1>&,
                                          const Matrix<2, 15>&, const Matrix<2, 2>&);
template void ESKF::measurementUpdate<3>(const Matrix<3, 1>&, const Matrix<3, 1>&,
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

    // バイアス補正
    Vector3 gyro_corrected = gyro - state_.gyro_bias;

    // ジャイロによる姿勢更新
    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    Quaternion q_gyro = state_.orientation * dq;
    q_gyro.normalize();

    // 加速度による姿勢推定（roll, pitch のみ）
    float accel_norm = accel.norm();
    if (accel_norm > 0.5f && accel_norm < 3.0f * 9.81f) {
        // 加速度から roll, pitch を計算
        float roll_acc = std::atan2(accel.y, accel.z);
        float pitch_acc = std::atan2(-accel.x, std::sqrt(accel.y*accel.y + accel.z*accel.z));

        // ジャイロから roll, pitch を取得
        float roll_gyro, pitch_gyro, yaw_gyro;
        q_gyro.toEuler(roll_gyro, pitch_gyro, yaw_gyro);

        // 相補フィルタ
        float alpha = config_.gyro_weight;
        float roll_fused = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
        float pitch_fused = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

        // フュージョン結果からクォータニオン再構成
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

    // 水平面での方位角計算
    float cos_roll = std::cos(state_.roll);
    float sin_roll = std::sin(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float sin_pitch = std::sin(state_.pitch);

    // 傾き補正
    float mag_x = mag.x * cos_pitch + mag.z * sin_pitch;
    float mag_y = mag.x * sin_roll * sin_pitch + mag.y * cos_roll - mag.z * sin_roll * cos_pitch;

    float yaw_mag = std::atan2(-mag_y, mag_x) + config_.mag_declination;

    // ヨーの相補フィルタ
    float alpha = config_.gyro_weight;
    state_.yaw = alpha * state_.yaw + (1.0f - alpha) * yaw_mag;

    // クォータニオン再構成
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

    // 状態予測
    // x = [altitude, velocity]'
    // x_new = F * x + B * u
    state_.altitude += state_.velocity * dt + 0.5f * accel_z * dt * dt;
    state_.velocity += accel_z * dt;

    // 共分散予測 P = F * P * F' + Q
    float F[2][2] = {{1, dt}, {0, 1}};
    float FP[2][2];
    float FPFt[2][2];

    // FP = F * P
    FP[0][0] = F[0][0] * P_[0][0] + F[0][1] * P_[1][0];
    FP[0][1] = F[0][0] * P_[0][1] + F[0][1] * P_[1][1];
    FP[1][0] = F[1][0] * P_[0][0] + F[1][1] * P_[1][0];
    FP[1][1] = F[1][0] * P_[0][1] + F[1][1] * P_[1][1];

    // FPFt = FP * F'
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

    // 観測モデル: z = H * x, H = [1, 0]
    float y = altitude - state_.altitude;

    // S = H * P * H' + R = P[0][0] + R
    float S = P_[0][0] + config_.measurement_noise_baro;

    // K = P * H' / S
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    // 状態更新
    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    // 共分散更新 P = (I - K*H) * P
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

    // 傾き補正
    float cos_roll = std::cos(roll);
    float cos_pitch = std::cos(pitch);
    float height = distance * cos_roll * cos_pitch;

    // 観測モデル: z = H * x, H = [1, 0]
    float y = height - state_.altitude;

    // S = H * P * H' + R
    float S = P_[0][0] + config_.measurement_noise_tof;

    // K = P * H' / S
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    // 状態更新
    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    // 共分散更新
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

    // ジャイロ補正済みオプティカルフロー
    // フローはカメラの動きと逆方向
    float flow_x_comp = flow_x - gyro_y;  // ピッチ角速度による補正
    float flow_y_comp = flow_y + gyro_x;  // ロール角速度による補正

    // 速度計算 (簡略モデル)
    state_.vx = flow_y_comp * height * config_.flow_scale;
    state_.vy = -flow_x_comp * height * config_.flow_scale;
}

void VelocityEstimator::reset()
{
    state_.vx = 0.0f;
    state_.vy = 0.0f;
}

}  // namespace stampfly
