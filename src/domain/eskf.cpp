#include "ebimu/domain/eskf.hpp"
#include <cmath>

namespace ebimu {

using Vec6 = Eigen::Matrix<float, 6, 1>;
using Mat6 = Eigen::Matrix<float, 6, 6>;

Eskf::Eskf() {
    x_.setZero();
    P_.setIdentity();
    P_ *= 0.01f;
}

void Eskf::init(float px, float py, float pz,
                float vx, float vy, float vz) {
    x_ << px, py, pz, vx, vy, vz;
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 0.01f;
    P_.block<3, 3>(3, 3) *= 0.01f;
    initialized_ = true;
}

void Eskf::predict(const ImuSample& s, const BiasCalibration& bias, float dt) {
    if (!initialized_) return;
    if (dt <= 0.0f || dt > 1.0f) return;
    
    // q_rel = q_start⁻¹ × q_abs
    Eigen::Quaternionf q_abs(s.qw, s.qx, s.qy, s.qz);
    q_abs.normalize();
    Eigen::Quaternionf q_start(bias.q_start_w, bias.q_start_x,
                                bias.q_start_y, bias.q_start_z);
    q_start.normalize();
    Eigen::Quaternionf q_rel = q_start.conjugate() * q_abs;
    Eigen::Matrix3f R = q_rel.toRotationMatrix();
    
    Eigen::Vector3f a_body(s.ax, s.ay, s.az);
    Eigen::Vector3f a_nav = R * a_body;
    
    a_nav(0) -= bias.bx;
    a_nav(1) -= bias.by;
    a_nav(2) -= bias.bz;
    a_nav(2) -= GRAVITY;
    
    Eigen::Vector3f p = x_.head<3>();
    Eigen::Vector3f v = x_.segment<3>(3);
    
    x_.head<3>()     = p + v * dt + 0.5f * a_nav * dt * dt;
    x_.segment<3>(3) = v + a_nav * dt;
    
    Mat6 F = Mat6::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;
    
    float sigma_a_sq = ACC_NOISE_STD * ACC_NOISE_STD;
    Mat6 Q = Mat6::Zero();
    float q_pos = 0.25f * dt * dt * dt * dt * sigma_a_sq;
    float q_vel = dt * dt * sigma_a_sq;
    float q_cross = 0.5f * dt * dt * dt * sigma_a_sq;
    
    for (int i = 0; i < 3; i++) {
        Q(i, i)     = q_pos;
        Q(i+3, i+3) = q_vel;
        Q(i, i+3)   = q_cross;
        Q(i+3, i)   = q_cross;
    }
    
    P_ = F * P_ * F.transpose() + Q;
}

bool Eskf::update_lidar(float px_meas, float py_meas) {
    if (!initialized_) return false;
    if (std::isnan(px_meas) || std::isnan(py_meas)) return false;
    
    Eigen::Vector2f z(px_meas, py_meas);
    
    Eigen::Matrix<float, 2, 6> H = Eigen::Matrix<float, 2, 6>::Zero();
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    
    float sigma_l_sq = LIDAR_NOISE_STD * LIDAR_NOISE_STD;
    Eigen::Matrix2f R_lidar;
    R_lidar << sigma_l_sq, 0.0f,
               0.0f,       sigma_l_sq;
    
    Eigen::Vector2f y = z - H * x_;
    Eigen::Matrix2f S = H * P_ * H.transpose() + R_lidar;
    Eigen::Matrix<float, 6, 2> K = P_ * H.transpose() * S.inverse();
    
    x_ = x_ + K * y;
    
    Mat6 I_KH = Mat6::Identity() - K * H;
    P_ = I_KH * P_;
    
    return true;
}

bool Eskf::update_zupt() {
    if (!initialized_) return false;
    
    // ZUPT: 속도 = 0 measurement
    // z = (0, 0, 0)^T
    // H = [0 0 0 1 0 0]
    //     [0 0 0 0 1 0]
    //     [0 0 0 0 0 1]   (속도만 측정)
    
    Eigen::Matrix<float, 3, 6> H = Eigen::Matrix<float, 3, 6>::Zero();
    H(0, 3) = 1.0f;
    H(1, 4) = 1.0f;
    H(2, 5) = 1.0f;
    
    // R_zupt: 1cm/s 정확도 가정
    const float sigma_v = 0.01f;
    const float sigma_v_sq = sigma_v * sigma_v;
    Eigen::Matrix3f R_zupt = sigma_v_sq * Eigen::Matrix3f::Identity();
    
    // Innovation: y = 0 - H*x = -v
    Eigen::Vector3f v_current = x_.segment<3>(3);
    Eigen::Vector3f y = -v_current;
    
    // S = H*P*H^T + R
    Eigen::Matrix3f S = H * P_ * H.transpose() + R_zupt;
    
    // K = P*H^T*S^-1
    Eigen::Matrix<float, 6, 3> K = P_ * H.transpose() * S.inverse();
    
    // State update
    x_ = x_ + K * y;
    
    // Covariance update
    Mat6 I_KH = Mat6::Identity() - K * H;
    P_ = I_KH * P_;
    
    return true;
}

EskfState Eskf::get_state() const {
    EskfState s;
    s.px = x_(0); s.py = x_(1); s.pz = x_(2);
    s.vx = x_(3); s.vy = x_(4); s.vz = x_(5);
    return s;
}

}  // namespace ebimu
