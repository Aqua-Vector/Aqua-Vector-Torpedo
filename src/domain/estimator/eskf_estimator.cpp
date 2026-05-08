// eskf_estimator.cpp — ESKF 구현 (Step C에서 한 부분씩 채움)

#include "torpedo/domain/estimator/eskf_estimator.hpp"

namespace torpedo::domain {

void EskfEstimator::init(const EskfInitParams& params, float dt) {
    // TODO: Step C-1
    x_.reset();
    x_.p = params.p0;
    x_.q = params.q0;
    
    // P 초기값 (대각)
    x_.P.setZero();
    x_.P.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * params.sigma_p * params.sigma_p;
    x_.P.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * params.sigma_v * params.sigma_v;
    x_.P.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * params.sigma_theta * params.sigma_theta;
    
    // Q (이산화)
    x_.Q.setZero();
    float q_v = params.vrw * params.vrw * dt;       // m²/s²
    float q_g = params.arw * params.arw * dt;       // rad²
    x_.Q.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * q_v;
    x_.Q.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * q_g;
}

void EskfEstimator::predict(const ImuSample& s,
                            const BiasEstimate& bias,
                            float dt) {
    // TODO
    // ---Bias 보정---
    // IMU 측정값에서 bias 빼서 진짜 가속도/각속도 추출
    Eigen::Vector3f a_corr(s.ax - bias.b_a.x(),
                           s.ay - bias.b_a.y(),
                           s.az - bias.b_a.z());
    Eigen::Vector3f w_corr(s.gx - bias.b_g.x(),
                           s.gy - bias.b_g.y(),
                           s.gz - bias.b_g.z());

    // ---F 행렬 매 사이클 계산---
    // F는 9×9, 3개 의미 있는 3×3 블록만 채움
    
    // body→nav 회전 행렬
    Eigen::Matrix3f R = x_.q.toRotationMatrix();
    
    // cross product 행렬 [v]×
    Eigen::Matrix3f a_skew = skew_symmetric(a_corr);
    Eigen::Matrix3f w_skew = skew_symmetric(w_corr);
    
    // F 행렬 구성
    Eigen::Matrix<float, 9, 9> F = Eigen::Matrix<float, 9, 9>::Zero();
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();    // δp ← δv
    F.block<3, 3>(3, 6) = -R * a_skew;                    // δv ← δθ
    F.block<3, 3>(6, 6) = -w_skew;                        // δθ ← δθ
    
    // ---P 행렬 진행---
    // Φ = I + F·dt (1차 근사 이산화)
    Eigen::Matrix<float, 9, 9> Phi 
        = Eigen::Matrix<float, 9, 9>::Identity() + F * dt;
    
    // P[k+1] = Φ·P·Φᵀ + Q
    x_.P = Phi * x_.P * Phi.transpose() + x_.Q;

    // ---Nominal 위치/속도 적분---
    // body 가속도 → nav 가속도 (회전 + 중력 보정)
    Eigen::Vector3f g_nav(0.0f, 0.0f, 9.81f);     // 중력 (nav frame, +z 위)
    Eigen::Vector3f a_nav = R * a_corr - g_nav;
    
    // 오일러 적분
    x_.p += x_.v * dt + 0.5f * a_nav * dt * dt;
    x_.v += a_nav * dt;

    // ---Nominal 자세 quaternion 적분---
    // small angle quaternion: Δq ≈ [1, θ/2]
    Eigen::Vector3f dtheta = w_corr * dt;
    
    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.x() = 0.5f * dtheta.x();
    dq.y() = 0.5f * dtheta.y();
    dq.z() = 0.5f * dtheta.z();
    
    // q[k+1] = q[k] ⊗ Δq
    x_.q = x_.q * dq;
    
    // 정규화 (수치 안정화 — quaternion drift 방지)
    x_.q.normalize();
}

void EskfEstimator::update_lidar(const Eigen::Vector2f& z_lidar) {
    // TODO
    (void)z_lidar;
}

void EskfEstimator::update_nhc() {
    // TODO
}

void EskfEstimator::inject_error(const Eigen::Matrix<float, 9, 1>& delta_x) {
    // TODO
    (void)delta_x;
}

Eigen::Matrix3f EskfEstimator::skew_symmetric(const Eigen::Vector3f& v) {
    Eigen::Matrix3f S;
    S <<     0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return S;
}

} // namespace torpedo::domain