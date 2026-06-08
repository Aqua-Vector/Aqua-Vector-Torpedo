// eskf_estimator.hpp — 9-state Error-State Kalman Filter
//
// 어뢰 INS의 핵심 추정 알고리즘.
// IMU 측정으로 nominal state 적분 (predict)
// LiDAR 위치 측정으로 error state 보정 (update)
// NHC (옆방향 속도 = 0) 보조 측정 (update_nhc)
//
// 사용 패턴:
//   EskfEstimator est;
//   est.init(p0, q0);
//   while (running) {
//       ImuSample s = imu.read();
//       BiasEstimate bias = bias_est.get();
//       est.predict(s, bias, dt);
//
//       if (lidar_arrived) est.update_lidar(z_lidar);
//       if (motion_state) est.update_nhc();
//   }
//
// Layer: Domain (Layer 3)
// 참조: ADR-005 (위치/속도 추정), ADR-006 (외부 측정 통합)

#pragma once

#include "torpedo/domain/estimator/eskf_state.hpp"
#include "torpedo/domain/estimator/bias_estimate.hpp"
#include "torpedo/sensor/imu_sample.hpp"

#include <Eigen/Dense>

namespace torpedo::domain {

/**
 * ESKF 초기화 파라미터.
 */
struct EskfInitParams {
    Eigen::Vector3f p0 = Eigen::Vector3f::Zero();   // 초기 위치 (m)
    Eigen::Quaternionf q0 = Eigen::Quaternionf::Identity();  // 초기 자세

    // P 초기 분산 (대각)
    float sigma_p = 0.05f;        // 위치 σ (m), LiDAR 정확도
    float sigma_v = 0.01f;        // 속도 σ (m/s), 정지 검증
    float sigma_theta = 0.0017f;  // 자세 σ (rad), 0.1°

    // Q (process noise) — Allan Variance에서 (ADR-001/005)
    float vrw = 5.89e-4f;  // m/s²/√Hz (가속도)
    float arw = 5.82e-5f;  // rad/√s (자이로)
};

/**
 * ESKF measurement noise — LiDAR.
 */
struct LidarMeasurementParams {
    float sigma_lidar = 0.05f;  // ±5cm (ADR-006)
};

/**
 * 9-state ESKF.
 *
 * Thread-safety: 단일 스레드 사용 (메인 루프).
 *                read 측은 atomic snapshot 사용 권장.
 */
class EskfEstimator {
public:
    EskfEstimator() = default;

    // -------- 초기화 --------
    void init(const EskfInitParams& params, float dt);

    // -------- Predict (매 사이클, 100Hz) --------
    void predict(const ImuSample& s,
                 const BiasEstimate& bias,
                 float dt);

    // -------- Update (측정 도착 시) --------
    /// LiDAR 위치 측정 (2D, m 단위)
    void update_lidar(const Eigen::Vector2f& z_lidar);

    /// NHC: body lateral velocity = 0
    void update_nhc();

    // -------- 상태 조회 --------
    const EskfState& state() const { return x_; }

private:
    EskfState x_;
    LidarMeasurementParams lidar_params_;

    // 내부 헬퍼
    void inject_error(const Eigen::Matrix<float, 9, 1>& delta_x);
    static Eigen::Matrix3f skew_symmetric(const Eigen::Vector3f& v);
};

} // namespace torpedo::domain