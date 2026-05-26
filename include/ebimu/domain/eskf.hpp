// eskf.hpp — 6-state ESKF (Error-State Kalman Filter)
//
// State: [px, py, pz, vx, vy, vz]^T  (위치 + 속도, nav frame)
// 자세: EBIMU quaternion 그대로 사용 (state 아님)
//
// 사용:
//   Eskf eskf;
//   eskf.init(...);
//   while (running) {
//     ImuSample s = imu.read();
//     eskf.predict(s, bias, dt);
//     if (lidar_ok) eskf.update_lidar(px, py);
//     State x = eskf.get_state();
//   }

#pragma once

#include "ebimu/sensor/imu_sample.hpp"
#include "ebimu/domain/bias_calibrator.hpp"

#include <Eigen/Dense>

namespace ebimu {

/// 외부 노출용 state 구조 (Eigen 미사용).
struct EskfState {
    float px = 0.0f, py = 0.0f, pz = 0.0f;  // 위치 (m)
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;  // 속도 (m/s)
};

class Eskf {
public:
    // ── 노이즈 파라미터 (실측 1시간 측정 기반) ──
    static constexpr float ACC_NOISE_STD  = 0.013f;   // m/s² (실측 ~1.3mg)
    static constexpr float LIDAR_NOISE_STD = 0.05f;   // m (5cm)
    static constexpr float GRAVITY        = 9.80665f; // m/s²
    
    Eskf();
    
    /// 초기화 — 초기 위치/속도 설정.
    /// 일반적으로 시작 위치 (0,0,0) + 정지 (0,0,0).
    void init(float px = 0.0f, float py = 0.0f, float pz = 0.0f,
              float vx = 0.0f, float vy = 0.0f, float vz = 0.0f);
    
    /// Predict step (100Hz).
    /// @param s     IMU 샘플 (가속도, 자이로, quaternion)
    /// @param bias  BiasCalibrator 결과 (가속도 bias 제거용)
    /// @param dt    시간 간격 (초). 일반적으로 0.01s.
    void predict(const ImuSample& s, const BiasCalibration& bias, float dt);
    
    /// LiDAR position update (2D).
    /// @return 성공 여부 (NaN이면 false)
    bool update_lidar(float px_meas, float py_meas);
    
    /// ZUPT: 속도 = 0 measurement update.
    /// 정지 감지 시 호출 → 잔여 속도 제거.
    bool update_zupt();
    
    // ── 상태 조회 ──
    EskfState get_state() const;
    Eigen::Matrix<float, 6, 6> get_covariance() const { return P_; }
    
    /// 위치 표준편차 (대각선 √).
    float pos_std_x() const { return std::sqrt(P_(0, 0)); }
    float pos_std_y() const { return std::sqrt(P_(1, 1)); }
    float pos_std_z() const { return std::sqrt(P_(2, 2)); }
    
    /// 초기화 여부.
    bool is_initialized() const { return initialized_; }

private:
    Eigen::Matrix<float, 6, 1> x_;  // state vector
    Eigen::Matrix<float, 6, 6> P_;  // covariance
    bool initialized_ = false;
};

}  // namespace ebimu