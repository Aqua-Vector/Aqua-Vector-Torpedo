// bias_calibrator.hpp — 시작 시 정지 캘리브레이션
//
// 어뢰가 정지한 상태에서 N초 동안 IMU 샘플 수집:
//   - 가속도 평균에서 중력(0,0,9.81) 빼면 → 가속도 bias (b_a)
//   - 자이로 평균 → 자이로 bias (b_g)
//   - 가속도 평균 방향 → 초기 pitch, roll (yaw=0)
//
// 좌표계: body frame (x=앞, y=좌, z=위)
// 단위: m/s², rad/s, rad
//
// 참조: ADR-001 (Bias estimator), ADR-006 (Initial calibration)

#pragma once

#include "torpedo/domain/estimator/bias_estimate.hpp"
#include "torpedo/sensor/imu_sample.hpp"

#include <Eigen/Dense>
#include <cstddef>

namespace torpedo::domain {

constexpr float GRAVITY_MS2 = 9.9974f;

struct CalibrationResult {
    BiasEstimate         bias;       // b_a, b_g
    Eigen::Quaternionf   q0;         // 초기 자세 (yaw=0)
    bool                 success = false;
    
    // 디버깅용
    float roll_rad  = 0.0f;
    float pitch_rad = 0.0f;
    int   samples_used = 0;
};

/**
 * 시작 시 정지 캘리브레이션.
 *
 * 사용:
 *   BiasCalibrator cal;
 *   cal.start(5.0f, 100);    // 5초 × 100Hz
 *   while (!cal.is_done()) {
 *       ImuSample s;
 *       if (imu.read(s)) cal.add_sample(s);
 *   }
 *   auto result = cal.finalize();
 *   eskf.set_initial_attitude(result.q0);
 *   bias = result.bias;
 */
class BiasCalibrator {
public:
    /// 캘리브레이션 시작
    void start(float duration_sec, int imu_rate_hz);
    
    /// 샘플 추가 (정지 중)
    /// @return true if 완료
    bool add_sample(const ImuSample& s);
    
    /// 결과 (완료 후)
    CalibrationResult finalize() const;
    
    bool is_done() const { return done_; }
    std::size_t samples_collected() const { return n_samples_; }
    std::size_t samples_needed() const { return samples_needed_; }
    float progress() const {
        return samples_needed_ > 0 
            ? static_cast<float>(n_samples_) / samples_needed_
            : 0.0f;
    }
    
private:
    // 누적 (대량 메모리 회피, 직접 합산)
    Eigen::Vector3f sum_accel_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f sum_gyro_  = Eigen::Vector3f::Zero();
    std::size_t     n_samples_ = 0;
    std::size_t     samples_needed_ = 0;
    bool            done_      = false;
};

} // namespace torpedo::domain