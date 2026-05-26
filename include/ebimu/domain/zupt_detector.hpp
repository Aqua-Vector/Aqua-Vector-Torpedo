// zupt_detector.hpp — Zero-Velocity Update 정지 감지
//
// IMU 측정값으로 정지 상태 감지.
// 조건:
//   |a| ≈ g (가속도 = 중력만)
//   |gyro| ≈ 0 (회전 X)
//   연속 N sample 유지
//
// 사용:
//   ZuptDetector detector;
//   if (detector.update(s)) {
//     // 정지 확정 → eskf.update_zupt()
//   }

#pragma once

#include "ebimu/sensor/imu_sample.hpp"
#include <cstdint>

namespace ebimu {

class ZuptDetector {
public:
    /**
     * @param accel_tol_ms2   |a| - g 허용 오차 (m/s²)
     * @param gyro_tol_rps    |gyro| 허용 오차 (rad/s)
     * @param min_consecutive 연속 stationary sample 필요 수
     */
    explicit ZuptDetector(float accel_tol_ms2  = 0.3f,
                          float gyro_tol_rps   = 0.05f,
                          int   min_consecutive = 50)
        : accel_tol_(accel_tol_ms2),
          gyro_tol_(gyro_tol_rps),
          min_consecutive_(min_consecutive) {}
    
    /// Sample 1개 평가. 정지 확정 시 true 반환.
    bool update(const ImuSample& s);
    
    /// 현재 stationary 카운터 (디버그용).
    int consecutive_count() const { return consecutive_; }
    
    /// Detector 리셋 (이동 검출 시 자동).
    void reset() { consecutive_ = 0; }
    
    /// 가장 최근 측정값 (출력용).
    float last_a_mag() const { return last_a_mag_; }
    float last_g_mag() const { return last_g_mag_; }

private:
    float accel_tol_;
    float gyro_tol_;
    int   min_consecutive_;
    
    int   consecutive_ = 0;
    float last_a_mag_ = 0.0f;
    float last_g_mag_ = 0.0f;
};

}  // namespace ebimu
