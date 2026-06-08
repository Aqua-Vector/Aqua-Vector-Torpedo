#include "ebimu/domain/zupt_detector.hpp"
#include <cmath>

namespace ebimu {

namespace {
constexpr float G = 9.80665f;
}

bool ZuptDetector::update(const ImuSample& s) {
    // 가속도 magnitude
    last_a_mag_ = std::sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
    
    // 자이로 magnitude
    last_g_mag_ = std::sqrt(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);
    
    // 정지 조건 체크
    bool accel_ok = std::fabs(last_a_mag_ - G) < accel_tol_;
    bool gyro_ok  = last_g_mag_ < gyro_tol_;
    
    if (accel_ok && gyro_ok) {
        consecutive_++;
    } else {
        consecutive_ = 0;  // 이동 감지 → 리셋
    }
    
    // 연속 min_consecutive_ 이상이면 정지 확정
    return consecutive_ >= min_consecutive_;
}

}  // namespace ebimu
