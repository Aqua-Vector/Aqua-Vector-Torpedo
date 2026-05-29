#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace ebimu {

void BiasCalibrator::start() {
    collecting_ = true;
    t_start_us_ = monotonic_us();
    n_ = 0;
    ax_nav_sum_ = ay_nav_sum_ = az_nav_sum_ = 0.0f;
    qw_sum_ = qx_sum_ = qy_sum_ = qz_sum_ = 0.0f;
}

void BiasCalibrator::add_sample(const ImuSample& s) {
    if (!collecting_) return;
    
    uint64_t elapsed = monotonic_us() - t_start_us_;
    if (elapsed >= duration_us_) {
        collecting_ = false;
        return;
    }
    
    // Quaternion 누적 (q_start 계산용)
    qw_sum_ += s.qw;
    qx_sum_ += s.qx;
    qy_sum_ += s.qy;
    qz_sum_ += s.qz;
    
    // ★ Body → Nav 변환 후 누적 (진짜 Nav frame!) ★
    // EBIMU quaternion으로 절대 nav frame 변환
    // → IMU 기울어짐도 quaternion에 반영되어 보정됨
    Eigen::Quaternionf q(s.qw, s.qx, s.qy, s.qz);
    q.normalize();
    Eigen::Vector3f a_body(s.ax, s.ay, s.az);
    Eigen::Vector3f a_nav = q.toRotationMatrix() * a_body;
    
    ax_nav_sum_ += a_nav(0);
    ay_nav_sum_ += a_nav(1);
    az_nav_sum_ += a_nav(2);
    n_++;
}

BiasCalibration BiasCalibrator::finalize() {
    collecting_ = false;
    
    BiasCalibration result;
    result.n_samples = n_;
    
    if (n_ < 100) {
        result.valid = false;
        return result;
    }
    
    float inv_n = 1.0f / static_cast<float>(n_);
    
    // 1. 평균 quaternion (시작 자세, 정렬용)
    float qw_avg = qw_sum_ * inv_n;
    float qx_avg = qx_sum_ * inv_n;
    float qy_avg = qy_sum_ * inv_n;
    float qz_avg = qz_sum_ * inv_n;
    
    float norm = std::sqrt(qw_avg*qw_avg + qx_avg*qx_avg + qy_avg*qy_avg + qz_avg*qz_avg);
    if (norm < 0.001f) {
        result.valid = false;
        return result;
    }
    
    result.q_start_w = qw_avg / norm;
    result.q_start_x = qx_avg / norm;
    result.q_start_y = qy_avg / norm;
    result.q_start_z = qz_avg / norm;
    
    // 2. Nav frame 가속도 평균
    float ax_nav_mean = ax_nav_sum_ * inv_n;
    float ay_nav_mean = ay_nav_sum_ * inv_n;
    float az_nav_mean = az_nav_sum_ * inv_n;
    
    // 3. 정지 시 Nav frame 가속도 = (0, 0, g) 가 정상
    //    차이 = 진짜 bias (절대 nav frame)
    //    → IMU 기울어짐은 quaternion이 보정 → bias 작아야
    result.bx = ax_nav_mean;
    result.by = ay_nav_mean;
    result.bz = az_nav_mean - g_;
    
    result.valid = true;
    return result;
}

float BiasCalibrator::progress() const {
    if (!collecting_) return 1.0f;
    uint64_t elapsed = monotonic_us() - t_start_us_;
    if (elapsed >= duration_us_) return 1.0f;
    return static_cast<float>(elapsed) / static_cast<float>(duration_us_);
}

}  // namespace ebimu