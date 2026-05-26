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
    
    // Quaternion + body accel 누적 (변환은 finalize에서)
    qw_sum_ += s.qw;
    qx_sum_ += s.qx;
    qy_sum_ += s.qy;
    qz_sum_ += s.qz;
    
    ax_nav_sum_ += s.ax;  // 사실은 body sum
    ay_nav_sum_ += s.ay;
    az_nav_sum_ += s.az;
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
    
    // 1. 평균 quaternion (시작 자세)
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
    
    // 2. 평균 body accel
    float ax_body_avg = ax_nav_sum_ * inv_n;
    float ay_body_avg = ay_nav_sum_ * inv_n;
    float az_body_avg = az_nav_sum_ * inv_n;
    
    // 3. 평균 quaternion으로 body → q_start frame 변환
    //    (시작 자세에서는 q_rel = I → body = q_start frame nav)
    //    실제로 q_avg ≈ q_start 이므로
    //    R(q_avg⁻¹ × q_avg) = R(I) = I
    //    → 변환된 nav = body 그대로
    //    
    //    정지 시 q_start frame nav 가속도 = (0, 0, g)
    //    → bias = body 평균 - (0, 0, g)
    //
    //    근데 body가 약간 기울어진 거면?
    //    → 평균 body가 (0, 0, g) 아닐 수도
    //    → 평균 R도 약간 회전
    //    → R_rel × body_avg = (0, 0, g)에 가까움
    
    // 깨끗하게: q_rel = q_start.conjugate() × q_avg = I (정확히)
    // → R(I) × body_avg = body_avg
    // → bias = body_avg - (0, 0, g)
    
    result.bx = ax_body_avg;
    result.by = ay_body_avg;
    result.bz = az_body_avg - g_;
    
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