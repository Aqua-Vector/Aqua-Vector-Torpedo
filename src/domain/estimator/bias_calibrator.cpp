#include "torpedo/domain/estimator/bias_calibrator.hpp"

#include <cmath>

namespace torpedo::domain {

void BiasCalibrator::start(float duration_sec, int imu_rate_hz) {
    samples_needed_ = static_cast<std::size_t>(duration_sec * imu_rate_hz);
    sum_accel_.setZero();
    sum_gyro_.setZero();
    n_samples_ = 0;
    done_ = false;
}

bool BiasCalibrator::add_sample(const ImuSample& s) {
    if (done_) return true;
    if (!s.valid) return false;
    
    sum_accel_ += Eigen::Vector3f(s.ax, s.ay, s.az);
    sum_gyro_  += Eigen::Vector3f(s.gx, s.gy, s.gz);
    n_samples_++;
    
    if (n_samples_ >= samples_needed_) {
        done_ = true;
    }
    return done_;
}

CalibrationResult BiasCalibrator::finalize() const {
    CalibrationResult res;
    res.samples_used = static_cast<int>(n_samples_);
    
    if (n_samples_ == 0) {
        res.success = false;
        res.q0 = Eigen::Quaternionf::Identity();
        return res;
    }
    
    // ─── 1. 평균 ───
    Eigen::Vector3f mean_accel = sum_accel_ / static_cast<float>(n_samples_);
    Eigen::Vector3f mean_gyro  = sum_gyro_  / static_cast<float>(n_samples_);
    
    // ─── 2. 자이로 bias = 평균 (정지면 0이어야) ───
    res.bias.b_g = mean_gyro;
    
    // ─── 3. 가속도 분석 ───
    // body frame에서 정지 시 가속도 = -중력_body + bias
    // body z = 위 → 정지 시 가속도 z ≈ +9.81 (sensor가 위로 가속하는 느낌)
    //
    // 자세 추정: 평균 가속도가 nav frame에서 (0,0,9.81)이라고 가정
    //   roll  = atan2(ay, az)
    //   pitch = atan2(-ax, sqrt(ay² + az²))
    //   yaw   = 0 (외부 측정으로 보정 예정)
    
    float ax = mean_accel.x();
    float ay = mean_accel.y();
    float az = mean_accel.z();
    
    res.roll_rad  = std::atan2(ay, az);
    res.pitch_rad = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    
    // ─── 4. 가속도 bias 추정 ───
    // 추정된 자세를 이용해 중력 벡터(body frame) 계산
    //   g_body = R(roll, pitch).transpose() × (0, 0, 9.81)_nav
    // 그 다음 b_a = mean_accel - g_body
    Eigen::AngleAxisf roll_aa(res.roll_rad,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_aa(res.pitch_rad, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf q_init = roll_aa * pitch_aa;  // roll 후 pitch (또는 반대, 작은 각이라 같음)
    res.q0 = q_init;
    
    // nav frame 중력 (z 위)
    Eigen::Vector3f g_nav(0.0f, 0.0f, GRAVITY_MS2);
    
    // body frame에서 본 중력 = R_nb^T × g_nav = R_bn × g_nav (q_init는 body→nav)
    // 정지 시 가속도계 측정 = -중력가속도 + bias (자유낙하 시 0)
    // 사실 specific force = a_inertial - g
    // 정지 (a_inertial = 0) → spec force = -g_body
    // 근데 IMU 출력 = +g_body in body frame (관습)
    // → 정지 시 IMU = +g_body
    // → bias = mean_accel - g_body
    Eigen::Vector3f g_body = q_init.conjugate() * g_nav;
    res.bias.b_a = mean_accel - g_body;
    
    res.success = true;
    return res;

    // bias_calibrator.cpp finalize() 안에 추가
    std::printf("[DEBUG] mean_accel = (%.4f, %.4f, %.4f) m/s²\n",
    mean_accel.x(), mean_accel.y(), mean_accel.z());
}

} // namespace torpedo::domain