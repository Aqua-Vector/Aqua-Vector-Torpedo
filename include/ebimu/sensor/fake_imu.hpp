// fake_imu.hpp — 단위 테스트용 가짜 IMU
//
// 하드웨어 없이 알고리즘 검증 가능.
// 사용:
//   FakeImu imu;
//   imu.set_bias(0.3f, -0.4f, 0.04f);  // 인위적 bias
//   imu.set_gravity(9.80665f);          // 중력
//   if (imu.init()) {
//     ImuSample s;
//     while (imu.read_sample(s)) { ... }
//   }

#pragma once

#include "ebimu/sensor/iimu.hpp"
#include <cstdint>

namespace ebimu {

class FakeImu : public IImu {
public:
    FakeImu() = default;
    
    // ── 시뮬 설정 ──
    
    /// 가속도 bias 설정 (m/s², body frame).
    void set_bias(float bax, float bay, float baz) {
        bax_ = bax; bay_ = bay; baz_ = baz;
    }
    
    /// 중력 크기 (기본 9.80665).
    void set_gravity(float g) { g_ = g; }
    
    /// 자이로 값 설정 (rad/s).
    void set_gyro(float gx, float gy, float gz) {
        gx_ = gx; gy_ = gy; gz_ = gz;
    }
    
    /// Quaternion 설정 (자세).
    void set_quaternion(float qw, float qx, float qy, float qz) {
        qw_ = qw; qx_ = qx; qy_ = qy; qz_ = qz;
    }
    
    /// dt (마이크로초, 다음 read_sample에서 t_us 증가).
    /// 기본 10000 us = 10ms = 100Hz
    void set_dt_us(uint64_t dt) { dt_us_ = dt; }
    
    // ── IImu 구현 ──
    
    bool init() override {
        initialized_ = true;
        t_us_ = 0;
        return true;
    }
    
    bool read_sample(ImuSample& out) override {
        if (!initialized_) return false;
        
        // 시간 증가
        t_us_ += dt_us_;
        out.t_us = t_us_;
        
        // 가속도: bias + 중력 (정지 가정, body frame az = g)
        out.ax = bax_;
        out.ay = bay_;
        out.az = g_ + baz_;  // 중력 + bias
        
        // 자이로
        out.gx = gx_;
        out.gy = gy_;
        out.gz = gz_;
        
        // Quaternion (자세)
        out.qw = qw_;
        out.qx = qx_;
        out.qy = qy_;
        out.qz = qz_;
        
        return true;
    }
    
    void close() override { initialized_ = false; }
    
    // ── 상태 조회 ──
    bool is_initialized() const { return initialized_; }
    uint64_t current_t_us() const { return t_us_; }

private:
    bool     initialized_ = false;
    uint64_t t_us_        = 0;
    uint64_t dt_us_       = 10000;  // 10ms = 100Hz
    
    // bias (m/s²)
    float bax_ = 0.0f, bay_ = 0.0f, baz_ = 0.0f;
    
    // 중력 (m/s²)
    float g_ = 9.80665f;
    
    // 자이로 (rad/s)
    float gx_ = 0.0f, gy_ = 0.0f, gz_ = 0.0f;
    
    // Quaternion (단위 = 정지 직립)
    float qw_ = 1.0f;
    float qx_ = 0.0f, qy_ = 0.0f, qz_ = 0.0f;
};

}  // namespace ebimu