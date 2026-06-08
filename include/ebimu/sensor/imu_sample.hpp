// imu_sample.hpp — IMU 측정 샘플
//
// EBIMU24GV5의 한 사이클 출력을 표현.
// - 가속도/자이로 raw (단위 변환 후)
// - AHRS 융합 quaternion (EBIMU 자체 계산)

#pragma once

#include <cstdint>

namespace ebimu {

/**
 * IMU 단일 샘플.
 * EBIMU24GV5 99Hz 출력 한 줄을 표현.
 */
struct ImuSample {
    uint64_t t_us = 0;   // 수신 시간 (monotonic, us)
    
    // ── 가속도 (body frame, m/s²) ──
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    
    // ── 자이로 (body frame, rad/s) ──
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    
    // ── EBIMU 자체 자세 (AHRS 융합, body→nav quaternion) ──
    // Hamilton convention (w, x, y, z)
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
};

}  // namespace ebimu