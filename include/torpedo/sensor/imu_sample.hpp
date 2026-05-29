// imu_sample.hpp — IMU 한 샘플의 데이터 구조 정의
//
// 6축 IMU(가속도 3 + 각속도 3) 측정값 + 타임스탬프 + 유효성 플래그를
// 단일 구조체로 묶음. IImu가 읽고, AttitudeFilter/ESKF가 소비하는
// 시스템 전체의 표준 데이터 단위.
//
// 좌표계: body frame (어뢰 기준, x=앞 / y=좌 / z=위)
// 단위:   m/s² (specific force, 중력 포함), rad/s
//
#pragma once

#include <cstdint>

namespace torpedo {

/**
 * IMU 한 샘플의 모든 측정값.
 *
 * 좌표계: body frame (어뢰 기준)
 *   x: 앞 (heading)
 *   y: 좌측
 *   z: 위
 *
 * 단위:
 *   가속도 ax/ay/az: m/s² (specific force, 중력 포함)
 *   각속도 gx/gy/gz: rad/s
 *
 * 타임스탬프: t_us
 *   monotonic μs, 측정 시작 시점부터.
 *   IClock::now_us()로 얻음.
 */
struct ImuSample {
    uint64_t t_us;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    bool valid;
};

} // namespace torpedo
