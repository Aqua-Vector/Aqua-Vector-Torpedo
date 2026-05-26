// iimu.hpp — IMU 추상 인터페이스
//
// EbimuImu, FakeImu 등이 구현.
// 도메인 로직(ESKF, BiasCalibrator)이 구체 IMU에 의존하지 않게 함.

#pragma once

#include "ebimu/sensor/imu_sample.hpp"

namespace ebimu {

/**
 * IMU 추상 인터페이스.
 * 
 * 사용 흐름:
 *   if (!imu.init()) return;
 *   while (running) {
 *     ImuSample s;
 *     if (imu.read_sample(s)) { ... }
 *   }
 *   imu.close();
 */
class IImu {
public:
    virtual ~IImu() = default;
    
    /// 초기화 (UART/SPI 열기, 설정 등).
    /// @return 성공 여부
    virtual bool init() = 0;
    
    /// 한 샘플 읽기.
    /// @param out  성공 시 채워질 샘플
    /// @return 성공 여부 (timeout/실패 시 false)
    virtual bool read_sample(ImuSample& out) = 0;
    
    /// 리소스 정리.
    virtual void close() = 0;
};

}  // namespace ebimu