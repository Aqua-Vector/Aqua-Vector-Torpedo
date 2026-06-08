// iimu.hpp — IMU 추상 인터페이스
//
// IMU 종류(실제 ISM330DHCX, FakeImu, ReplayImu 등)를 동일 인터페이스로
// 다루기 위한 추상 클래스. Domain 계층은 IImu만 알면 됨 → 하드웨어 없이도
// 알고리즘 단위 테스트 가능.
//
// 사용 패턴:
//   imu->init() → while(running) imu->read(sample) → imu->shutdown()
//
#pragma once

#include "torpedo/sensor/imu_sample.hpp"

namespace torpedo {

/**
 * IMU 추상 인터페이스.
 *
 * 구현체:
 *   - Ism330Imu: 실제 ISM330DHCX (SPI)
 *   - FakeImu: 테스트용 (정지+중력)
 *   - ReplayImu: CSV 재생 (분석/시뮬용)
 *
 * 사용:
 *   IImu* imu = new Ism330Imu(spi_bus, clock);
 *   imu->init();
 *   ImuSample s;
 *   while (imu->read(s)) { ... }
 *   imu->shutdown();
 */
class IImu {
public:
    virtual ~IImu() = default;

    /// 초기화 (SPI 설정, 칩 ID 확인). 성공 시 true.
    virtual bool init() = 0;

    /// 한 샘플 읽기. 새 데이터 + 유효하면 true.
    virtual bool read(ImuSample& out) = 0;

    /// 정리 (SPI 해제 등).
    virtual void shutdown() = 0;
};

} // namespace torpedo
