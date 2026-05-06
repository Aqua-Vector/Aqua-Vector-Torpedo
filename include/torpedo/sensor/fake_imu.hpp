// fake_imu.hpp — 테스트용 가짜 IMU (헤더)
//
// 실제 하드웨어 없이 알고리즘 검증용. 정지+중력 상태를 시뮬레이션:
//   가속도 = (0, 0, 9.81), 각속도 = (0, 0, 0)
//
#pragma once

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/hal/iclock.hpp"

namespace torpedo {

/**
 * 테스트용 가짜 IMU.
 *
 * 정지 상태 시뮬레이션:
 *   가속도: (0, 0, 9.81) — 중력만
 *   각속도: (0, 0, 0)
 *
 * 향후 확장 (Day 2 이후):
 *   - 회전 시뮬 (yaw 90° 등)
 *   - 평면 운동 시뮬
 *   - CSV 재생 (ReplayImu로 분리 가능)
 */
class FakeImu : public IImu {
public:
    explicit FakeImu(IClock& clock) : clock_(clock) {}

    bool init() override;
    bool read(ImuSample& out) override;
    void shutdown() override;

private:
    IClock& clock_;
    bool initialized_ = false;
};

} // namespace torpedo
