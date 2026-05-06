#include "torpedo/sensor/fake_imu.hpp"

namespace torpedo {

bool FakeImu::init() {
    initialized_ = true;
    return true;
}

bool FakeImu::read(ImuSample& out) {
    if (!initialized_) return false;

    out.t_us = clock_.now_us();
    // 정지 + 중력 (body z축이 위)
    out.ax = 0.0f;
    out.ay = 0.0f;
    out.az = 9.81f;
    out.gx = 0.0f;
    out.gy = 0.0f;
    out.gz = 0.0f;
    out.valid = true;
    return true;
}

void FakeImu::shutdown() {
    initialized_ = false;
}

} // namespace torpedo
