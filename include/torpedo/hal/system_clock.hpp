// system_clock.hpp — Linux 기반 IClock 구현 (헤더)
//
// clock_gettime(CLOCK_MONOTONIC)을 사용하여 단조 증가하는
// 마이크로초 타임스탬프 제공. PetaLinux도 동일 API 지원.
//
#pragma once

#include "torpedo/hal/iclock.hpp"

namespace torpedo {

/**
 * Linux clock_gettime(CLOCK_MONOTONIC) 기반 IClock 구현.
 * PetaLinux도 동일 API 지원.
 */
class SystemClock : public IClock {
public:
    uint64_t now_us() const override;
};

} // namespace torpedo
