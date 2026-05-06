// system_clock.cpp — Linux 기반 IClock 구현
//
// clock_gettime(CLOCK_MONOTONIC) 호출하여 (sec, nsec) → μs 변환.
// uint64_t 사용으로 약 58만 년 동안 오버플로 없음.
//
#include "torpedo/hal/system_clock.hpp"

#include <ctime>

namespace torpedo {

uint64_t SystemClock::now_us() const {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

} // namespace torpedo
