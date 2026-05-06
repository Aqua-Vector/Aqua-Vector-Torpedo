// iclock.hpp — 시간 추상화 인터페이스
//
// 시스템 시간을 마이크로초(μs)로 반환하는 추상 인터페이스.
// 실제 환경(SystemClock)과 테스트 환경(FakeClock, 향후 추가)을
// 동일 인터페이스로 다루기 위함.
//
// 의존성 주입(DI)으로 IImu 등 시간 필요한 컴포넌트에 전달.
//
#pragma once

#include <cstdint>

namespace torpedo {

/**
 * 시간 추상화 인터페이스.
 *
 * 실제: SystemClock (Linux clock_gettime CLOCK_MONOTONIC)
 * 테스트: FakeClock (수동 시간 진행) — 향후 추가
 *
 * 모든 메서드는 마이크로초(μs) 반환.
 */
class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t now_us() const = 0;
};

} // namespace torpedo
