// system_clock.hpp — 고정밀 단조 시간
//
// 모든 시간 측정의 표준 함수.
// CLOCK_MONOTONIC 기반 (시스템 시간 변경에 영향 X).

#pragma once

#include <cstdint>

namespace ebimu {

/**
 * 단조 증가하는 마이크로초 시간.
 *
 * - 시작점: 시스템 부팅 시점 (절대값 의미 X)
 * - 보장: 항상 단조 증가
 * - 정밀도: 1 마이크로초 (Linux clock_gettime 기준)
 *
 */
uint64_t monotonic_us();

/**
 * 단조 증가 밀리초 (편의 함수).
 */
inline uint64_t monotonic_ms() {
    return monotonic_us() / 1000;
}

}  // namespace ebimu