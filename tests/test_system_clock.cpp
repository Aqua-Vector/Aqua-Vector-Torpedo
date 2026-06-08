#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <unistd.h>

int main() {
    using namespace ebimu;
    
    std::printf("=== system_clock 검증 ===\n");
    
    uint64_t t0 = monotonic_us();
    uint64_t t1 = monotonic_us();
    if (t1 < t0) {
        std::printf("[FAIL] 단조성 위반\n");
        return 1;
    }
    std::printf("[OK] 단조성\n");
    
    uint64_t before = monotonic_us();
    usleep(10000);
    uint64_t after = monotonic_us();
    uint64_t diff = after - before;
    
    if (diff < 9000 || diff > 15000) {
        std::printf("[FAIL] 10ms sleep 후 %llu us\n", (unsigned long long)diff);
        return 1;
    }
    std::printf("[OK] 10ms sleep: %llu us\n", (unsigned long long)diff);
    
    uint64_t us = monotonic_us();
    uint64_t ms = monotonic_ms();
    if (ms * 1000 > us + 2000 || us > ms * 1000 + 2000) {
        std::printf("[FAIL] us/ms 변환\n");
        return 1;
    }
    std::printf("[OK] us/ms 일관성\n");
    
    std::printf("\n[ALL PASS]\n");
    return 0;
}