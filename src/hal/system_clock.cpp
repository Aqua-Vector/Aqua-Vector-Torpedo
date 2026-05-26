#include "ebimu/hal/system_clock.hpp"
#include <time.h>

namespace ebimu {

uint64_t monotonic_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

}  // namespace ebimu