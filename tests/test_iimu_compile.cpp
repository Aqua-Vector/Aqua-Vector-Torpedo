// test_iimu_compile.cpp — IImu 인터페이스 컴파일 + 동작 검증
//
// 5계층 의존성 주입(SystemClock → FakeImu) 패턴이 컴파일되고
// 동작하는지 검증. 정지+중력 출력값을 0.01 허용오차로 검사.
//
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/sensor/fake_imu.hpp"

#include <cstdio>
#include <cmath>
#include <unistd.h>

int main() {
    // HAL 계층
    torpedo::SystemClock clock;

    // Sensor 계층 (HAL 의존성 주입)
    torpedo::FakeImu imu(clock);

    // 사용
    if (!imu.init()) {
        printf("[FAIL] imu.init() returned false\n");
        return 1;
    }
    printf("[OK] FakeImu initialized\n");

    // 5번 읽기 (각 사이 100μs 대기로 t_us 변화 확인)
    for (int i = 0; i < 5; ++i) {
        torpedo::ImuSample s;
        if (!imu.read(s)) {
            printf("[FAIL] imu.read() returned false at i=%d\n", i);
            return 1;
        }
        printf("  [%d] t=%llu us, a=(%.2f, %.2f, %.2f), g=(%.2f, %.2f, %.2f)\n",
               i,
               static_cast<unsigned long long>(s.t_us),
               s.ax, s.ay, s.az,
               s.gx, s.gy, s.gz);

        // 검증: 정지 + 중력
        if (std::abs(s.ax) > 0.01f || std::abs(s.ay) > 0.01f) {
            printf("[FAIL] ax/ay should be 0 in stationary state\n");
            return 1;
        }
        if (std::abs(s.az - 9.81f) > 0.01f) {
            printf("[FAIL] az should be 9.81 (gravity)\n");
            return 1;
        }
        if (std::abs(s.gx) > 0.01f || std::abs(s.gy) > 0.01f
                || std::abs(s.gz) > 0.01f) {
            printf("[FAIL] gyro should be 0 in stationary state\n");
            return 1;
        }
        if (!s.valid) {
            printf("[FAIL] sample.valid should be true\n");
            return 1;
        }

        usleep(100);  // 100μs 대기 (다음 read의 t_us 차이 보기)
    }

    imu.shutdown();
    printf("[OK] All tests passed\n");
    return 0;
}
