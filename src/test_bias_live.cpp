// test_bias_live.cpp — ZYBO에서 진짜 EBIMU 5초 캘리브
#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <unistd.h>

int main() {
    using namespace ebimu;
    
    EbimuConfig cfg;
    cfg.device = "/dev/ttyUSB0";
    cfg.baud = 921600;
    
    EbimuImu imu(cfg);
    if (!imu.init()) {
        std::printf("[FAIL] IMU init\n");
        return 1;
    }
    
    std::printf("=== EBIMU 5초 정지 캘리브 (Nav frame) ===\n");
    std::printf("정지 상태 유지하세요...\n\n");
    
    // 안정화 2초
    std::printf("[안정화] 2초 dummy read...\n");
    uint64_t t_warmup = monotonic_us() + 2000000ULL;
    int dropped = 0;
    while (monotonic_us() < t_warmup) {
        ImuSample s;
        if (imu.read_sample(s)) dropped++;
        else usleep(1000);
    }
    std::printf("  버린 sample: %d\n\n", dropped);
    
    // 5초 캘리브
    BiasCalibrator calib(5000000ULL);
    calib.start();
    
    int last_print = -1;
    int read_ok = 0, read_fail = 0;
    while (calib.is_collecting()) {
        ImuSample s;
        if (!imu.read_sample(s)) {
            read_fail++;
            usleep(1000);
            continue;
        }
        calib.add_sample(s);
        read_ok++;
        
        int sec = (int)(calib.progress() * 5);
        if (sec != last_print && sec <= 5) {
            std::printf("[%ds/5s] %.0f%% (ok=%d, fail=%d)\n",
                        sec, calib.progress() * 100, read_ok, read_fail);
            last_print = sec;
        }
    }
    
    auto result = calib.finalize();
    imu.close();
    
    std::printf("\n=== 캘리브 결과 (Nav frame) ===\n");
    std::printf("  Sample: %d\n", result.n_samples);
    std::printf("  Valid: %s\n", result.valid ? "true" : "false");
    
    if (!result.valid) {
        std::printf("[FAIL]\n");
        return 1;
    }
    
    std::printf("\n  Nav frame bias (m/s²):\n");
    std::printf("    bx = %+.6f m/s² (%+.2f mg)\n", result.bx, result.bx / 9.80665f * 1000);
    std::printf("    by = %+.6f m/s² (%+.2f mg)\n", result.by, result.by / 9.80665f * 1000);
    std::printf("    bz = %+.6f m/s² (%+.2f mg)  ← 중력 제거 후\n",
                result.bz, result.bz / 9.80665f * 1000);
    
    std::printf("\n[OK] 캘리브 완료\n");
    return 0;
}