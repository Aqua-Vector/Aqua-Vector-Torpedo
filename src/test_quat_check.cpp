// test_quat_check.cpp — Quaternion 진단 (raw print 추가)
#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <Eigen/Dense>
#include <cstdio>
#include <cmath>
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
    
    std::printf("=== Quaternion 진단 (각 sample 직접 출력) ===\n");
    std::printf("정지 + 직립 상태 유지\n\n");
    
    // 안정화 2초
    std::printf("[안정화 2초 — 버리는 sample]\n");
    uint64_t t_warmup = monotonic_us() + 2000000ULL;
    int dropped = 0;
    while (monotonic_us() < t_warmup) {
        ImuSample s;
        if (imu.read_sample(s)) dropped++;
        else usleep(1000);
    }
    std::printf("  버린 sample: %d\n\n", dropped);
    
    // 정확히 100 sample
    std::printf("[100 sample 직접 출력 (처음 10개만 raw)]\n");
    std::printf("idx  |   ax       ay       az    |   qw      qx      qy      qz   |   |a|     |q|\n");
    
    int count = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float qw_sum = 0, qx_sum = 0, qy_sum = 0, qz_sum = 0;
    int outlier = 0;  // |a| > 12 m/s² 카운트
    
    while (count < 100) {
        ImuSample s;
        if (!imu.read_sample(s)) {
            usleep(1000);
            continue;
        }
        
        float a_mag = std::sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        float q_norm = std::sqrt(s.qw*s.qw + s.qx*s.qx + s.qy*s.qy + s.qz*s.qz);
        
        if (count < 10 || a_mag > 12.0f) {
            std::printf("%3d  | %+.3f  %+.3f  %+.3f  | %+.4f %+.4f %+.4f %+.4f |  %.3f   %.4f%s\n",
                        count, s.ax, s.ay, s.az,
                        s.qw, s.qx, s.qy, s.qz,
                        a_mag, q_norm,
                        a_mag > 12.0f ? "  <-- OUTLIER" : "");
        }
        
        if (a_mag > 12.0f) outlier++;
        
        ax_sum += s.ax; ay_sum += s.ay; az_sum += s.az;
        qw_sum += s.qw; qx_sum += s.qx; qy_sum += s.qy; qz_sum += s.qz;
        count++;
    }
    
    float ax_avg = ax_sum / count;
    float ay_avg = ay_sum / count;
    float az_avg = az_sum / count;
    float qw_avg = qw_sum / count;
    float qx_avg = qx_sum / count;
    float qy_avg = qy_sum / count;
    float qz_avg = qz_sum / count;
    
    float a_avg_mag = std::sqrt(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg);
    
    std::printf("\n=== 100 sample 통계 ===\n");
    std::printf("  Outlier (|a|>12 m/s²): %d / 100\n", outlier);
    std::printf("  ax 평균: %+.4f m/s²\n", ax_avg);
    std::printf("  ay 평균: %+.4f m/s²\n", ay_avg);
    std::printf("  az 평균: %+.4f m/s²  (기대 ~9.81)\n", az_avg);
    std::printf("  |a_avg| = %.4f m/s²  (기대 ~9.81)\n", a_avg_mag);
    
    if (outlier > 0) {
        std::printf("\n outlier 발견 → parse 잘못 가능성\n");
    }
    if (a_avg_mag > 11.0f) {
        std::printf("|a_avg| 비정상 — parse 또는 EBIMU 문제\n");
    }
    
    imu.close();
    return 0;
}