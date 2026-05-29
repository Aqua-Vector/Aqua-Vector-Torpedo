// test_ebimu_live.cpp — ZYBO에서 진짜 EBIMU 동작 확인
//
// 10초 동안 read_sample 호출하며 통계 출력.
// 99Hz 안정성, 가속도, quaternion, 자이로 모두 확인.

#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <cmath>
#include <unistd.h>

int main(int argc, char* argv[]) {
    int duration_sec = (argc > 1) ? std::atoi(argv[1]) : 10;
    
    ebimu::EbimuConfig cfg;
    cfg.device = "/dev/ttyUSB0";
    cfg.baud = 921600;
    
    ebimu::EbimuImu imu(cfg);
    if (!imu.init()) {
        std::printf("[FAIL] init 실패\n");
        return 1;
    }
    
    std::printf("=== EBIMU 실측 검증 (%d초) ===\n\n", duration_sec);
    
    uint64_t t_start = ebimu::monotonic_us();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000ULL;
    
    int sample_count = 0;
    int last_print_sec = -1;
    
    // 통계
    float a_mag_sum = 0.0f;
    float a_mag_min = 1000.0f, a_mag_max = 0.0f;
    float q_norm_sum = 0.0f;
    float g_max = 0.0f;
    
    while (ebimu::monotonic_us() < t_end) {
        ebimu::ImuSample s;
        if (!imu.read_sample(s)) {
            usleep(1000);
            continue;
        }
        
        sample_count++;
        
        // 가속도 크기 (중력 1g = 9.81 m/s²이어야)
        float a_mag = std::sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        a_mag_sum += a_mag;
        if (a_mag < a_mag_min) a_mag_min = a_mag;
        if (a_mag > a_mag_max) a_mag_max = a_mag;
        
        // Quaternion norm (1.0 이어야)
        float q_norm = std::sqrt(s.qw*s.qw + s.qx*s.qx + s.qy*s.qy + s.qz*s.qz);
        q_norm_sum += q_norm;
        
        // 자이로 절대값
        float g_abs = std::sqrt(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);
        if (g_abs > g_max) g_max = g_abs;
        
        // 1초마다 출력
        uint64_t elapsed = ebimu::monotonic_us() - t_start;
        int sec = (int)(elapsed / 1000000ULL);
        if (sec != last_print_sec) {
            float a_mag_avg = a_mag_sum / sample_count;
            float q_norm_avg = q_norm_sum / sample_count;
            int hz = (elapsed > 0) ? (int)((uint64_t)sample_count * 1000000ULL / elapsed) : 0;
            
            std::printf("[%2ds] samples=%d (~%d Hz) | |a|=%.3f m/s² | |q|=%.4f | g_max=%.4f rad/s\n",
                        sec, sample_count, hz, a_mag_avg, q_norm_avg, g_max);
            last_print_sec = sec;
        }
    }
    
    imu.close();
    
    // 최종 통계
    uint64_t total_us = ebimu::monotonic_us() - t_start;
    float total_sec = total_us / 1000000.0f;
    int hz = (int)((float)sample_count / total_sec);
    
    std::printf("\n=== 최종 통계 ===\n");
    std::printf("  총 sample: %d\n", sample_count);
    std::printf("  측정 시간: %.2f 초\n", total_sec);
    std::printf("  평균 Hz: %d\n", hz);
    std::printf("  가속도 |a|: 평균 %.3f m/s² (min %.3f, max %.3f)\n",
                a_mag_sum / sample_count, a_mag_min, a_mag_max);
    std::printf("  Quaternion |q|: 평균 %.4f (1.0 이상적)\n",
                q_norm_sum / sample_count);
    
    // 검증
    int errors = 0;
    
    if (hz < 90 || hz > 110) {
        std::printf("Hz 비정상 (기대 99 ±10)\n");
        errors++;
    } else {
        std::printf("  ✓ Hz 정상\n");
    }
    
    float a_avg = a_mag_sum / sample_count;
    if (a_avg < 9.5f || a_avg > 10.2f) {
        std::printf("가속도 크기 비정상 (정지 시 9.81 m/s² 기대)\n");
        errors++;
    } else {
        std::printf("  ✓ 가속도 크기 ≈ 1g (정지)\n");
    }
    
    float q_avg = q_norm_sum / sample_count;
    if (q_avg < 0.99f || q_avg > 1.01f) {
        std::printf("Quaternion norm 비정상 (1.0 기대)\n");
        errors++;
    } else {
        std::printf("  ✓ Quaternion norm ≈ 1.0\n");
    }
    
    if (errors == 0) {
        std::printf("\n[ALL PASS] EBIMU 실측 검증 통과\n");
        return 0;
    } else {
        std::printf("\n[FAIL] %d개 검증 실패\n", errors);
        return 1;
    }
}