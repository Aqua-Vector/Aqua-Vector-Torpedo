// test_ins_live.cpp — 통합 실측 (ZUPT 포함)
#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/domain/eskf.hpp"
#include "ebimu/domain/zupt_detector.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <Eigen/Dense>
#include <cstdio>
#include <cmath>
#include <unistd.h>

float quat_to_yaw_deg(float w, float x, float y, float z) {
    float yaw_rad = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    return -yaw_rad * 180.0f / 3.14159265f;  // 시연 컨벤션
}

int main(int argc, char* argv[]) {
    using namespace ebimu;
    
    int duration_sec = (argc > 1) ? std::atoi(argv[1]) : 30;
    
    std::printf("=== EBIMU + ESKF + ZUPT 통합 실측 ===\n");
    std::printf("Yaw: 오른쪽=+, 왼쪽=-\n");
    std::printf("ZUPT: 정지 감지 시 속도 0 update\n");
    std::printf("측정 시간: %d 초\n\n", duration_sec);
    
    EbimuConfig cfg;
    cfg.device = "/dev/ttyUSB0";
    cfg.baud = 921600;
    EbimuImu imu(cfg);
    if (!imu.init()) {
        std::printf("[FAIL] IMU init\n");
        return 1;
    }
    
    // 안정화
    std::printf("[안정화] 2초...\n");
    uint64_t t_warmup = monotonic_us() + 2000000ULL;
    while (monotonic_us() < t_warmup) {
        ImuSample s;
        if (!imu.read_sample(s)) usleep(1000);
    }
    std::printf("  완료\n\n");
    
    // 캘리브
    std::printf("[1/3] 5초 정지 캘리브\n");
    BiasCalibrator calib(5000000ULL);
    calib.start();
    
    int last_print = -1;
    while (calib.is_collecting()) {
        ImuSample s;
        if (!imu.read_sample(s)) {
            usleep(1000);
            continue;
        }
        calib.add_sample(s);
        int sec = (int)(calib.progress() * 5);
        if (sec != last_print && sec <= 5) {
            std::printf("  [%d/5] %.0f%%\n", sec, calib.progress() * 100);
            last_print = sec;
        }
    }
    
    auto bias = calib.finalize();
    if (!bias.valid) {
        std::printf("[FAIL] 캘리브\n");
        return 1;
    }
    std::printf("  Bias: bx=%+.2fmg, by=%+.2fmg, bz=%+.2fmg\n",
                bias.bx / 9.80665f * 1000,
                bias.by / 9.80665f * 1000,
                bias.bz / 9.80665f * 1000);
    
    float yaw_start = quat_to_yaw_deg(bias.q_start_w, bias.q_start_x,
                                       bias.q_start_y, bias.q_start_z);
    std::printf("  시작 yaw: %+.2f°  → 시연 frame 0°로 정렬\n\n", yaw_start);
    
    // ESKF + ZUPT
    Eskf eskf;
    eskf.init();
    ZuptDetector zupt;  // 기본: |a-g|<0.3, |gyro|<0.05, 50 sample
    
    std::printf("[3/3] 항법 루프 (ZUPT 활성)\n");
    std::printf("%-4s | %-22s | %-22s | %-9s | %-5s\n",
                "Time", "Position XY (m)", "Velocity (m/s)", "Yaw", "ZUPT");
    std::printf("-----+------------------------+------------------------+-----------+------\n");
    
    uint64_t t_start = monotonic_us();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000ULL;
    uint64_t t_prev = t_start;
    
    int sample_count = 0;
    int zupt_count = 0;
    int print_count = 0;
    float last_yaw_rel = 0;
    bool zupt_active = false;
    
    Eigen::Quaternionf q_start(bias.q_start_w, bias.q_start_x,
                                bias.q_start_y, bias.q_start_z);
    q_start.normalize();
    
    while (monotonic_us() < t_end) {
        ImuSample s;
        if (!imu.read_sample(s)) {
            usleep(1000);
            continue;
        }
        
        uint64_t t_now = monotonic_us();
        float dt = (t_now - t_prev) / 1000000.0f;
        t_prev = t_now;
        
        eskf.predict(s, bias, dt);
        sample_count++;
        
        // ZUPT 정지 감지
        zupt_active = zupt.update(s);
        if (zupt_active) {
            eskf.update_zupt();
            zupt_count++;
        }
        
        // 자세 (출력용)
        Eigen::Quaternionf q_abs(s.qw, s.qx, s.qy, s.qz);
        q_abs.normalize();
        Eigen::Quaternionf q_rel = q_start.conjugate() * q_abs;
        last_yaw_rel = quat_to_yaw_deg(q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z());
        
        int elapsed_sec = (int)((t_now - t_start) / 1000000ULL);
        if (elapsed_sec != print_count) {
            auto x = eskf.get_state();
            std::printf("%-4d | (%+.3f, %+.3f)       | (%+.3f, %+.3f)       | %+7.2f° | %s\n",
                        elapsed_sec,
                        x.px, x.py,
                        x.vx, x.vy,
                        last_yaw_rel,
                        zupt_active ? "ON" : "off");
            print_count = elapsed_sec;
        }
    }
    
    imu.close();
    
    auto x = eskf.get_state();
    float pos_mag = std::sqrt(x.px*x.px + x.py*x.py);
    
    std::printf("\n=== 최종 ===\n");
    std::printf("  Sample: %d (~%d Hz)\n", sample_count, sample_count / duration_sec);
    std::printf("  ZUPT 적용: %d sample (%.1f%%)\n",
                zupt_count, 100.0f * zupt_count / sample_count);
    std::printf("  최종 XY: (%+.3f, %+.3f) m, magnitude=%.3f m\n",
                x.px, x.py, pos_mag);
    std::printf("  최종 Yaw: %+.2f°\n", last_yaw_rel);
    
    if (pos_mag < 0.5f) {
        std::printf("  [OK] XY drift < 0.5m\n");
    } else if (pos_mag < 1.0f) {
        std::printf("  [WARN] XY drift 0.5~1m\n");
    } else {
        std::printf("  [FAIL] XY drift > 1m\n");
    }
    
    return 0;
}
