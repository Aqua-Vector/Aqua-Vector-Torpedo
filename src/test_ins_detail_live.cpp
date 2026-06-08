// test_ins_detail_live.cpp — 10Hz 상세 출력 (진단)
//
// 0.1초마다 position/velocity/yaw/raw accel/ZUPT
// 짚을 점: 점프 순간 진짜 무슨 일이 있는지

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
    return -yaw_rad * 180.0f / 3.14159265f;
}

int main(int argc, char* argv[]) {
    using namespace ebimu;
    
    int duration_sec = (argc > 1) ? std::atoi(argv[1]) : 15;
    
    std::printf("=== EBIMU 상세 진단 (10Hz 출력) ===\n");
    std::printf("측정: %d 초 (10Hz = %d sample)\n\n", duration_sec, duration_sec * 10);
    
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
        std::printf("[FAIL]\n");
        return 1;
    }
    std::printf("  Bias: bx=%+.2fmg, by=%+.2fmg, bz=%+.2fmg\n",
                bias.bx / 9.80665f * 1000,
                bias.by / 9.80665f * 1000,
                bias.bz / 9.80665f * 1000);
    
    float yaw_start = quat_to_yaw_deg(bias.q_start_w, bias.q_start_x,
                                       bias.q_start_y, bias.q_start_z);
    std::printf("  시작 yaw: %+.2f°\n\n", yaw_start);
    
    // ESKF + ZUPT
    Eskf eskf;
    eskf.init();
    ZuptDetector zupt;
    
    std::printf("[3/3] 항법 루프 (10Hz)\n");
    std::printf("%-5s | %-16s | %-16s | %-7s | %-16s | %-4s\n",
                "Time", "Position (m)", "Velocity (m/s)", "Yaw°", "Accel raw (m/s²)", "ZUPT");
    std::printf("------+------------------+------------------+---------+------------------+------\n");
    
    uint64_t t_start = monotonic_us();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000ULL;
    uint64_t t_prev = t_start;
    uint64_t t_next_print = t_start + 100000ULL;  // 100ms = 10Hz
    
    int sample_count = 0;
    bool zupt_active = false;
    
    // 마지막 raw 값 (출력용)
    float last_ax = 0, last_ay = 0;
    float last_yaw_rel = 0;
    
    Eigen::Quaternionf q_start_eig(bias.q_start_w, bias.q_start_x,
                                    bias.q_start_y, bias.q_start_z);
    q_start_eig.normalize();
    
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
        
        zupt_active = zupt.update(s);
        if (zupt_active) {
            eskf.update_zupt();
        }
        
        last_ax = s.ax;
        last_ay = s.ay;
        
        Eigen::Quaternionf q_abs(s.qw, s.qx, s.qy, s.qz);
        q_abs.normalize();
        Eigen::Quaternionf q_rel = q_start_eig.conjugate() * q_abs;
        last_yaw_rel = quat_to_yaw_deg(q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z());
        
        // 10Hz 출력
        if (t_now >= t_next_print) {
            float t = (t_now - t_start) / 1000000.0f;
            auto x = eskf.get_state();
            
            std::printf("%5.1f | (%+.3f, %+.3f) | (%+.3f, %+.3f) | %+7.2f | (%+.2f, %+.2f)    | %s\n",
                        t,
                        x.px, x.py,
                        x.vx, x.vy,
                        last_yaw_rel,
                        last_ax, last_ay,
                        zupt_active ? "ON" : "off");
            
            t_next_print = t_now + 100000ULL;
        }
    }
    
    imu.close();
    
    auto x = eskf.get_state();
    float pos_mag = std::sqrt(x.px*x.px + x.py*x.py);
    
    std::printf("\n=== 최종 ===\n");
    std::printf("  Sample: %d (~%d Hz)\n", sample_count, sample_count / duration_sec);
    std::printf("  최종 XY: (%+.3f, %+.3f) m, magnitude=%.3f m\n",
                x.px, x.py, pos_mag);
    std::printf("  최종 yaw: %+.2f°\n", last_yaw_rel);
    
    return 0;
}