// test_axis_id.cpp — IMU 축 방향 식별
//
// 보드를 들어 올리면서 어느 축이 +방향인지 확인

#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <cmath>
#include <unistd.h>

float quat_to_yaw_deg(float w, float x, float y, float z) {
    return std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * 180.0f / 3.14159265f;
}

int main() {
    using namespace ebimu;
    
    EbimuConfig cfg;
    cfg.device = "/dev/ttyUSB0";
    cfg.baud = 921600;
    EbimuImu imu(cfg);
    if (!imu.init()) {
        std::printf("[FAIL] init\n");
        return 1;
    }
    
    std::printf("=== IMU 축 방향 진단 ===\n\n");
    std::printf("보드를 움직이면서 어떤 축이 변하는지 관찰\n");
    std::printf("0.5초마다 출력, Ctrl+C로 종료\n\n");
    std::printf("정지+평평: az ≈ +9.8 (중력)\n");
    std::printf("X 방향 위로: ax → +9.8\n");
    std::printf("Y 방향 위로: ay → +9.8\n");
    std::printf("거꾸로:      az → -9.8\n\n");
    std::printf("%-6s | %-8s %-8s %-8s | %-10s | %-8s\n",
                "Time", "ax(m/s²)", "ay(m/s²)", "az(m/s²)", "|a|", "Yaw(°)");
    std::printf("-------+--------+--------+--------+------------+----------\n");
    
    uint64_t t_start = monotonic_us();
    uint64_t t_next_print = t_start + 500000ULL;
    
    while (true) {
        ImuSample s;
        if (!imu.read_sample(s)) {
            usleep(1000);
            continue;
        }
        
        uint64_t t_now = monotonic_us();
        if (t_now < t_next_print) continue;
        t_next_print = t_now + 500000ULL;
        
        float a_mag = std::sqrt(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        float yaw = quat_to_yaw_deg(s.qw, s.qx, s.qy, s.qz);
        float t = (t_now - t_start) / 1000000.0f;
        
        // 어느 축이 큰지 표시
        std::string hint = "";
        float abs_x = std::fabs(s.ax);
        float abs_y = std::fabs(s.ay);
        float abs_z = std::fabs(s.az);
        if (abs_x > 7.0f) hint = (s.ax > 0) ? " ← +X 위로" : " ← -X 위로";
        else if (abs_y > 7.0f) hint = (s.ay > 0) ? " ← +Y 위로" : " ← -Y 위로";
        else if (abs_z > 7.0f) hint = (s.az > 0) ? " ← +Z 위 (평평)" : " ← -Z 위 (거꾸로)";
        
        std::printf("%-6.1f | %+6.2f  %+6.2f  %+6.2f  | %+6.2f     | %+7.2f%s\n",
                    t, s.ax, s.ay, s.az, a_mag, yaw, hint.c_str());
    }
    
    imu.close();
    return 0;
}