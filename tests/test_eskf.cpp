#include "ebimu/domain/eskf.hpp"
#include "ebimu/sensor/fake_imu.hpp"

#include <cstdio>
#include <cmath>

bool nearly(float a, float b, float tol) {
    return std::fabs(a - b) < tol;
}

int main() {
    using namespace ebimu;
    
    std::printf("=== 6-state ESKF (Nav frame bias) 검증 ===\n");
    
    // [Test 1] 정지
    {
        std::printf("\n[Test 1] 정지 (a=0) → 위치/속도 유지\n");
        Eskf eskf;
        eskf.init();
        
        BiasCalibration bias;
        bias.valid = true;  // bx=by=bz=0
        
        ImuSample s;
        s.ax = 0; s.ay = 0; s.az = 9.80665f;
        s.qw = 1; s.qx = 0; s.qy = 0; s.qz = 0;
        
        for (int i = 0; i < 100; i++) eskf.predict(s, bias, 0.01f);
        
        auto x = eskf.get_state();
        std::printf("  1초 후: p=(%+.4f, %+.4f, %+.4f) v=(%+.4f, %+.4f, %+.4f)\n",
                    x.px, x.py, x.pz, x.vx, x.vy, x.vz);
        
        if (!nearly(x.px, 0, 0.01f) || !nearly(x.vx, 0, 0.01f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK] 정지 → drift < 1cm\n");
    }
    
    // [Test 2] 1 m/s² 등가속
    {
        std::printf("\n[Test 2] 1 m/s² 가속 (body x)\n");
        Eskf eskf;
        eskf.init();
        
        BiasCalibration bias;
        bias.valid = true;
        
        ImuSample s;
        s.ax = 1.0f; s.ay = 0; s.az = 9.80665f;
        s.qw = 1; s.qx = 0; s.qy = 0; s.qz = 0;
        
        for (int i = 0; i < 100; i++) eskf.predict(s, bias, 0.01f);
        
        auto x = eskf.get_state();
        std::printf("  1초 후: px=%.4f (기대 0.5), vx=%.4f (기대 1.0)\n", x.px, x.vx);
        
        if (!nearly(x.vx, 1.0f, 0.02f) || !nearly(x.px, 0.5f, 0.02f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK]\n");
    }
    
    // [Test 3] Nav bias 제거
    {
        std::printf("\n[Test 3] Nav bias 제거\n");
        Eskf eskf;
        eskf.init();
        
        BiasCalibration bias;
        bias.valid = true;
        bias.bx = 0.5f;  // nav frame x bias
        
        ImuSample s;
        s.ax = 0.5f; s.ay = 0; s.az = 9.80665f;
        s.qw = 1; s.qx = 0; s.qy = 0; s.qz = 0;
        
        for (int i = 0; i < 100; i++) eskf.predict(s, bias, 0.01f);
        
        auto x = eskf.get_state();
        std::printf("  1초 후: px=%.4f (기대 0)\n", x.px);
        
        if (!nearly(x.px, 0, 0.01f) || !nearly(x.vx, 0, 0.01f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK] Nav bias 정확 제거\n");
    }
    
    // [Test 4] LiDAR update
    {
        std::printf("\n[Test 4] LiDAR update\n");
        Eskf eskf;
        eskf.init(5.0f, 3.0f, 0);
        
        if (!eskf.update_lidar(5.5f, 3.2f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        
        auto x = eskf.get_state();
        std::printf("  px=%.4f py=%.4f\n", x.px, x.py);
        
        if (x.px <= 5.0f || x.px >= 5.5f) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK]\n");
    }
    
    // [Test 5] NaN 거부
    {
        std::printf("\n[Test 5] NaN 거부\n");
        Eskf eskf;
        eskf.init();
        if (eskf.update_lidar(std::nanf(""), 1.0f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK]\n");
    }
    
    std::printf("\n[ALL PASS] ESKF 검증 통과\n");
    return 0;
}