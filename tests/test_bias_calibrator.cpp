#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/sensor/fake_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <cmath>
#include <unistd.h>

bool nearly(float a, float b, float tol = 0.05f) {
    return std::fabs(a - b) < tol;
}

int main() {
    using namespace ebimu;
    
    std::printf("=== BiasCalibrator (Nav frame) 검증 ===\n");
    
    // [Test 1] 정지 직립 + bias 0 → 모두 0
    {
        std::printf("\n[Test 1] 정지 직립 (qw=1) + bias 0\n");
        FakeImu imu;
        imu.set_quaternion(1.0f, 0, 0, 0);  // 단위 quat (직립)
        imu.init();
        
        BiasCalibrator calib(1200000ULL);
        calib.start();
        for (int i = 0; i < 130; i++) {
            ImuSample s;
            if (imu.read_sample(s)) calib.add_sample(s);
            usleep(10000);
        }
        
        auto r = calib.finalize();
        std::printf("  결과: bx=%.4f, by=%.4f, bz=%.4f (n=%d)\n",
                    r.bx, r.by, r.bz, r.n_samples);
        
        if (!r.valid || !nearly(r.bx, 0) || !nearly(r.by, 0) || !nearly(r.bz, 0)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK] 정지 직립 → bias ~0\n");
    }
    
    // [Test 2] 자세 회전 (yaw 45°) + bias 0 → bias 여전히 0 (핵심!)
    {
        std::printf("\n[Test 2] yaw 45° 회전 + bias 0 → bias ~0 (Nav frame 핵심)\n");
        FakeImu imu;
        // yaw 45° quaternion: w=cos(22.5°), z=sin(22.5°)
        float w = std::cos(0.39269908f);
        float z = std::sin(0.39269908f);
        imu.set_quaternion(w, 0, 0, z);
        // Body frame 가속도: R^T × (0,0,9.81) — yaw 회전이라 Z 그대로
        // 정지 직립이라 body az = 9.81 (yaw는 az 안 바꿈)
        imu.set_bias(0, 0, 0);
        imu.init();
        
        BiasCalibrator calib(1200000ULL);
        calib.start();
        for (int i = 0; i < 130; i++) {
            ImuSample s;
            if (imu.read_sample(s)) calib.add_sample(s);
            usleep(10000);
        }
        
        auto r = calib.finalize();
        std::printf("  결과: bx=%.4f, by=%.4f, bz=%.4f\n", r.bx, r.by, r.bz);
        
        if (!r.valid || !nearly(r.bx, 0, 0.1) || !nearly(r.by, 0, 0.1) || !nearly(r.bz, 0, 0.1)) {
            std::printf("[FAIL] 자세 회전에도 nav frame bias 0이어야\n");
            return 1;
        }
        std::printf("[OK] 자세 회전 무관 — nav bias ~0\n");
    }
    
    // [Test 3] 실측 bias 모방
    {
        std::printf("\n[Test 3] EBIMU 실측 nav bias 모방 (-3mg, +7mg, +0mg)\n");
        FakeImu imu;
        imu.set_quaternion(1.0f, 0, 0, 0);  // 직립
        // Body frame bias = Nav frame bias (직립이라)
        const float true_b = 0.03f;  // 3mg
        imu.set_bias(-true_b, +0.07f, 0);
        imu.init();
        
        BiasCalibrator calib(1200000ULL);
        calib.start();
        for (int i = 0; i < 130; i++) {
            ImuSample s;
            if (imu.read_sample(s)) calib.add_sample(s);
            usleep(10000);
        }
        
        auto r = calib.finalize();
        std::printf("  추정: bx=%.4f, by=%.4f, bz=%.4f\n", r.bx, r.by, r.bz);
        
        if (!nearly(r.bx, -true_b, 0.01f) || !nearly(r.by, 0.07f, 0.01f)) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK] Nav bias 추정 정확\n");
    }
    
    // [Test 4] Sample 부족
    {
        std::printf("\n[Test 4] Sample 부족 거부\n");
        FakeImu imu;
        imu.init();
        
        BiasCalibrator calib(1200000ULL);
        calib.start();
        for (int i = 0; i < 50; i++) {
            ImuSample s;
            if (imu.read_sample(s)) calib.add_sample(s);
            usleep(10000);
        }
        
        auto r = calib.finalize();
        if (r.valid) {
            std::printf("[FAIL]\n");
            return 1;
        }
        std::printf("[OK] 거부 (n=%d)\n", r.n_samples);
    }
    
    std::printf("\n[ALL PASS] BiasCalibrator (Nav frame) 검증 통과\n");
    return 0;
}