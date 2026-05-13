// BiasCalibrator 검증
//
// 시나리오:
//   1. 평평한 정지 → bias 거의 0, pitch/roll 거의 0
//   2. 기울어진 정지 → pitch/roll 비제로, bias 작게
//   3. 자이로 bias 시뮬 → 정확히 추정

#include "torpedo/domain/estimator/bias_calibrator.hpp"

#include <cstdio>
#include <cmath>

using namespace torpedo;
using namespace torpedo::domain;

static ImuSample make_sample(float ax, float ay, float az,
                              float gx, float gy, float gz) {
    ImuSample s;
    s.ax = ax; s.ay = ay; s.az = az;
    s.gx = gx; s.gy = gy; s.gz = gz;
    s.t_us = 0;
    s.valid = true;
    return s;
}

static bool approx(float a, float b, float tol) {
    return std::abs(a - b) < tol;
}

int main() {
    // ─── Test 1: 평평한 정지 ───
    printf("[Test 1] 평평한 정지 (z=g, 노이즈 없음)\n");
    {
        BiasCalibrator cal;
        cal.start(5.0f, 100);  // 5초 × 100Hz = 500 샘플
        
        // 평평하게 정지 — z만 중력
        for (int i = 0; i < 500; i++) {
            ImuSample s = make_sample(0, 0, GRAVITY_MS2, 0, 0, 0);
            cal.add_sample(s);
        }
        
        if (!cal.is_done()) { printf("[FAIL] not done\n"); return 1; }
        
        auto res = cal.finalize();
        if (!res.success) { printf("[FAIL] not success\n"); return 1; }
        
        printf("  samples=%d, roll=%.4f, pitch=%.4f\n",
               res.samples_used, res.roll_rad, res.pitch_rad);
        printf("  b_a = (%.4f, %.4f, %.4f)\n",
               res.bias.b_a.x(), res.bias.b_a.y(), res.bias.b_a.z());
        printf("  b_g = (%.4f, %.4f, %.4f)\n",
               res.bias.b_g.x(), res.bias.b_g.y(), res.bias.b_g.z());
        
        if (!approx(res.roll_rad,  0.0f, 1e-4f)) { printf("[FAIL] roll\n"); return 1; }
        if (!approx(res.pitch_rad, 0.0f, 1e-4f)) { printf("[FAIL] pitch\n"); return 1; }
        if (res.bias.b_a.norm() > 1e-4f) { printf("[FAIL] b_a too large\n"); return 1; }
        if (res.bias.b_g.norm() > 1e-4f) { printf("[FAIL] b_g too large\n"); return 1; }
        printf("  OK\n");
    }
    
    // ─── Test 2: 자이로 bias 시뮬 ───
    printf("\n[Test 2] 자이로 bias 시뮬 (gx=0.01 rad/s)\n");
    {
        BiasCalibrator cal;
        cal.start(5.0f, 100);
        
        for (int i = 0; i < 500; i++) {
            ImuSample s = make_sample(0, 0, GRAVITY_MS2,
                                       0.01f, -0.005f, 0.002f);
            cal.add_sample(s);
        }
        
        auto res = cal.finalize();
        printf("  b_g = (%.4f, %.4f, %.4f)\n",
               res.bias.b_g.x(), res.bias.b_g.y(), res.bias.b_g.z());
        
        if (!approx(res.bias.b_g.x(),  0.01f,  1e-5f)) { printf("[FAIL] b_g.x\n"); return 1; }
        if (!approx(res.bias.b_g.y(), -0.005f, 1e-5f)) { printf("[FAIL] b_g.y\n"); return 1; }
        if (!approx(res.bias.b_g.z(),  0.002f, 1e-5f)) { printf("[FAIL] b_g.z\n"); return 1; }
        printf("  OK (자이로 bias 정확히 추정)\n");
    }
    
    // ─── Test 3: 기울어진 정지 (pitch) ───
    printf("\n[Test 3] 5도 pitch 기울어진 정지\n");
    {
        // 5도 pitch 시:
        //   ax = -sin(5°) × g ≈ -0.854 m/s²
        //   az = +cos(5°) × g ≈ 9.769 m/s²
        const float pitch = 5.0f * 3.14159265f / 180.0f;
        const float ax = -std::sin(pitch) * GRAVITY_MS2;
        const float az =  std::cos(pitch) * GRAVITY_MS2;
        
        BiasCalibrator cal;
        cal.start(5.0f, 100);
        
        for (int i = 0; i < 500; i++) {
            ImuSample s = make_sample(ax, 0, az, 0, 0, 0);
            cal.add_sample(s);
        }
        
        auto res = cal.finalize();
        printf("  추정 pitch = %.4f rad (expected %.4f)\n", res.pitch_rad, pitch);
        printf("  b_a = (%.4f, %.4f, %.4f) (작아야 함)\n",
               res.bias.b_a.x(), res.bias.b_a.y(), res.bias.b_a.z());
        
        if (!approx(res.pitch_rad, pitch, 1e-3f)) { printf("[FAIL] pitch 추정\n"); return 1; }
        if (res.bias.b_a.norm() > 1e-2f) {
            printf("[FAIL] b_a 너무 큼 (자세 추정 후 잔차 작아야)\n");
            return 1;
        }
        printf("  OK (자세 추정 정확)\n");
    }
    
    // ─── Test 4: invalid 샘플 무시 ───
    printf("\n[Test 4] valid=false 샘플 무시\n");
    {
        BiasCalibrator cal;
        cal.start(0.1f, 100);  // 10 샘플
        
        for (int i = 0; i < 5; i++) {
            ImuSample s = make_sample(0, 0, GRAVITY_MS2, 0, 0, 0);
            s.valid = false;
            cal.add_sample(s);
        }
        if (cal.is_done()) { printf("[FAIL] invalid도 카운트\n"); return 1; }
        if (cal.samples_collected() != 0) { printf("[FAIL] count\n"); return 1; }
        printf("  OK\n");
    }
    
    printf("\n[OK] BiasCalibrator 검증 통과\n");
    return 0;
}