// ImuDownsampler 검증
//
// 1. 모든 sub-sample 동일 → 출력 = 같은 값
// 2. Spike 1개 → 제거됨, 나머지 평균
// 3. 노이즈 → 평균이 0에 가까워짐
// 4. 8개 미만 → invalid 결과

#include "torpedo/sensor/imu_downsampler.hpp"

#include <cstdio>
#include <cmath>

using namespace torpedo;

static ImuSample make_sample(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              uint64_t t_us = 0) {
    ImuSample s;
    s.ax = ax; s.ay = ay; s.az = az;
    s.gx = gx; s.gy = gy; s.gz = gz;
    s.t_us = t_us;
    s.valid = true;
    return s;
}

static bool approx(float a, float b, float tol) {
    return std::abs(a - b) < tol;
}

int main() {
    // ─── Test 1: 모든 sub-sample 동일 ───
    printf("[Test 1] 동일 sub-sample 8개 → 같은 값\n");
    {
        ImuDownsampler down;
        for (int i = 0; i < 8; i++) {
            down.add(make_sample(1.0f, 2.0f, 9.81f,
                                  0.01f, -0.02f, 0.005f, i * 1000));
        }
        if (!down.is_full()) { printf("[FAIL] not full\n"); return 1; }
        
        ImuSample r = down.finalize();
        if (!r.valid) { printf("[FAIL] invalid\n"); return 1; }
        
        if (!approx(r.ax, 1.0f, 1e-5f) ||
            !approx(r.ay, 2.0f, 1e-5f) ||
            !approx(r.az, 9.81f, 1e-5f)) {
            printf("[FAIL] accel %.4f %.4f %.4f\n", r.ax, r.ay, r.az);
            return 1;
        }
        if (!approx(r.gx, 0.01f, 1e-5f) ||
            !approx(r.gy, -0.02f, 1e-5f) ||
            !approx(r.gz, 0.005f, 1e-5f)) {
            printf("[FAIL] gyro\n");
            return 1;
        }
        if (r.t_us != 7000) { printf("[FAIL] t_us=%lu\n", r.t_us); return 1; }
        printf("  OK\n");
    }
    
    // ─── Test 2: Spike 1개 → Trimmed Mean이 제거 ───
    printf("\n[Test 2] Spike 1개 제거 — 7개 정상 + 1개 큰 값\n");
    {
        ImuDownsampler down;
        // 7개 = ax 1.0
        for (int i = 0; i < 7; i++) {
            down.add(make_sample(1.0f, 0, 9.81f, 0, 0, 0));
        }
        // 1개 = ax 100 (큰 spike)
        down.add(make_sample(100.0f, 0, 9.81f, 0, 0, 0));
        
        ImuSample r = down.finalize();
        // 8개: [1, 1, 1, 1, 1, 1, 1, 100] 정렬: [1,1,1,1,1,1,1,100]
        // 양 끝 제외 (1, 100) → 남은 6개 = [1,1,1,1,1,1] → 평균 = 1.0
        printf("  ax = %.4f (spike 제거 → 1.0 기대)\n", r.ax);
        if (!approx(r.ax, 1.0f, 1e-5f)) {
            printf("[FAIL] Trimmed Mean 동작 X\n");
            return 1;
        }
        printf("  OK (spike 제거됨)\n");
    }
    
    // ─── Test 3: 큰 spike 양쪽 → 둘 다 제거 ───
    printf("\n[Test 3] Spike 양쪽 — 1개 큰 값 + 1개 작은 값\n");
    {
        ImuDownsampler down;
        // 6개 = ax 5.0
        for (int i = 0; i < 6; i++) {
            down.add(make_sample(5.0f, 0, 9.81f, 0, 0, 0));
        }
        // 1개 = ax -100 (작은 spike)
        down.add(make_sample(-100.0f, 0, 9.81f, 0, 0, 0));
        // 1개 = ax 100 (큰 spike)
        down.add(make_sample(100.0f, 0, 9.81f, 0, 0, 0));
        
        ImuSample r = down.finalize();
        // 정렬: [-100, 5, 5, 5, 5, 5, 5, 100]
        // 양 끝 제외 → [5, 5, 5, 5, 5, 5] → 평균 = 5.0
        printf("  ax = %.4f (둘 다 제거 → 5.0 기대)\n", r.ax);
        if (!approx(r.ax, 5.0f, 1e-5f)) {
            printf("[FAIL]\n");
            return 1;
        }
        printf("  OK (양쪽 spike 제거)\n");
    }
    
    // ─── Test 4: 노이즈 평균화 ───
    printf("\n[Test 4] 가우시안 노이즈 → 평균이 진짜 값으로\n");
    {
        ImuDownsampler down;
        // 진짜 값 = 9.81 + 노이즈 (-0.1 ~ +0.1)
        float values[8] = {9.71f, 9.85f, 9.78f, 9.83f, 
                           9.80f, 9.82f, 9.79f, 9.87f};
        for (int i = 0; i < 8; i++) {
            down.add(make_sample(0, 0, values[i], 0, 0, 0));
        }
        
        ImuSample r = down.finalize();
        printf("  az = %.4f (진짜값 9.81 가까이)\n", r.az);
        if (std::abs(r.az - 9.81f) > 0.02f) {
            printf("[FAIL] 평균 정확도\n");
            return 1;
        }
        printf("  OK (노이즈 평균화)\n");
    }
    
    // ─── Test 5: 8개 미만 → invalid ───
    printf("\n[Test 5] 8개 미만 (5개만) → invalid\n");
    {
        ImuDownsampler down;
        for (int i = 0; i < 5; i++) {
            down.add(make_sample(1, 1, 1, 0, 0, 0));
        }
        if (down.is_full()) { printf("[FAIL] 5개에 is_full\n"); return 1; }
        
        ImuSample r = down.finalize();
        if (r.valid) { printf("[FAIL] invalid 안 됨\n"); return 1; }
        printf("  OK (불완전 → invalid)\n");
    }
    
    // ─── Test 6: reset 후 재사용 ───
    printf("\n[Test 6] reset 후 재사용\n");
    {
        ImuDownsampler down;
        for (int i = 0; i < 8; i++) {
            down.add(make_sample(7, 0, 0, 0, 0, 0));
        }
        down.reset();
        if (down.count() != 0) { printf("[FAIL] reset\n"); return 1; }
        
        for (int i = 0; i < 8; i++) {
            down.add(make_sample(3, 0, 0, 0, 0, 0));
        }
        ImuSample r = down.finalize();
        if (!approx(r.ax, 3.0f, 1e-5f)) { 
            printf("[FAIL] reset 후 잘못 결과 %.4f\n", r.ax); 
            return 1; 
        }
        printf("  OK\n");
    }
    
    printf("\n[OK] ImuDownsampler 검증 통과\n");
    return 0;
}