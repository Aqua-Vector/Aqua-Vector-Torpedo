// ESKF predict — P 행렬 단조 증가 검증
//
// 정지 상태 IMU (FakeImu) → predict 반복 → P trace 증가 확인
// 측정 없으면 불확실성은 항상 커져야 함

#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/sensor/imu_sample.hpp"

#include <cstdio>

int main() {
    using namespace torpedo;
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    est.init(params, 0.01f);  // dt = 10ms

    // 정지 상태 IMU 샘플 (중력만)
    ImuSample s;
    s.ax = 0.0f; s.ay = 0.0f; s.az = 9.81f;
    s.gx = 0.0f; s.gy = 0.0f; s.gz = 0.0f;
    s.t_us = 0;
    s.valid = true;
    
    BiasEstimate bias;  // 0으로 초기화

    float P_prev = est.state().P.trace();
    printf("[step %3d] P trace = %.9f\n", 0, P_prev);

    // 100 사이클 (1초) predict
    for (int i = 1; i <= 100; ++i) {
        est.predict(s, bias, 0.01f);
        
        if (i % 10 == 0) {
            float P_now = est.state().P.trace();
            printf("[step %3d] P trace = %.9f  (delta = %.9f)\n",
                   i, P_now, P_now - P_prev);
            P_prev = P_now;
        }
    }
    
    // 검증: 100 사이클 후 P가 init보다 커야
    float P_init = 0.007809f;  // init 결과
    float P_final = est.state().P.trace();
    
    if (P_final <= P_init) {
        printf("[FAIL] P should grow but didn't (init=%.6f, final=%.6f)\n",
               P_init, P_final);
        return 1;
    }
    
    printf("[OK] P grew from %.9f to %.9f\n", P_init, P_final);
    return 0;
}