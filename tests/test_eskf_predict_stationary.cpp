// ESKF predict — 정지 상태 검증
//
// 정지 + 중력만 입력 → 위치/속도 그대로 유지 검증
// (중력이 정확히 보상되어야)

#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/sensor/imu_sample.hpp"

#include <cstdio>
#include <cmath>

int main() {
    using namespace torpedo;
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    params.p0 = Eigen::Vector3f(1.0f, 2.0f, 0.0f);  // 시작 위치
    est.init(params, 0.01f);

    // 정지 IMU (중력만)
    ImuSample s;
    s.ax = 0.0f; s.ay = 0.0f; s.az = 9.81f;  // body z = 위, 중력 측정
    s.gx = 0.0f; s.gy = 0.0f; s.gz = 0.0f;
    s.t_us = 0;
    s.valid = true;
    
    BiasEstimate bias;  // 0

    Eigen::Vector3f p_initial = est.state().p;
    Eigen::Vector3f v_initial = est.state().v;

    // 100 사이클 (1초) predict
    for (int i = 0; i < 100; ++i) {
        est.predict(s, bias, 0.01f);
    }
    
    Eigen::Vector3f p_after = est.state().p;
    Eigen::Vector3f v_after = est.state().v;
    
    printf("[정지 1초 후]\n");
    printf("  p initial = (%.4f, %.4f, %.4f)\n",
           p_initial.x(), p_initial.y(), p_initial.z());
    printf("  p after   = (%.4f, %.4f, %.4f)\n",
           p_after.x(), p_after.y(), p_after.z());
    printf("  v initial = (%.4f, %.4f, %.4f)\n",
           v_initial.x(), v_initial.y(), v_initial.z());
    printf("  v after   = (%.4f, %.4f, %.4f)\n",
           v_after.x(), v_after.y(), v_after.z());
    
    // 검증: 정지면 위치/속도 변화 없어야 (중력 정확히 상쇄)
    float p_drift = (p_after - p_initial).norm();
    float v_drift = v_after.norm();
    
    printf("\n  p drift = %.6f m\n", p_drift);
    printf("  v drift = %.6f m/s\n", v_drift);
    
    if (p_drift > 1e-4f) {
        printf("[FAIL] 정지 상태에서 위치 변동 (중력 보상 실패)\n");
        return 1;
    }
    if (v_drift > 1e-4f) {
        printf("[FAIL] 정지 상태에서 속도 변동\n");
        return 1;
    }
    
    printf("[OK] 정지 검증 통과\n");
    return 0;
}