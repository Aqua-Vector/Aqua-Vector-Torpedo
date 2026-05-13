// ESKF 종합 테스트 — predict ↔ update 사이클
//
// 시나리오:
//   - 정지 상태 IMU 사용 (단순화)
//   - predict 50회 (500ms) 후 LiDAR update 1회
//   - 5번 반복 (총 2.5초)
//
// 검증:
//   - predict 동안 P 증가
//   - update 후 P 감소
//   - 사이클 안정 (발산 없음)
//   - 위치 추정이 측정 근처 유지

#include "torpedo/domain/estimator/eskf_estimator.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

int main() {
    using namespace torpedo;
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    params.p0 = Eigen::Vector3f(0.0f, 0.0f, 0.0f);  // 출발 (0, 0)
    est.init(params, 0.01f);

    // 정지 IMU
    ImuSample s;
    s.ax = 0.0f; s.ay = 0.0f; s.az = 9.81f;
    s.gx = 0.0f; s.gy = 0.0f; s.gz = 0.0f;
    s.t_us = 0;
    s.valid = true;
    
    BiasEstimate bias;

    // LiDAR 측정 (실제로는 어뢰가 (0, 0)에서 안 움직인 상태)
    // 약간의 노이즈 가정 (±2cm)
    Eigen::Vector2f z_lidar(0.02f, -0.01f);

    printf("[ESKF 종합 테스트 — predict ↔ update 사이클]\n\n");
    printf("초기 상태:\n");
    printf("  p = (%.4f, %.4f) m\n", est.state().p.x(), est.state().p.y());
    printf("  P trace = %.6f\n\n", est.state().P.trace());

    // 사이클 5번 반복
    std::vector<float> P_history;
    P_history.push_back(est.state().P.trace());
    
    for (int cycle = 1; cycle <= 5; ++cycle) {
        printf("─── Cycle %d ───\n", cycle);
        
        // Phase 1: predict 50회 (500ms)
        for (int i = 0; i < 50; ++i) {
            est.predict(s, bias, 0.01f);
        }
        float P_predict_end = est.state().P.trace();
        Eigen::Vector3f p_predict_end = est.state().p;
        printf("  predict 50회 후: P = %.6f, p = (%.4f, %.4f)\n",
               P_predict_end, p_predict_end.x(), p_predict_end.y());
        
        // Phase 2: LiDAR update
        est.update_lidar(z_lidar);
        float P_update_end = est.state().P.trace();
        Eigen::Vector3f p_update_end = est.state().p;
        printf("  update 후:        P = %.6f, p = (%.4f, %.4f)\n",
               P_update_end, p_update_end.x(), p_update_end.y());
        
        // 검증: predict 후 P 증가, update 후 P 감소
        if (P_predict_end <= P_history.back()) {
            printf("[FAIL] cycle %d: predict 후 P 안 증가 (%.6f → %.6f)\n",
                   cycle, P_history.back(), P_predict_end);
            return 1;
        }
        if (P_update_end >= P_predict_end) {
            printf("[FAIL] cycle %d: update 후 P 안 감소 (%.6f → %.6f)\n",
                   cycle, P_predict_end, P_update_end);
            return 1;
        }
        
        P_history.push_back(P_update_end);
        printf("\n");
    }
    
    // ─── 종합 검증 ───
    printf("─── 종합 검증 ───\n");
    
    // 1. P가 발산하지 않고 어느 수준에 수렴
    float P_final = P_history.back();
    float P_initial = P_history.front();
    printf("  P 초기 = %.6f\n", P_initial);
    printf("  P 최종 = %.6f\n", P_final);
    
    // 2. 최종 P가 합리적 범위 (10x 미만 증가)
    if (P_final > 10.0f * P_initial) {
        printf("[FAIL] P 폭증 (%.2fx)\n", P_final / P_initial);
        return 1;
    }
    
    // 3. 위치 추정이 측정 근처
    Eigen::Vector3f p_final = est.state().p;
    float distance_to_z = std::sqrt(
        (p_final.x() - z_lidar.x()) * (p_final.x() - z_lidar.x()) +
        (p_final.y() - z_lidar.y()) * (p_final.y() - z_lidar.y())
    );
    printf("  최종 p = (%.4f, %.4f)\n", p_final.x(), p_final.y());
    printf("  측정까지 거리 = %.4f m\n", distance_to_z);
    
    if (distance_to_z > 0.10f) {  // 10cm 이내 (LiDAR 5cm × 2)
        printf("[FAIL] 위치 추정이 측정에서 너무 멀음\n");
        return 1;
    }
    
    // 4. NaN 폭발 없음
    if (!std::isfinite(P_final)) {
        printf("[FAIL] P NaN\n");
        return 1;
    }
    
    // 5. quaternion norm 유지
    float q_norm = est.state().q.norm();
    printf("  q norm = %.6f\n", q_norm);
    if (std::abs(q_norm - 1.0f) > 1e-4f) {
        printf("[FAIL] q norm = %.6f (1.0이어야)\n", q_norm);
        return 1;
    }
    
    printf("\n[OK] 종합 테스트 통과 — ESKF 사이클 동작 검증\n");
    printf("  ✓ predict 후 P 증가 (5/5 cycle)\n");
    printf("  ✓ update 후 P 감소 (5/5 cycle)\n");
    printf("  ✓ P 안정 (발산 없음)\n");
    printf("  ✓ 위치 추정이 측정 근처 유지\n");
    printf("  ✓ quaternion 정규화\n");
    return 0;
}