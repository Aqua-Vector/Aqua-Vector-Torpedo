// ESKF update_lidar — 측정 보정 검증
//
// 시나리오:
// 1. 초기 위치 (1, 2) 로 init
// 2. LiDAR 측정 (1.05, 2.10) 도착 (5cm, 10cm 차이)
// 3. nominal 위치가 측정 방향으로 끌려가는지 확인
// 4. P trace가 줄어드는지 확인 (측정 보정 효과)

#include "torpedo/domain/estimator/eskf_estimator.hpp"

#include <cstdio>
#include <cmath>

int main() {
    using namespace torpedo;
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    params.p0 = Eigen::Vector3f(1.0f, 2.0f, 0.0f);
    est.init(params, 0.01f);

    // 초기 상태 확인
    Eigen::Vector3f p_before = est.state().p;
    float P_before = est.state().P.trace();

    printf("[update_lidar 검증]\n");
    printf("  초기 nominal p = (%.4f, %.4f)\n",
           p_before.x(), p_before.y());
    printf("  초기 P trace = %.6f\n\n", P_before);

    // LiDAR 측정 도착 (5cm, 10cm 차이)
    Eigen::Vector2f z_lidar(1.05f, 2.10f);
    printf("  측정값 z = (%.4f, %.4f)\n", z_lidar.x(), z_lidar.y());
    printf("  측정 잔차 = (%.4f, %.4f) m\n\n",
           z_lidar.x() - p_before.x(),
           z_lidar.y() - p_before.y());

    est.update_lidar(z_lidar);

    Eigen::Vector3f p_after = est.state().p;
    float P_after = est.state().P.trace();

    printf("  보정 후 p = (%.4f, %.4f)\n",
           p_after.x(), p_after.y());
    printf("  보정 후 P trace = %.6f\n\n", P_after);

    // ------- 검증 -------
    
    // 1. nominal이 측정 방향으로 이동했나?
    Eigen::Vector2f shift_x(p_after.x() - p_before.x(),
                             p_after.y() - p_before.y());
    Eigen::Vector2f innov(z_lidar.x() - p_before.x(),
                          z_lidar.y() - p_before.y());
    
    // shift는 innov와 같은 방향이어야 (K · innov)
    float dot = shift_x.dot(innov);
    if (dot <= 0) {
        printf("[FAIL] nominal이 측정 반대 방향으로 이동\n");
        return 1;
    }
    
    // shift 크기는 innov보다 작아야 (K < 1)
    if (shift_x.norm() >= innov.norm()) {
        printf("[FAIL] K > 1, 측정값 그대로 (예측 무시)\n");
        return 1;
    }
    
    // 2. P 줄어들었나? (측정으로 불확실성 감소)
    if (P_after >= P_before) {
        printf("[FAIL] P가 줄지 않음 (측정 효과 없음)\n");
        return 1;
    }
    
    // 3. 가까이 끌려갔는지 확인
    float distance_to_z = (Eigen::Vector2f(p_after.x(), p_after.y()) - z_lidar).norm();
    float distance_initial = innov.norm();
    
    printf("[검증]\n");
    printf("  측정까지 거리 (보정 전) = %.4f m\n", distance_initial);
    printf("  측정까지 거리 (보정 후) = %.4f m\n", distance_to_z);
    printf("  K 비율 = %.4f (0~1 사이여야)\n", shift_x.norm() / innov.norm());

    if (distance_to_z >= distance_initial) {
        printf("[FAIL] 측정 방향으로 안 끌려감\n");
        return 1;
    }
    
    printf("\n[OK] update_lidar 검증 통과\n");
    printf("  - nominal이 측정 방향으로 이동 ✓\n");
    printf("  - P 감소 (불확실성 줄음) ✓\n");
    printf("  - K가 0~1 (적절한 가중) ✓\n");
    return 0;
}