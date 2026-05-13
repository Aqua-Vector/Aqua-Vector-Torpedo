// EskfEstimator 컴파일 + init 검증

#include "torpedo/domain/estimator/eskf_estimator.hpp"

#include <cstdio>

int main() {
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    params.p0 = Eigen::Vector3f(1.0f, 2.0f, 0.0f);  // 임의 시작 위치

    est.init(params, 0.01f);  // dt = 10ms

    const EskfState& x = est.state();

    printf("[OK] EskfEstimator init\n");
    printf("  p = (%.2f, %.2f, %.2f) m\n", x.p.x(), x.p.y(), x.p.z());
    printf("  P trace = %.6f\n", x.P.trace());
    printf("  Q trace = %.9f\n", x.Q.trace());

    // 검증
    if ((x.p - params.p0).norm() > 1e-6f) {
        printf("[FAIL] p not initialized\n");
        return 1;
    }
    if (x.P.trace() <= 0) {
        printf("[FAIL] P should be positive\n");
        return 1;
    }
    if (x.Q.trace() <= 0) {
        printf("[FAIL] Q should be positive\n");
        return 1;
    }

    printf("[OK] All checks passed\n");
    return 0;
}