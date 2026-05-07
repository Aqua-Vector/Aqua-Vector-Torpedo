// EskfState 컴파일 + 기본 동작 검증
//
// 빌드: g++ -std=c++17 -Wall -Iinclude -I/usr/include/eigen3 \
//             tests/test_eskf_state.cpp -o test_eskf_state
// 실행: ./test_eskf_state

#include "torpedo/domain/estimator/eskf_state.hpp"

#include <cstdio>
#include <cmath>

int main() {
    torpedo::domain::EskfState x;

    x.reset();

    printf("[OK] EskfState created and reset\n");
    printf("  p = (%.2f, %.2f, %.2f) m\n", x.p.x(), x.p.y(), x.p.z());
    printf("  v = (%.2f, %.2f, %.2f) m/s\n", x.v.x(), x.v.y(), x.v.z());
    printf("  q = (w=%.2f, x=%.2f, y=%.2f, z=%.2f)\n",
           x.q.w(), x.q.x(), x.q.y(), x.q.z());
    printf("  P trace = %.6f\n", x.P.trace());
    printf("  Q trace = %.6f\n", x.Q.trace());

    if (x.p.norm() > 1e-6f) { printf("[FAIL] p should be 0\n"); return 1; }
    if (x.v.norm() > 1e-6f) { printf("[FAIL] v should be 0\n"); return 1; }
    if (std::abs(x.q.w() - 1.0f) > 1e-6f) {
        printf("[FAIL] q should be identity\n");
        return 1;
    }
    if (x.P.norm() > 1e-6f) { printf("[FAIL] P should be 0\n"); return 1; }

    printf("[OK] All checks passed\n");
    return 0;
}