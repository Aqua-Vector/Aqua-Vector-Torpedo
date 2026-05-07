// ESKF predict — yaw 회전 검증
//
// gz = 1 rad/s 입력 → 1초 후 yaw ≈ 1 rad (57.3°) 확인
// quaternion 적분의 정확성 검증

#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/sensor/imu_sample.hpp"

#include <cstdio>
#include <cmath>

int main() {
    using namespace torpedo;
    using namespace torpedo::domain;

    EskfEstimator est;
    EskfInitParams params;
    est.init(params, 0.01f);

    // gz = 1 rad/s 회전 (정지 + yaw 회전)
    ImuSample s;
    s.ax = 0.0f; s.ay = 0.0f; s.az = 9.81f;
    s.gx = 0.0f; s.gy = 0.0f; s.gz = 1.0f;  // yaw 1 rad/s
    s.t_us = 0;
    s.valid = true;
    
    BiasEstimate bias;

    // 100 사이클 (1초) predict
    for (int i = 0; i < 100; ++i) {
        est.predict(s, bias, 0.01f);
    }
    
    // quaternion → yaw 추출
    Eigen::Quaternionf q = est.state().q;
    
    // ZYX Euler에서 yaw (z 회전)
    // yaw = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))
    float yaw = std::atan2(
        2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())
    );
    
    printf("[Yaw 회전 1초 후]\n");
    printf("  q = (w=%.4f, x=%.4f, y=%.4f, z=%.4f)\n",
           q.w(), q.x(), q.y(), q.z());
    printf("  q norm = %.6f (1.0이어야)\n", q.norm());
    printf("  yaw = %.4f rad = %.2f deg\n", yaw, yaw * 180.0f / 3.14159f);
    printf("  expected ≈ 1.0 rad = 57.30 deg\n");
    
    // 검증
    float yaw_error = std::abs(yaw - 1.0f);
    float norm_error = std::abs(q.norm() - 1.0f);
    
    printf("\n  yaw error  = %.6f rad\n", yaw_error);
    printf("  norm error = %.6f\n", norm_error);
    
    if (yaw_error > 0.01f) {  // 0.01 rad ≈ 0.6° 허용
        printf("[FAIL] yaw 적분 부정확\n");
        return 1;
    }
    if (norm_error > 1e-5f) {
        printf("[FAIL] quaternion 정규화 실패\n");
        return 1;
    }
    
    printf("[OK] yaw 회전 검증 통과\n");
    return 0;
}