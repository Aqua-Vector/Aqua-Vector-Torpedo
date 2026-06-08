// bias_estimate.hpp — ESKF에 주입할 bias 값 구조체
//
// BiasEstimator (ADR-001 EMA)에서 추정한 결과를
// ESKF에 전달하기 위한 단순 데이터 구조.
//
// 단위:
//   b_a: m/s² (가속도 bias)
//   b_g: rad/s (자이로 bias)
//
// Layer: Domain (Layer 3)
// 참조: ADR-001 (Bias 시간 상수)

#pragma once

#include <Eigen/Dense>

namespace torpedo::domain {

struct BiasEstimate {
    Eigen::Vector3f b_a = Eigen::Vector3f::Zero();  // 가속도 bias
    Eigen::Vector3f b_g = Eigen::Vector3f::Zero();  // 자이로 bias
};

} // namespace torpedo::domain