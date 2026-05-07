// eskf_state.hpp — 9-state ESKF의 state 구조 정의
//
// Nominal state: [p, v, q] (실제 사용 값, 비선형 적분)
// Error state:   [δp, δv, δθ] ∈ ℝ⁹ (칼만 추정, 매 사이클 reset)
// Bias:          별도 EMA (ADR-001, 칼만 외부)
//
// 좌표계: nav frame (위치/속도), body→nav (q)
// 단위:   p [m], v [m/s], q [unit quaternion], P [variance]
//
// 참조:  ADR-005 (위치/속도 추정), ADR-002 (Quaternion)

#pragma once

#include <Eigen/Dense>

namespace torpedo::domain {

/**
 * 9-state ESKF의 state 구조.
 *
 * Nominal state (실제 사용 값):
 *   p, v: 3D 위치/속도 (nav frame)
 *   q:    body→nav quaternion
 *
 * Error covariance:
 *   P: 9×9 (δp 3 + δv 3 + δθ 3)
 *   Q: 9×9 process noise (init 시 1회 계산, 상수)
 *
 * Bias는 BiasEstimator (ADR-001)에서 별도 관리.
 * EskfState 안에 포함하지 않음.
 */
struct EskfState {
    // -------- Nominal state --------
    Eigen::Vector3f p;          // 위치 (nav frame, m)
    Eigen::Vector3f v;          // 속도 (nav frame, m/s)
    Eigen::Quaternionf q;       // 자세 (Hamilton, body→nav)

    // -------- Error covariance --------
    Eigen::Matrix<float, 9, 9> P;

    // -------- Process noise (init 시 1회 계산) --------
    Eigen::Matrix<float, 9, 9> Q;

    /// 초기 상태로 reset
    void reset() {
        p.setZero();
        v.setZero();
        q.setIdentity();
        P.setZero();
        Q.setZero();
    }
};

} // namespace torpedo::domain