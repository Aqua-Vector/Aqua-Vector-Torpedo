#ifndef HYBRID_GUIDANCE_CONTROLLER_HPP_
#define HYBRID_GUIDANCE_CONTROLLER_HPP_

#include "IGuidanceController.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>

namespace guidance {

/**
 * @brief Unified Hybrid Guidance (PN + PPG)
 * 모든 연산을 프로젝트 표준인 [Y:전진, X:우측(CW+)] 좌표계에서 통합 수행합니다.
 */
class HybridGuidanceController : public IGuidanceController {
private:
    float prev_los_angle_cw_ = 0.0f;
    bool is_first_run_ = true;

    // 제어 파라미터
    const float N_GAIN = 4.0f;             // PN 게인
    const float TRANSITION_START_M = 2.0f;  // PN -> PPG 전환 시작 거리
    const float TRANSITION_END_M = 0.5f;    // PPG 전용 구간 시작 거리
    const float MAX_STEER = 25.0f;          // 최대 조향각 (deg)

    float normalizeAngle(float rad) const {
        while (rad > M_PI) rad -= 2.0f * M_PI;
        while (rad < -M_PI) rad += 2.0f * M_PI;
        return rad;
    }

public:
    HybridGuidanceController() { reset(); }

    float calculateSteering(const torpedo::domain::EskfState& my_state, 
                            const Eigen::Vector2f& target_pos, 
                            float dt) override {
        if (dt <= 0.0f) return 0.0f;

        // 1. 기본 상태 추출
        Eigen::Vector2f my_pos = my_state.p.head<2>();
        Eigen::Vector2f d_pos = target_pos - my_pos;
        float dist = d_pos.norm();
        if (dist < 0.001f) return 0.0f;

        // 2. 현재 시선각 계산 (LOS Angle, Y축 기준 CW+)
        float current_los_cw = std::atan2(d_pos.x(), d_pos.y());

        // 3. 내 현재 헤딩 계산 (Heading, Y축 기준 CW+)
        Eigen::Vector3f forward_v = my_state.q * Eigen::Vector3f(0, 1, 0);
        float my_yaw_cw = std::atan2(forward_v.x(), forward_v.y());

        // --- A. PPG (Pure Pursuit) 성분 계산 ---
        float heading_error_rad = normalizeAngle(current_los_cw - my_yaw_cw);
        float ppg_steer_cw = heading_error_rad * (180.0f / M_PI);

        // --- B. PN (Proportional Navigation) 성분 계산 ---
        float pn_steer_cw = ppg_steer_cw; // 기본값
        if (!is_first_run_) {
            float los_rate = normalizeAngle(current_los_cw - prev_los_angle_cw_) / dt;
            float speed = my_state.v.head<2>().norm();
            if (speed < 0.1f) speed = 0.1f;

            // a_n = N * V * los_rate (가로 가속도)
            // delta = (L/V^2) * a_n => (L/V) * N * los_rate
            const float L = 0.17f; // Wheelbase
            float pn_rad = (L / speed) * N_GAIN * los_rate;
            pn_steer_cw = pn_rad * (180.0f / M_PI);
        }
        prev_los_angle_cw_ = current_los_cw;
        is_first_run_ = false;

        // --- C. 블렌딩 (Distance-based Weight) ---
        float pn_weight = 1.0f;
        if (dist > TRANSITION_START_M) pn_weight = 1.0f;
        else if (dist < TRANSITION_END_M) pn_weight = 0.0f;
        else pn_weight = (dist - TRANSITION_END_M) / (TRANSITION_START_M - TRANSITION_END_M);

        float final_steer_cw = (pn_weight * pn_steer_cw) + ((1.0f - pn_weight) * ppg_steer_cw);

        return std::clamp(final_steer_cw, -MAX_STEER, MAX_STEER);
    }

    void reset() override {
        prev_los_angle_cw_ = 0.0f;
        is_first_run_ = true;
    }
};

} // namespace guidance

#endif
