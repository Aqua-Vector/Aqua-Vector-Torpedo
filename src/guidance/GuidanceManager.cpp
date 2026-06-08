#include "GuidanceManager.hpp"

namespace guidance {

GuidanceManager::GuidanceManager() {
        controller_ = std::make_unique<HybridGuidanceController>();
        reset();
}

void GuidanceManager::reset() {
        gpm_.reset();
        tse_.reset();
        if (controller_) controller_->reset();
        last_lidar_ts_ = 0;
        dt_acc_ = 0.0f;
}

ControlState GuidanceManager::update(const torpedo::domain::EskfState& my_state, 
                                    const std::optional<Eigen::Vector2f>& lidar_pos, 
                                    uint64_t lidar_ts,
                                    bool terminal_trigger, 
                                    float dt) {
        ControlState cmd = {0.0f, 0.0f, 0.0f, 0};
        dt_acc_ += dt;

        // 1. 유도 페이즈 결정 (최우선)
        float dist_to_target = 9999.0f;
        if (tse_.getState().is_valid) {
            dist_to_target = (tse_.getState().pos - my_state.p.head<2>()).norm();
        }
        GuidancePhase phase = gpm_.evaluatePhase(dist_to_target, terminal_trigger);

        // 2. 페이즈에 따른 타겟 상태 업데이트 전략
        if (phase == GuidancePhase::TERMINAL) {
            // [종말 유도] 실측 중단 및 순수 추측항법(DR) 모드 진입
            tse_.predictVirtualState(dt);
        } else {
            // [중기 유도] 실측 데이터(Lidar) 기반 추정 시도
            bool is_valid_lidar = lidar_pos.has_value() && !std::isnan(lidar_pos->x()) && !std::isnan(lidar_pos->y());
            if (is_valid_lidar && lidar_ts > last_lidar_ts_) {
                tse_.updateFromLidar(*lidar_pos, dt_acc_);
                last_lidar_ts_ = lidar_ts;
                dt_acc_ = 0.0f;
            } else {
                tse_.predictVirtualState(dt);
            }
        }

        // 3. 페이즈별 PN 유도 타겟 설정
        // [사용자 요구사항] hw_test_kinetick_pn_guidance 와 동일하게 추측항법 위치로 직접 유도
        Eigen::Vector2f pn_target;
        if (phase == GuidancePhase::TERMINAL) {
            // 종말 유도: 타겟의 현재 추측항법(DR) 위치를 목표로 유도 (PIP 대신 안정성 우선)
            pn_target = tse_.getState().pos;
        } else {
            // 중기 유도: 수신된 Lidar 좌표 사용 (단, NaN이 아닐 때만)
            bool is_current_valid = lidar_pos.has_value() && !std::isnan(lidar_pos->x());
            
            if (is_current_valid && lidar_ts == last_lidar_ts_) {
                pn_target = *lidar_pos;
            } else {
                pn_target = tse_.getState().pos;
            }
        }

        // 4. PN(비례항법) 알고리즘 실행
        float steering_angle = 0.0f;
        if (tse_.getState().is_valid && controller_) {
            steering_angle = controller_->calculateSteering(my_state, pn_target, dt);
        }

        // 5. 최종 제어 명령 출력 결정
        if (phase == GuidancePhase::INTERCEPTED) {
                cmd.velocity = 0.0f;
                cmd.rudder = 0.0f;
        } else if (phase == GuidancePhase::STANDBY) {
                cmd.velocity = 0.0f;
                cmd.rudder = 0.0f;
        } else {
                // MIDCOURSE 또는 TERMINAL
                cmd.velocity = DEFAULT_SPEED;
                cmd.rudder = steering_angle;
        }

        return cmd;
}

} // namespace guidance
