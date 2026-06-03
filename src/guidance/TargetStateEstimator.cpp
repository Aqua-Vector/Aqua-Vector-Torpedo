#include "guidance/TargetStateEstimator.hpp"
#include <iostream>

namespace guidance {

TargetStateEstimator::TargetStateEstimator() {
	reset();
}

void TargetStateEstimator::reset() {
	is_initialized_ = false;
	current_state_.pos.setZero();
	current_state_.vel.setZero();
	current_state_.is_valid = false;
}

void TargetStateEstimator::updateFromLidar(const Eigen::Vector2f& raw_pos, float dt_sec) {
	if (!is_initialized_) {
		current_state_.pos = raw_pos;
		current_state_.vel.setZero();
		current_state_.is_valid = true;
		is_initialized_ = true;
		return;
	}

	if (dt_sec > 0.0f) {
		// [Target DR] 더미 데이터 특성을 활용한 속도 추출
		// 데이터가 100% 정확하므로 노이즈 필터링 없이 즉시 수용 (alpha = 1.0)
		Eigen::Vector2f instant_vel = (raw_pos - current_state_.pos) / dt_sec;
		
		current_state_.vel = instant_vel;
		current_state_.pos = raw_pos;
	}
	
	current_state_.is_valid = true;
}

void TargetStateEstimator::predictVirtualState(float dt_sec) {
	if (!is_initialized_ || !current_state_.is_valid) return;

	// [Blind Phase] 수학적으로 완벽한 직선 방정식 적용 (Dead Reckoning)
	// P(t) = P_last + V_target * dt
	current_state_.pos += current_state_.vel * dt_sec;
}

Eigen::Vector2f TargetStateEstimator::calculatePIP(const Eigen::Vector2f& my_pos, float my_speed) const {
	if (!current_state_.is_valid) return my_pos;

	// 타겟과 나의 상대 거리
	float dist = (current_state_.pos - my_pos).norm();

	// 요격 예상 시간 (Time to Intercept)
	float safe_speed = std::max(my_speed, GuidanceConstants::MIN_SAFE_SPEED_MPS);
	float time_to_intercept = dist / safe_speed;

	// PIP = 현재 타겟 위치 + 타겟 속도 * 요격 시간
	return current_state_.pos + (current_state_.vel * time_to_intercept);
}

} // namespace guidance
