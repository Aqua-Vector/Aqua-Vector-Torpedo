#include "TargetStateEstimator.hpp"

void TargetStateEstimator::updateFromLidar(const Eigen::Vector2f& raw_pos, float dt) {
	if (!state_.is_valid) {
		state_.pos = raw_pos;
		state_.vel = Eigen::Vector2f::Zero();
		state_.is_valid = true;
		return;
	}

	if (dt <= 0.0f) return;

	Eigen::Vector2f instant_vel = (raw_pos - state_.pos) /dt;
	state_.vel = (lpf_alpha_ * instant_vel) + ((1.0f - lpf_alpha_) * state_.vel);
	state_.pos = raw_pos;
}

void TargetStateEstimator::predictVirtualState(float dt) {
	if (!state_.is_valid || dt <= 0.0f) return;
	state_.pos += state_.vel * dt;
}

Eigen::Vector2f TargetStateEstimator::calculatePIP(const Eigen::Vector2f& my_pos, float my_speed) const {
	if (!state_.is_valid) return my_pos;

	float dist = (state_.pos - my_pos).norm();

	float safe_speed = std::max(my_speed, GuidanceConstants::MIN_SAFE_SPEED_MPS);
	float time_to_interact = dist / safe_speed;

	return state_.pos + (state_.vel * time_to_interact);
}
