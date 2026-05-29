#include "PNGuidanceController.hpp"
#include <cmath>
#include <algorithm>

PNGuidanceController::PNGuidanceController() {
	reset();
}

void PNGuidanceController::reset() {
	prev_los_angle_ = 0.0f;
	is_first_run_ = true;
}

float PNGuidanceController::calculateSteering(const torpedo::domain::EskfState& my_state, const Eigen::Vector2f& target_pos, float dt) {
	if (dt <= 0.0f) return 0.0f;

	Eigen::Matrix3f R = my_state.q.toRotationMatrix();
	// [수정] Y축이 전진축이므로, Navigation Frame에서의 헤딩은 Body-Y(2번째 열)의 각도임
	float my_yaw = std::atan2(R(1,1), R(0,1));

	Eigen::Vector2f relative_pos = target_pos - my_state.p.head<2>();
	float current_los = std::atan2(relative_pos.y(), relative_pos.x());

	if (is_first_run_) {
		prev_los_angle_ = current_los;
		is_first_run_ = false;

		float heading_error = normalizeAngle(current_los - my_yaw);
		return std::clamp(static_cast<float>(heading_error * (180.0f / M_PI) * K_P), -GuidanceConstants::MAX_STEER_DEG, GuidanceConstants::MAX_STEER_DEG);

	}

	float los_diff = normalizeAngle(current_los - prev_los_angle_);
	float los_rate = los_diff / dt;
	prev_los_angle_ = current_los;

	float heading_error = normalizeAngle(current_los - my_yaw);

	float steering_rad = (heading_error * K_P) + (los_rate * N_GAIN);

	float steering_deg = steering_rad * (180.0f / M_PI);

	return std::clamp(steering_deg, -GuidanceConstants::MAX_STEER_DEG, GuidanceConstants::MAX_STEER_DEG);
}

float PNGuidanceController::normalizeAngle(float angle) const {
	while (angle > M_PI) angle -= 2.0f * M_PI;
	while (angle < -M_PI) angle += 2.0f * M_PI;
	return angle;
}
