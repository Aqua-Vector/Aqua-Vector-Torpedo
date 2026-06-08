#include "PNGuidanceController.hpp"
#include <cmath>
#include <algorithm>

namespace guidance {

PNGuidanceController::PNGuidanceController() {
	reset();
}

void PNGuidanceController::reset() {
	prev_los_angle_ = 0.0f;
	is_first_run_ = true;
}

float PNGuidanceController::calculateSteering(const torpedo::domain::EskfState& my_state, const Eigen::Vector2f& target_pos, float dt) {
	if (dt <= 0.0f) return 0.0f;

	// 1. 상태 변수 추출
	Eigen::Vector2f my_pos = my_state.p.head<2>();
	Eigen::Vector2f my_vel = my_state.v.head<2>();
	float V_m = my_vel.norm();
	if (V_m < 0.1f) V_m = 0.1f; // Zero-division 방지

	// 2. 상대 기하학 계산
	Eigen::Vector2f relative_pos = target_pos - my_pos;
	float range = relative_pos.norm();
	// [수정] atan2(x, y)를 사용하여 오른쪽을 양수(+)로 정의 (CW+ 표준)
	float current_los = std::atan2(relative_pos.x(), relative_pos.y());

	// 3. LOS Rate (lambda_dot) 계산
	if (is_first_run_) {
		prev_los_angle_ = current_los;
		is_first_run_ = false;
		return 0.0f; // 첫 루프는 변화율 측정 불가
	}

	float los_diff = normalizeAngle(current_los - prev_los_angle_);
	float los_rate = los_diff / dt;
	prev_los_angle_ = current_los;

	// 4. Closing Velocity (V_c) 계산
	// V_c = - d(range)/dt
	float V_c = V_m;
	if (range > 0.001f) {
		// 내 속도 벡터(my_vel)의 타겟 방향 투영
		V_c = (my_vel.x() * (relative_pos.x() / range)) + (my_vel.y() * (relative_pos.y() / range));
	}
	if (V_c < 0.1f) V_c = V_m; 

	// 5. PN 수식: a_n = N * V_c * los_rate
	const float N = 4.0f;
	float lateral_accel = N * V_c * los_rate;

	// 6. 가속도를 조향각(Degree)으로 변환
	const float WHEEL_BASE = 0.17f;
	float steering_rad = (WHEEL_BASE * lateral_accel) / (V_m * V_m);
	float steering_deg = steering_rad * (180.0f / M_PI);

	// 7. [안전 장치] 만약 타겟과의 각도 오차가 너무 크면(예: 45도 이상), PN보다는 타겟을 향해 먼저 꺾음
	Eigen::Matrix3f R = my_state.q.toRotationMatrix();
	// [수정] 내 헤딩도 atan2(x, y)로 계산하여 CW+ 일치
	float my_yaw = std::atan2(R(0,1), R(1,1));
	float heading_error = normalizeAngle(current_los - my_yaw);
	
	if (std::abs(heading_error) > (45.0f * M_PI / 180.0f)) {
		// Pure Pursuit 성분 살짝 가미 (안정성)
		steering_deg += heading_error * (180.0f / M_PI) * 0.5f;
	}

	return std::clamp(steering_deg, -GuidanceConstants::MAX_STEER_DEG, GuidanceConstants::MAX_STEER_DEG);
}

float PNGuidanceController::normalizeAngle(float angle) const {
	while (angle > M_PI) angle -= 2.0f * M_PI;
	while (angle < -M_PI) angle += 2.0f * M_PI;
	return angle;
}

} // namespace guidance
