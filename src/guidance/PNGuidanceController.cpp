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
	float current_los = std::atan2(relative_pos.y(), relative_pos.x());

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
	// 타겟이 정지해 있다고 가정할 때, V_c는 내 속도의 타겟 방향 투영 성분
	// V_c = - d(range)/dt
	float V_c = V_m;
	if (range > 0.001f) {
		V_c = (my_vel.x() * (relative_pos.x() / range)) + (my_vel.y() * (relative_pos.y() / range));
	}
	if (V_c < 0.1f) V_c = V_m; 

	// 5. 정통 PN 수식: a_n = N * V_c * los_rate
	// N_GAIN은 보통 3~5 사이의 값을 사용 (여기서는 4.0 적용)
	const float N = 4.0f;
	float lateral_accel = N * V_c * los_rate;

	// 6. 가속도를 조향각(Degree)으로 변환
	// delta = (L / V^2) * a_n  (L: Wheel Base, 약 0.17m)
	// 하지만 실제 차량 모델에서는 단순화하여 속도에 반비례하도록 매핑
	const float WHEEL_BASE = 0.17f;
	float steering_rad = (WHEEL_BASE * lateral_accel) / (V_m * V_m);
	
	// 과도한 조향 방지 및 Degree 변환
	float steering_deg = steering_rad * (180.0f / M_PI);

	// 7. [안전 장치] 만약 타겟과의 각도 오차가 너무 크면(예: 90도 이상), PN보다는 타겟을 향해 먼저 꺾음
	Eigen::Matrix3f R = my_state.q.toRotationMatrix();
	float my_yaw = std::atan2(R(1,1), R(0,1));
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
