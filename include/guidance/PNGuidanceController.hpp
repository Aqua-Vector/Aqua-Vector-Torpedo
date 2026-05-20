#ifndef PN_GUIDANCE_CONTROLLER_HPP_
#define PN_GUIDANCE_CONTROLLER_HPP_

#include "GuidanceTypes.hpp"
#include "torpedo/domain/estimator/eskf_state.hpp"
#include <Eigen/Dense>

class PNGuidanceController {
private:
	/**
	 * @brief 라디안 각도를 -PI ~ PI 범위로 정규화
	 */
	float normalizeAngle(float angle) const;

	float prev_los_angle_;
	bool is_first_run_;

	const float N_GAIN = 3.0f;
	const float K_P = 1.0f;

public:
	PNGuidanceController();
	~PNGuidanceController() = default;

	/**
	 * @brief PN 기반 조향각 계산
	 * @param my_state ESKF에서 추정한 현재 나의 상태 (위치, 자세 등)
	 * @param target_pos 추적할 타겟(또는 PIP)의 좌표
	 * @param dt 경과 시간(초)
	 * @return 최종 조향각 (Degree, -25, +25)
	 */
	float calculateSteering(const torpedo::domain::EskfState& my_state, const Eigen::Vector2f& target_pos, float dt);

	void reset();
};

#endif /* PN_GUIDANCE_CONTROLLER_HPP_ */
