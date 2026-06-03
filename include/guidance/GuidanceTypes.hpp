#ifndef GUIDANCE_TYPES_HPP_
#define GUIDANCE_TYPES_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <algorithm>

/* 유도 페이즈 정의 */
enum class GuidancePhase : uint8_t {
	STANDBY,
	MIDCOURSE,
	TERMINAL,
	INTERCEPTED
};

/* 타겟 상태 정보 */
struct TargetState {
	Eigen::Vector2f pos = Eigen::Vector2f::Zero();
	Eigen::Vector2f vel = Eigen::Vector2f::Zero();
	bool is_valid = false;
};

/* 유효 상수 */
namespace GuidanceConstants {
	static constexpr float INTERCEPT_THRESHOLD_M = 0.2f; 	// 요격 판정 거리
	static constexpr float TERMINAL_SWITCH_DIST_M = 2.0f; 	// 종말 유도 전환 거리
	static constexpr float MAX_STEER_DEG = 30.0f; 		// 최대 조향각
	static constexpr float MIN_SAFE_SPEED_MPS = 0.5f; 	// PIP 계산용 최소 안전 속도  
}

#endif /* GUIDANCE_TYPES_HPP_ */
