#ifndef GUIDANCE_MANAGER_HPP_
#define GUIDANCE_MANAGER_HPP_

#include "TargetStateEstimator.hpp"
#include "GuidancePhaseManager.hpp"
#include "PNGuidanceController.hpp"
#include "ControlTypes.hpp"
#include <optional>

namespace guidance {

class GuidanceManager {
private:
	TargetStateEstimator tse_;
	GuidancePhaseManager gpm_;
	PNGuidanceController pnc_;

	const float DEFAULT_SPEED = -60.0f;

public:
	GuidanceManager();
	~GuidanceManager() = default;

	/**
	 * @brief 매 사이클 호출되어 최종 유도 명령 산출
	 * @param my_state ESKF에서 제공하는 현재 나의 위치/자세 상태
	 * @param lidar_pos 통제소에서 수신한 타겟 좌표
	 * @param terminal_trigger 통제소에서 수신한 종말 유도 전환 명령\
	 * @param dt 경과 시간
	 * @return 결정된 제어 명령
	 */
	ControlState update(const torpedo::domain::EskfState& my_state, const std::optional<Eigen::Vector2f>& lidar_pos, bool terminal_trigger, float dt);

	GuidancePhase getPhase() const { return gpm_.getCurrentPhase(); }
	const TargetState& getTargetState() const { return tse_.getState(); }

	/**
	 * @brief 시스템 초기화
	 */
	void reset();
};

} // namespace guidance

#endif /* GUIDANCE_MANAGER_HPP_ */
