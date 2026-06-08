#ifndef GUIDANCE_MANAGER_HPP_
#define GUIDANCE_MANAGER_HPP_

#include "TargetStateEstimator.hpp"
#include "GuidancePhaseManager.hpp"
#include "IGuidanceController.hpp"
#include "HybridGuidanceController.hpp"
#include "ControlTypes.hpp"
#include <optional>
#include <memory>

namespace guidance {

class GuidanceManager {
private:
	TargetStateEstimator tse_;
	GuidancePhaseManager gpm_;
	std::unique_ptr<IGuidanceController> controller_;

	uint64_t last_lidar_ts_ = 0;
	float dt_acc_ = 0.0f;
	const float DEFAULT_SPEED = 200.0f;

public:
	GuidanceManager();
	~GuidanceManager() = default;

	/**
	 * @brief 매 사이클 호출되어 최종 유도 명령 산출
	 * @param my_state ESKF에서 제공하는 현재 나의 위치/자세 상태
	 * @param lidar_pos 통제소에서 수신한 타겟 좌표
	 * @param lidar_ts 타겟 좌표의 타임스탬프 (중복 필터링용)
	 * @param terminal_trigger 통제소에서 수신한 종말 유도 전환 명령
	 * @param dt 경과 시간
	 * @return 결정된 제어 명령
	 */
	ControlState update(const torpedo::domain::EskfState& my_state, 
	                    const std::optional<Eigen::Vector2f>& lidar_pos, 
	                    uint64_t lidar_ts,
	                    bool terminal_trigger, 
	                    float dt);

	GuidancePhase getPhase() const { return gpm_.getCurrentPhase(); }
	const TargetState& getTargetState() const { return tse_.getState(); }

	/**
	 * @brief 시스템 초기화
	 */
	void reset();
};

} // namespace guidance

#endif /* GUIDANCE_MANAGER_HPP_ */
