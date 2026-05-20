#ifndef GUIDANCE_PHASE_MANAGER_HPP_
#define GUIDANCE_PHASE_MANAGER_HPP_

#include "GuidanceTypes.hpp"

class GuidancePhaseManager {
private:
	GuidancePhase current_phase_;

public:
	GuidancePhaseManager() : current_phase_(GuidancePhase::STANDBY) {}
	~GuidancePhaseManager() = default;

	/**
	 * @brief 현재 유도 페이즈를 판단하고 갱신
	 * @param dist_to_target 타겟과의 현재 거리(m)
	 * @param terminal_tirgger_flag 통제소에서 수신된 종말유도 전환 플래그 (true 시 전환)
	 * @return 결정된 유도 페이즈
	 */
	GuidancePhase evaluatePhase(float dist_to_target, bool terminal_tirgger_flag);
	GuidancePhase getCurrentPhase() const { return current_phase_; }
	void reset() { current_phase_ = GuidancePhase::STANDBY; }
};

#endif /* GUIDANCE_PHASE_MANAGER_HPP_ */
