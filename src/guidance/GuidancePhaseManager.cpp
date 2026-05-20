#include "GuidancePhaseManager.hpp"

GuidancePhase GuidancePhaseManager::evaluatePhase(float dist_to_target, bool terminal_trigger_flag) {
	if (dist_to_target < GuidanceConstants::INTERCEPT_THRESHOLD_M) {
		current_phase_ = GuidancePhase::INTERCEPTED;
		return current_phase_;
	}

	switch (current_phase_) {
		case GuidancePhase::STANDBY:
			if (dist_to_target > 0.0f) {
				current_phase_ = GuidancePhase::MIDCOURSE;
			}
			break;

		case GuidancePhase::MIDCOURSE:
			if (terminal_trigger_flag) {
				current_phase_ = GuidancePhase::TERMINAL;
			}
			break;

		case GuidancePhase::TERMINAL:
			break;

		case GuidancePhase::INTERCEPTED:
			break;
	}

	return current_phase_;
}

