#include "AutoSource.hpp"
#include "ControlDataValidator.hpp"

bool AutoSource::fetchLatestState(ControlState& out_state, uint64_t& out_timestamp) {
	return mailbox_.fetch(out_state, out_timestamp);
}

void AutoSource::updateTargetState(const ControlState& new_state, uint64_t timestamp) {
	ControlState safe_state = new_state;

	safe_state.last_update_time_ms = timestamp;

	ControlDataValidator::sanitize(safe_state);

	mailbox_.update(safe_state, timestamp);
}
