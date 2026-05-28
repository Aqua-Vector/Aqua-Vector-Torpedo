#include "ManualSource.hpp"
#include "ControlDataValidator.hpp"

bool ManualSource::fetchLatestState(ControlState& out_state, uint64_t& out_timestamp) {
	return mailbox_.fetch(out_state, out_timestamp);
}

void ManualSource::onControlPacketReceived(const ControlPayload& payload, uint64_t timestamp) {
	ControlState new_state;

	new_state.velocity = payload.velocity;
	new_state.rudder = payload.rudder;
	new_state.elevator = payload.elevator;

	new_state.last_update_time_ms = timestamp;

	ControlDataValidator::sanitize(new_state);

	mailbox_.update(new_state, timestamp);
}
