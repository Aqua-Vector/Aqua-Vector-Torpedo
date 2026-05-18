#include "ModeMux.hpp"
#include "ControlDataValidator.hpp"

ModeMux::ModeMux(std::shared_ptr<IControlDataSource> manual_source, std::shared_ptr<IControlDataSource> auto_source)
	: current_mode_(SystemMode::MANUAL), last_update_time_ms_(0), manual_source_(manual_source), auto_source_(auto_source) {
		applyFailsafeState();
}

void ModeMux::setMode(SystemMode new_mode) {
	current_mode_ = new_mode;
}

SystemMode ModeMux::getMode() const {
	return current_mode_;
}

void ModeMux::applyFailsafeState() {
	last_valid_state_.velocity = 0.0f;
	last_valid_state_.rudder = 0.0f;
	last_valid_state_.elevator = 0.0f;
}

ControlState ModeMux::processControlLoop(uint64_t current_time_ms) {
	// FAILSAFE 락다운 확인 -> 수동 복구 전까지 새 데이터 무시
	if (current_mode_ == SystemMode::FAILSAFE) {
		applyFailsafeState();
		ControlDataValidator::sanitize(last_valid_state_);
		return last_valid_state_;
	}

	ControlState fetch_state;
	uint64_t data_timestamp = 0;
	bool is_data_updated = false;

	// 모드에 따른 데이터 소스 접근
	if (current_mode_ == SystemMode::MANUAL && manual_source_) {
		is_data_updated = manual_source_->fetchLatestState(fetch_state, data_timestamp);
	} else if (current_mode_ == SystemMode::AUTO && auto_source_) {
		is_data_updated = auto_source_->fetchLatestState(fetch_state, data_timestamp);
	}

	// Watchdog 로직
	if (is_data_updated) {
		if (current_time_ms >= data_timestamp) {
			const uint64_t elapsed_time = current_time_ms - data_timestamp;

			if (elapsed_time <= WATCHDOG_TIMEOUT_MS) {
				last_valid_state_ = fetch_state;
				last_update_time_ms_ = data_timestamp;
			} else {
				current_mode_ = SystemMode::FAILSAFE;
			}
		} else {
			current_mode_ = SystemMode::FAILSAFE;
		}
	} else {
		if (current_time_ms > WATCHDOG_TIMEOUT_MS) {
			current_mode_ = SystemMode::FAILSAFE;
		}
	}

	// 루프 도중 FAILSAFE로 변경된 경우 처리
	if (current_mode_ == SystemMode::FAILSAFE) {
		applyFailsafeState();
	}

	ControlDataValidator::sanitize(last_valid_state_);

	return last_valid_state_;
}
