#include "ModeMux.hpp"
#include <algorithm>
#include <iostream>

ModeMux::ModeMux(Mailbox<ControlState>* manual_source, Mailbox<ControlState>* auto_source)
	: current_mode_(SystemMode::STANDBY), manual_source_(manual_source), auto_source_(auto_source) {
	setupSafeState();
}

void ModeMux::setupSafeState() {
	ControlState safe_state = {0.0f, 0.0f, 0.0f, 0};
	// 타임스탬프 0은 데이터가 없음을 의미할 수 있지만, 
	// safe_mailbox는 항상 유효한(하지만 0인) 값을 주어야 하므로 1로 설정하거나 
	// fetch 로직에서 0을 체크하는 부분을 고려해야 함. 
	// 여기서는 단순히 모든 출력을 0으로 고정하는 용도.
	safe_mailbox_.update(safe_state, 1); 
}

void ModeMux::setMode(SystemMode new_mode) {
	// LOCKDOWN 상태에서는 외부에서 모드를 변경할 수 없도록 강제 (수동 복구 로직 필요 시 확장)
	if (current_mode_ != SystemMode::LOCKDOWN) {
		current_mode_ = new_mode;
	}
}

SystemMode ModeMux::getMode() const {
	return current_mode_;
}

void ModeMux::notifyHardwareError() {
	current_mode_ = SystemMode::LOCKDOWN;
}

Mailbox<ControlState>* ModeMux::getActiveMailbox(uint64_t current_time_ms) {
	// 1. 하드웨어 락다운, FAILSAFE, 또는 초기 STANDBY 상태면 Safe 반환
	if (current_mode_ == SystemMode::LOCKDOWN || 
		current_mode_ == SystemMode::FAILSAFE || 
		current_mode_ == SystemMode::STANDBY) {
		return &safe_mailbox_;
	}

	// 2. 현재 모드에 따른 대상 소스 결정
	Mailbox<ControlState>* target_source = (current_mode_ == SystemMode::AUTO) ? auto_source_ : manual_source_;

	if (!target_source) {
		current_mode_ = SystemMode::FAILSAFE;
		return &safe_mailbox_;
	}

	// 3. Watchdog 체크 (데이터 신선도 검사)
	uint64_t last_update = target_source->getLastUpdateTime();
	
	if (last_update != 0) {
		// [BUG FIX] 미세한 시간 역전(Clock Skew) 허용 로직 추가
		// 데이터 시각이 현재보다 미래인 경우, 100ms 이내라면 "매우 신선한 데이터"로 간주함.
		bool is_future_data = (last_update > current_time_ms);
		uint64_t skew_or_age = is_future_data ? (last_update - current_time_ms) : (current_time_ms - last_update);

		if ((is_future_data && skew_or_age > 100) || (!is_future_data && skew_or_age > WATCHDOG_TIMEOUT_MS)) {
			if (current_mode_ != SystemMode::FAILSAFE) {
				std::cerr << "[ModeMux] Watchdog Timeout! Mode: " << static_cast<int>(current_mode_) 
						<< " | Last update: " << last_update << " | Current: " << current_time_ms 
						<< " | Diff: " << (is_future_data ? "-" : "") << skew_or_age << "ms" << std::endl;
				current_mode_ = SystemMode::FAILSAFE;
			}
			return &safe_mailbox_;
		}
	}

	// 4. 모든 조건 만족 시 해당 소스 주소 반환
	return target_source;
}
