#include "ModeMux.hpp"
#include <algorithm>

ModeMux::ModeMux(Mailbox<ControlState>* manual_source, Mailbox<ControlState>* auto_source)
	: current_mode_(SystemMode::MANUAL), manual_source_(manual_source), auto_source_(auto_source) {
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
	// 1. 하드웨어 락다운 또는 소프트웨어 FAILSAFE 상태면 Safe 반환
	if (current_mode_ == SystemMode::LOCKDOWN || current_mode_ == SystemMode::FAILSAFE) {
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
	
	// 데이터가 없거나, 시스템 시계가 꼬였거나(미래 데이터), 타임아웃이 발생한 경우
	if (last_update == 0 || current_time_ms < last_update || (current_time_ms - last_update) > WATCHDOG_TIMEOUT_MS) {
		current_mode_ = SystemMode::FAILSAFE;
		return &safe_mailbox_;
	}

	// 4. 모든 조건 만족 시 해당 소스 주소 반환
	return target_source;
}
