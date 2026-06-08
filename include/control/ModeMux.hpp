#ifndef MODE_MUX_HPP_
#define MODE_MUX_HPP_

#include <cstdint>
#include "ControlTypes.hpp"
#include "Mailbox.hpp"

class ModeMux {
private:
	SystemMode current_mode_;
	
	Mailbox<ControlState>* manual_source_;
	Mailbox<ControlState>* auto_source_;
	Mailbox<ControlState> safe_mailbox_;

	static constexpr uint64_t WATCHDOG_TIMEOUT_MS = 5000;

	void setupSafeState();

public:
	ModeMux(Mailbox<ControlState>* manual_source, Mailbox<ControlState>* auto_source);
	~ModeMux() = default;

	void setMode(SystemMode new_mode);
	SystemMode getMode() const;

	// 하드웨어 에러 통지 (LOCKDOWN 전이)
	void notifyHardwareError();

	// 현재 활성화된 메일박스의 주소를 반환
	Mailbox<ControlState>* getActiveMailbox(uint64_t current_time_ms);
};

#endif /* MODE_MUX_HPP_ */
