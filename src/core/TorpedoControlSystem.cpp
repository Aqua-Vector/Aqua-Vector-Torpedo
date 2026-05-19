#include "TorpedoControlSystem.hpp"
#include <iostream>
#include <iomanip>

TorpedoControlSystem::TorpedoControlSystem(
		ModeMux& mode_mux,
		ActuatorManager& actuator_manager,
     		ManualSource& manual_source,
     		AutoSource& auto_source,
    		NetworkManager<TorpedoParser, UartLink, TorpedoPacket>& gcs_manager,
		NetworkManager<STMControlParser, UartLink, STMPacket>& stm32_manager
		)
	: mode_mux_(mode_mux), 
	actuator_manager_(actuator_manager),
	manual_source_(manual_source),
	auto_source_(auto_source),
	gcs_manager_(gcs_manager),
	stm32_manager_(stm32_manager),
	is_running_(false) {}

TorpedoControlSystem::~TorpedoControlSystem() {
	stop();
}

bool TorpedoControlSystem::init() {
	std::cout << "[TCS] Initializing System..." << std::endl;

	// 액추에이터 초기화
	if (actuator_manager_.initAll() != ErrorCode::OK) {
		std::cerr << "[TCS] Failed to initialize ActuatorManager" << std::endl;
		return false;
	}

	// 통신 매니저 시작 (내부 스레드 구동)
	if (!gcs_manager_.start() || !stm32_manager_.start()) {
		std::cerr << "[TCS] Failed to start NetworkManagers" << std::endl;
		return false;
	}

	std::cout << "[TCS] Initialization Successful" << std::endl;
	return true;
}

bool TorpedoControlSystem::start() {
	if (is_running_) return true;
	is_running_ = true;
	main_thread_ = std::thread(&TorpedoControlSystem::mainLoopTask, this);

	std::cout << "[TCS] Main Control Loop Started (100Hz)" << std::endl;
	return true;
}

void TorpedoControlSystem::stop() {
	if (!is_running_) return;

	is_running_ = false;
	if (main_thread_.joinable()) {
		main_thread_.join();
	}

	// 안전을 위한 액추에이터 Failsafe 트리거 (모든 출력 0)
	actuator_manager_.triggerFailSafe();

	// 통신 종료
	gcs_manager_.stop();
	stm32_manager_.stop();

	std::cout << "[TCS] System Stopped Safely" << std::endl;
}

void TorpedoControlSystem::mainLoopTask() {
	// 지터 방지를 위한 기준 시간 설정
	auto next_wakeup = std::chrono::steady_clock::now();
	uint32_t loop_count = 0;
	while (is_running_) {
		uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

		// 단일 제어 사이클 실행 
		processControlCycle(now_ms);

		// 주기적 디버그 출력
		if (++loop_count >= 100) {
			SystemMode mode = mode_mux_.getMode();
			std::cout << "[TCS Status] Mode: " << static_cast<int>(mode) << " | Time: " << now_ms << "ms" << "\n";
			loop_count = 0;
		}

		next_wakeup += std::chrono::microseconds(loop_period_us_);

		if (std::chrono::steady_clock::now() > next_wakeup) {
			next_wakeup = std::chrono::steady_clock::now();
		} else {
			std::this_thread::sleep_until(next_wakeup);
		}
	}
}

void TorpedoControlSystem::processControlCycle(uint64_t current_time_ms) {
	// Mux에게 현재 유효한 메일박스 주소 물어봄
	auto* active_mailbox = mode_mux_.getActiveMailbox(current_time_ms);

	// 결정된 메일박스에서 제어 명령 획득
	ControlState target_state;
	uint64_t timestamp;
	if (!active_mailbox->fetch(target_state, timestamp)) {
		return;
	}

	// 값 검증
	ControlDataValidator::sanitize(target_state);

	// 하위 구동계로 명령 송신
	STMPacket stm_pkt;
	stm_pkt.msg_id = 0x10;
	stm_pkt.payload.velocity = target_state.velocity;
	stm_pkt.payload.rudder = target_state.rudder;
	stm_pkt.payload.elevator = target_state.elevator;
	stm32_manager_.send(stm_pkt);

	// 로컬 액추에이터 구동
	ErrorCode err = actuator_manager_.applyControl(target_state, 0.01f);

	// 하드웨어 치명적 에러 발생 시 시스템 락다운 트리거
	if (err != ErrorCode::OK) {
		mode_mux_.notifyHardwareError();
	}
}

void TorpedoControlSystem::onStm32FeedbackReceived(const FeedbackPayload& payload, uint64_t timestamp_ms) {
	// TODO: GuidanceManager가 이 메일 박스 주소를 참조
	stm32_feedback_mb_.update(payload, timestamp_ms);
}
