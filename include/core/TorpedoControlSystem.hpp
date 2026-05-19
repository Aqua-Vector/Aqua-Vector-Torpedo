#ifndef TORPEDO_CONTROL_SYSTEM_HPP_
#define TORPEDO_CONTROL_SYSTEM_HPP_

#include <thread>
#include <atomic>
#include <chrono>

#include "ModeMux.hpp"
#include "ActuatorManager.hpp"
#include "ManualSource.hpp"
#include "AutoSource.hpp"
#include "NetworkManager.hpp"
#include "TorpedoParser.hpp"
#include "STMControlParser.hpp"
#include "UartLink.hpp"
#include "GenericPacket.hpp"
#include "Payloads.hpp"
#include "ControlDataValidator.hpp"

/**
 * @brief 패킷 타입 별칭 정의
 */
using TorpedoPacket = GenericPacket<TorpedoTelemetryPayload, uint16_t>;
using STMPacket = GenericPacket<ControlPayload, uint8_t>;

/**
 * @brief TorpedoControlSystem
 * 시스템의 전체 흐름을 관장하는 클래스
 * 100Hz 주기로 루프를 돌며 비동기 통신 스레드와 제어 로직을 조율
 */
class TorpedoControlSystem {
private:
	// 핵심 컴포넌트 참조
	ModeMux& mode_mux_;
	ActuatorManager& actuator_manager_;
	ManualSource& manual_source_;
	AutoSource& auto_source_;

	// 통신 매니저 참조
	NetworkManager<TorpedoParser, UartLink, TorpedoPacket>& gcs_manager_;
	NetworkManager<STMControlParser, UartLink, STMPacket>& stm32_manager_;

	// 데이터 저장소
	Mailbox<FeedbackPayload> stm32_feedback_mb_;

	// 스레드 및 상태 관리
	std::atomic<bool> is_running_;
	std::thread main_thread_;

	// 타이밍 설정
	const uint32_t loop_period_us_ = 10000;

public:
	/**
	 * @brief 의존성 주입을 통한 생성자
	 */
	TorpedoControlSystem(
			ModeMux& mode_mux,
			ActuatorManager& actuator_manager,
			ManualSource& manual_source,
			AutoSource& auto_source,
			NetworkManager<TorpedoParser, UartLink, TorpedoPacket>& gcs_manager,
			NetworkManager<STMControlParser, UartLink, STMPacket>& stm32_manager
			);

	~TorpedoControlSystem();

	// 복사 및 이동 금지 (스레드 관리 객체)
	TorpedoControlSystem(const TorpedoControlSystem&) = delete;
	TorpedoControlSystem& operator = (const TorpedoControlSystem&) = delete;

	/**
	 * @brief 시스템 초기화 (하드웨어 및 통신 링크 점검)
	 */
	bool init();

	/**
	 * @brief 메인 제어 루프 시작
	 */
	bool start();

	/**
	 * @brief 시스템 안전 종료
	 */
	void stop();

private:
	/**
	 * @brief 100Hz 주기 타이밍 루프
	 */
	void mainLoopTask();

	/**
	 * @brief 단일 제어 사이클 실행 (Mux 판단 -> 데이터 획득 -> STM32/Actuator 명령)
	 */
	void processControlCycle(uint64_t current_time_ms);

	/**
	 * @brief STM32로부터 피드백 수신 내부 콜백
	 */
	void onStm32FeedbackReceived(const FeedbackPayload& payload, uint64_t timestamp_ms);
};

#endif /* TORPEDO_CONTROL_SYSTEM_HPP_*/
