#ifndef TORPEDO_CONTROL_SYSTEM_HPP_
#define TORPEDO_CONTROL_SYSTEM_HPP_

#include <thread>
#include <atomic>
#include <chrono>
#include <optional>

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
#include "utils/Mailbox.hpp"
#include "utils/LowPassFilter.hpp"

// 유도 및 추정 엔진 헤더 추가
#include "guidance/GuidanceManager.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "torpedo/sensor/iimu.hpp"

#include "protocol/GenericPacket.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/ProtocolPolicies.hpp"

/**
 * @brief 패킷 타입 별칭 정의
 */
using STMPacket = GenericPacket<ControlPayload, uint8_t>;

/**
 * @brief TorpedoControlSystem
 * 시스템의 전체 흐름을 관장하는 클래스
 */
class TorpedoControlSystem {
public:
	/**
	 * @brief 의존성 주입을 통한 생성자
	 */
	TorpedoControlSystem(
			ModeMux& mode_mux,
			ManualSource& manual_source,
			AutoSource& auto_source,
			torpedo::IImu& imu,
			torpedo::domain::EskfEstimator& eskf,
			torpedo::domain::RpsPositionTracker& rps_tracker,
			ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>>& gcs_manager,
			ITxNetworkManager<STMPacket>& stm32_manager
			);

	~TorpedoControlSystem();

	// 복사 및 이동 금지 (스레드 관리 객체)
	TorpedoControlSystem(const TorpedoControlSystem&) = delete;
	TorpedoControlSystem& operator = (const TorpedoControlSystem&) = delete;

	/**
	 * @brief 시스템 초기화 (하드웨어 및 통신 링크 점검)
	 */
	bool init(bool skip_calibration = false);

	/**
	 * @brief 메인 제어 루프 시작
	 */
	bool start();

	/**
	 * @brief 시스템 안전 종료
	 */
	void stop();

	/**
	 * @brief 시스템 실행 상태 확인
	 */
	bool isRunning() const { return is_running_.load(); }

	/**
	 * @brief STM32로부터 피드백 수신 콜백 (Public for Handlers)
	 */
	void onStm32FeedbackReceived(const FeedbackPayload& payload, uint64_t timestamp_ms);

	/**
	 * @brief GCS로부터 데이터 수신 콜백 (Public for Handlers)
	 */
	void onGcsDataReceived(const ControlStationPayload& payload, uint64_t timestamp_ms);

	/**
	 * @brief 유도 매니저 접근 (로그용)
	 */
	const guidance::GuidanceManager& getGuidanceManager() const { return guidance_manager_; }

	// 데이터 조회 메서드
	float getLatestSteeringYaw() const { return latest_steering_yaw_; }
	uint64_t getLatestFeedbackTime() const { return stm32_feedback_mb_.getLastUpdateTime(); }
	uint32_t getLoopElapsedUs() const { return last_loop_elapsed_us_.load(); }
	float getLatestCumulativeYaw() const { return cumulative_yaw_rad_; }

private:
	// 핵심 컴포넌트 참조
	ModeMux& mode_mux_;
	ManualSource& manual_source_;
	AutoSource& auto_source_;

	// 센서 및 유도/추정 엔진
	torpedo::IImu& imu_;
	guidance::GuidanceManager guidance_manager_;
	torpedo::domain::EskfEstimator& eskf_estimator_;
	torpedo::domain::RpsPositionTracker& rps_tracker_;
	torpedo::domain::BiasEstimate bias_estimate_;

	// 상태 변수
	float latest_imu_yaw_ = 0.0f;
	float imu_yaw_offset_ = 0.0f;
	std::atomic<bool> imu_data_received_{false};
	float latest_steering_yaw_ = 0.0f;
	float cumulative_yaw_rad_ = 0.0f;

	// LPF 필터
	utils::LowPassFilter speed_lpf_{0.2f};
	utils::LowPassFilter yaw_lpf_{0.3f};

	// 통신 매니저 및 데이터 저장소
	ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>>& gcs_manager_;
	ITxNetworkManager<STMPacket>& stm32_manager_;
	Mailbox<ControlStationPayload> gcs_data_mb_;
	Mailbox<FeedbackPayload> stm32_feedback_mb_;

	// 물리 상수
	// [수정] 0.0338(덜 감)과 0.0342(더 감)의 절충안으로 0.0340 적용
	const float WHEEL_RADIUS_MIDCOURSE = 0.0340f; 
	const float WHEEL_RADIUS_TERMINAL  = 0.0340f;
	const float WHEEL_BASE = 0.170f; 
	const float STEERING_SCALE_FACTOR = 0.3f;

	float getCurrentWheelRadius() const {
		if (guidance_manager_.getPhase() == GuidancePhase::TERMINAL) return WHEEL_RADIUS_TERMINAL;
		return WHEEL_RADIUS_MIDCOURSE;
	}
	float getRpsToMps() const { return 2.0f * 3.14159265f * getCurrentWheelRadius(); }

	// 스레드 및 상태 관리
	std::atomic<bool> is_running_;
	std::thread main_thread_;

	uint64_t last_feedback_time_us_ = 0;
	uint64_t last_gcs_ts_ = 0;
	const uint32_t loop_period_us_ = 10000; // 100Hz

protected:
	/**
	 * @brief 단일 제어 사이클 실행
	 */
	void processControlCycle(uint64_t current_time_ms);

	/**
	 * @brief 통제소로 좌표 정보 직접 송신 (10Hz)
	 */
	void sendUplinkTelemetry(uint64_t current_time_ms);

private:
	/**
	 * @brief 100Hz 주기 타이밍 루프
	 */
	void mainLoopTask();

	std::atomic<uint32_t> last_loop_elapsed_us_{0};
};

#endif /* TORPEDO_CONTROL_SYSTEM_HPP_ */
