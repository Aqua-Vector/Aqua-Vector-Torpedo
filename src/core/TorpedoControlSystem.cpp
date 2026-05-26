#include "core/TorpedoControlSystem.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "protocol/GenericPacket.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/ProtocolPolicies.hpp"
#include "protocol/ProtocolIds.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

TorpedoControlSystem::TorpedoControlSystem(
		ModeMux& mode_mux,
		ActuatorManager& actuator_manager,
		ManualSource& manual_source,
		AutoSource& auto_source,
		torpedo::IImu& imu,
		torpedo::domain::EskfEstimator& eskf,
		torpedo::domain::RpsPositionTracker& rps_tracker,
		ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>>& gcs_manager,
		ITxNetworkManager<STMPacket>& stm32_manager
		)
	: mode_mux_(mode_mux), 
	actuator_manager_(actuator_manager),
	manual_source_(manual_source),
	auto_source_(auto_source),
	imu_(imu),
	eskf_estimator_(eskf),
	rps_tracker_(rps_tracker),
	gcs_manager_(gcs_manager),
	stm32_manager_(stm32_manager),
	is_running_(false) {
		bias_estimate_.b_a.setZero();
		bias_estimate_.b_g.setZero();
	}

TorpedoControlSystem::~TorpedoControlSystem() {
	stop();
}

bool TorpedoControlSystem::init() {
	std::cout << "[TCS] Initializing System..." << std::endl;

	// IMU 초기화
	if (!imu_.init()) {
		std::cerr << "[TCS] Failed to initialize IMU" << std::endl;
		return false;
	}

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

	// [추가] 5초간 정지 상태 캘리브레이션 수행
	std::cout << "[TCS] Starting Static Calibration (5 seconds)..." << std::endl;
	torpedo::domain::BiasCalibrator cal;
	cal.start(5.0f, 100); // 5초, 100Hz

	auto cal_start = std::chrono::steady_clock::now();
	while (!cal.is_done()) {
		torpedo::ImuSample s;
		if (imu_.read(s)) {
			cal.add_sample(s);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
		// 타임아웃 방지 (10초 이상 걸리면 중단)
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::seconds>(now - cal_start).count() > 10) {
			std::cerr << "[TCS] Calibration Timeout!" << std::endl;
			break;
		}
	}

	if (cal.is_done()) {
		auto res = cal.finalize();
		if (res.success) {
			bias_estimate_ = res.bias;
			// ESKF 초기 자세 설정 (원본 ESKF를 건드리지 않으려면 re-init 사용)
			torpedo::domain::EskfInitParams params;
			params.q0 = res.q0;
			eskf_estimator_.init(params, 0.01f);
			
			std::cout << "[TCS] Calibration Successful!" << std::endl;
			std::cout << " - Bias Accel: " << bias_estimate_.b_a.transpose() << std::endl;
			std::cout << " - Bias Gyro: " << bias_estimate_.b_g.transpose() << std::endl;
		}
	} else {
		std::cerr << "[TCS] Calibration Failed!" << std::endl;
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
	imu_.shutdown();

	std::cout << "[TCS] System Stopped Safely" << std::endl;
}

void TorpedoControlSystem::mainLoopTask() {
	// 지터 방지를 위한 기준 시간 설정
	auto next_wakeup = std::chrono::steady_clock::now();
	uint32_t loop_count = 0;
	while (is_running_) {
		uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

		// IMU 데이터 읽기 및 ESKF Predict
		torpedo::ImuSample imu_sample;
		if (imu_.read(imu_sample)) {
			// [수정] 가속도 데이터는 사용하지 않도록 0으로 설정 (사용자 요청: 가속도 말고 heading만 사용)
			imu_sample.ax = 0.0f;
			imu_sample.ay = 0.0f;
			imu_sample.az = 0.0f;

			eskf_estimator_.predict(imu_sample, bias_estimate_, 0.01f);

			// [수정] MiniIMU의 절대 각도(Yaw)를 LPF 적용하여 RPS 트래커 전용으로 보관
			latest_imu_yaw_ = yaw_lpf_.updateAngle(imu_sample.yaw);
		}

		// 단일 제어 사이클 실행 
		processControlCycle(now_ms);

		// 주기적 디버그 출력
		if (++loop_count >= 100) {
			SystemMode mode = mode_mux_.getMode();
			const auto& pos = rps_tracker_.getPosition();
			float speed = rps_tracker_.getSpeed();
			
			// 마지막 제어 명령 peek를 위해 Mailbox fetch 대신 Guidance 결과 직접 로깅 고려 가능하나,
			// 여기서는 간단히 상태만 출력
			std::cout << "[TCS Status] Mode: " << static_cast<int>(mode) 
					  << " | Pos: (" << pos.x() << ", " << pos.y() << ")"
					  << " | Spd: " << speed << " m/s\n";
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
	// 1. GCS로부터 최신 데이터(Lidar) 획득
	ControlStationPayload gcs_data;
	uint64_t gcs_ts;
	bool has_gcs = gcs_data_mb_.fetch(gcs_data, gcs_ts);

	// 2. STM32로부터 최신 피드백 획득 및 RPS 트래커 업데이트
	FeedbackPayload stm_fb;
	uint64_t stm_ts;
	if (stm32_feedback_mb_.fetch(stm_fb, stm_ts)) {
		float raw_speed = (stm_fb.m1_rps - stm_fb.m2_rps) * 0.5f * RPS_TO_MPS;
		float filtered_speed = speed_lpf_.update(raw_speed);
		
		// [수정] ESKF의 드리프트 되는 자세 대신, MiniIMU의 절대 Yaw를 사용하여 위치 적분
		Eigen::Quaternionf q_stable = Eigen::Quaternionf(Eigen::AngleAxisf(latest_imu_yaw_, Eigen::Vector3f::UnitZ()));
		rps_tracker_.update(filtered_speed, q_stable, 0.01f);
	}

	// 데이터 유효성 검사 (500ms 이상 지연 시 데이터 없음으로 처리)
	std::optional<Eigen::Vector2f> target_pos = std::nullopt;
	bool terminal_trigger = false;

	if (has_gcs && (current_time_ms - gcs_ts) < 500) {
		// [치명적 버그 수정] gcs_data.target_x는 타겟(목적지)입니다.
		// 이것을 내 위치(lidar_pos)로 착각하여 rps_tracker를 덮어쓰면 
		// 내 위치 == 타겟 위치가 되어 거리가 0이 되고, 유도 알고리즘이 도착한 것으로 착각해 속도를 0으로 끕니다.
		target_pos = Eigen::Vector2f(gcs_data.target_x, gcs_data.target_y);
		terminal_trigger = (gcs_data.flags & 0x01);
		
		// 실제 LiDAR 보정이 필요하다면 gcs_data.torpedo_x, torpedo_y를 사용해야 합니다.
		// HIL 테스트 환경에서는 가상 타겟만 주고 있으므로 보정 로직을 생략합니다.
	}

	// 3. 유도 알고리즘 실행 (GuidanceManager)
	// ESKF의 자세와 RPS 트래커의 위치를 조합한 하이브리드 상태 생성
	torpedo::domain::EskfState hybrid_state = eskf_estimator_.state();
	hybrid_state.p = rps_tracker_.getPosition();
	
	// [수정] 속도 벡터를 Nav Frame으로 변환 (자세 q 적용)
	float speed_val = rps_tracker_.getSpeed();
	hybrid_state.v = eskf_estimator_.state().q * Eigen::Vector3f(speed_val, 0.0f, 0.0f);

	ControlState auto_cmd = guidance_manager_.update(
			hybrid_state, 
			target_pos, 
			terminal_trigger, 
			0.01f
	);

	// [중요] 타임스탬프를 명시적으로 현재 시간으로 설정하여 ModeMux 워치독 방지
	auto_cmd.last_update_time_ms = current_time_ms;
	auto_source_.updateTargetState(auto_cmd, current_time_ms);

	// 4. Mux에게 현재 유효한 메일박스 주소 물어봄
	SystemMode prev_mode = mode_mux_.getMode();
	auto* active_mailbox = mode_mux_.getActiveMailbox(current_time_ms);
	SystemMode current_mode = mode_mux_.getMode();

	if (prev_mode != current_mode) {
		std::cout << "[TCS] Mode Changed: " << static_cast<int>(prev_mode) << " -> " << static_cast<int>(current_mode) 
				  << " (Last Auto Update: " << auto_source_.getMailbox()->getLastUpdateTime() << ")" << std::endl;
	}

	// 결정된 메일박스에서 제어 명령 획득
	ControlState target_state;
	uint64_t timestamp;
	if (active_mailbox->fetch(target_state, timestamp)) {
		// 값 검증
		ControlDataValidator::sanitize(target_state);

		// [수정] 기존 성공한 hw_test_stm32.cpp 방식에 따라 직접 직렬화 및 송신 시도
		// NetworkManager의 비동기 큐 대신 직접 Link를 사용하는 것은 구조상 어려우므로, 
		// stm32_manager_.send() 내부에서 쓰이는 것과 동일한 직렬화 과정을 검증합니다.
		
		STMPacket stm_pkt;
		stm_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL; 
		stm_pkt.payload.velocity = target_state.velocity;
		stm_pkt.payload.rudder = target_state.rudder;
		stm_pkt.payload.elevator = target_state.elevator;
		
		// NetworkManager를 통한 송신
		stm32_manager_.send(stm_pkt);

		// 로컬 액추에이터 구동
		actuator_manager_.applyControl(target_state, 0.01f);

		// 디버그 로깅 강화: 실제로 계산된 값이 0인지 확인
		if (current_time_ms % 500 < 10) {
			std::cout << "[TCS Cmd] Target -> V: " << target_state.velocity 
					  << " | R: " << target_state.rudder 
					  << " | Mode: " << static_cast<int>(mode_mux_.getMode()) << std::endl;
		}
	}

	// 6. 통제소로 좌표 정보 직접 송신 (10Hz)
	sendUplinkTelemetry(current_time_ms);
}

void TorpedoControlSystem::sendUplinkTelemetry(uint64_t current_time_ms) {
	static uint16_t seq_counter = 0;
	static uint64_t last_uplink_ms = 0;

	if (current_time_ms - last_uplink_ms < 100) return; // 10Hz
	last_uplink_ms = current_time_ms;

	// RPS 트래커 기반 위치 사용
	const auto& pos = rps_tracker_.getPosition();
	const auto& q = eskf_estimator_.state().q;
	Eigen::Matrix3f R = q.toRotationMatrix();
	float yaw = std::atan2(R(1, 0), R(0, 0));

	// 통제소 전용 업링크 패킷 (GenericPacket 구조 사용)
	GenericPacket<TorpedoUplinkPayload, uint16_t> pkt;
	pkt.header[0] = TelemetryPolicy::SYNC1;
	pkt.header[1] = TelemetryPolicy::SYNC2;
	pkt.msg_id = TelemetryPolicy::MSG_ID;
	pkt.length = sizeof(TorpedoUplinkPayload);
	
	pkt.payload.seq = seq_counter++;
	pkt.payload.p_x = pos.x();
	pkt.payload.p_y = pos.y();
	pkt.payload.yaw = yaw;
	pkt.payload.status_flags = static_cast<uint8_t>(guidance_manager_.getPhase());
	pkt.payload.reserved = 0;

	// CRC 계산 (Header 제외: msg_id + length + payload)
	pkt.crc = TelemetryPolicy::calculateCrc(&pkt.msg_id, 2 + pkt.length);

	// NetworkManager를 통한 송신
	gcs_manager_.send(pkt);

	// TX 로그 (1초 주기)
	/*
	static uint64_t last_log_ms = 0;
	if (current_time_ms - last_log_ms >= 1000) {
		last_log_ms = current_time_ms;
		std::cout << "[TX Uplink] Seq: " << pkt.payload.seq 
				  << " | Pos: (" << std::fixed << std::setprecision(2) << pkt.payload.p_x 
				  << ", " << pkt.payload.p_y << ")" 
				  << " | Yaw: " << std::setprecision(1) << (pkt.payload.yaw * 180.0f / 3.14159265f) << " deg" << std::endl;
	}
	*/
}

void TorpedoControlSystem::onStm32FeedbackReceived(const FeedbackPayload& payload, uint64_t timestamp_ms) {
	stm32_feedback_mb_.update(payload, timestamp_ms);

	// RPS 기반 속도 계산 (추후 RpsPositionTracker 등에서 사용 가능)
	// (void)speed; // Silence unused warning if needed, but we keep the logic for future use
	[[maybe_unused]] float speed = (payload.m1_rps - payload.m2_rps) * 0.5f * RPS_TO_MPS;
	
	// TODO: 팀원이 만든 ESKF를 수정하지 않고 속도를 반영할 별도의 Tracker를 사용할 예정입니다.
	// 현재는 속도 업데이트를 건너뜁니다.
}

void TorpedoControlSystem::onGcsDataReceived(const ControlStationPayload& payload, uint64_t timestamp_ms) {
	gcs_data_mb_.update(payload, timestamp_ms);
}
