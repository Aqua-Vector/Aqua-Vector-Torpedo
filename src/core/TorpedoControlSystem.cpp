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

bool TorpedoControlSystem::init(bool skip_calibration) {
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

	if (skip_calibration) {
		std::cout << "[SYS] Skipping Static Calibration as requested." << std::endl;
		bias_estimate_.b_a.setZero();
		bias_estimate_.b_g.setZero();
		torpedo::domain::EskfInitParams params;
		eskf_estimator_.init(params, 0.01f);
	} else {
		// [추가] 5초간 정지 상태 캘리브레이션 수행
		std::cout << "[SYS] Starting Static Calibration (5 seconds)..." << std::endl;
		torpedo::domain::BiasCalibrator cal;
		cal.start(5.0f, 100); // 5초, 100Hz

		auto cal_start = std::chrono::steady_clock::now();
		uint64_t last_t_us = 0;
		int last_reported_sec = -1;

		while (!cal.is_done()) {
			torpedo::ImuSample s;
			if (imu_.read(s)) {
				// 중복 데이터 방지: 타임스탬프가 새로울 때만 샘플 추가
				if (s.t_us != last_t_us) {
					cal.add_sample(s);
					last_t_us = s.t_us;

					// 1초 단위로 진행 상황 출력
					int current_sec = static_cast<int>(cal.progress() * 5.0f);
					if (current_sec != last_reported_sec) {
						std::cout << "[SYS] Calibration Progress: " << (current_sec + 1) << " / 5s" << std::endl;
						last_reported_sec = current_sec;
					}
				}
			}
			
			// CPU 점유율 조절
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			
			// 타임아웃 방지 (15초 이상 걸리면 중단)
			auto now = std::chrono::steady_clock::now();
			if (std::chrono::duration_cast<std::chrono::seconds>(now - cal_start).count() > 15) {
				std::cerr << "[SYS] Calibration Timeout! (Got " << cal.finalize().samples_used << " samples)" << std::endl;
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
				
				std::cout << "[SYS] Calibration Successful! (" << res.samples_used << " samples)" << std::endl;
				std::cout << "[SYS] Bias Accel: " << bias_estimate_.b_a.transpose() << std::endl;
				std::cout << "[SYS] Bias Gyro: " << bias_estimate_.b_g.transpose() << std::endl;
			}
		} else {
			std::cerr << "[SYS] Calibration Failed! (Only " << cal.finalize().samples_used << " samples)" << std::endl;
		}
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

			// [주석 처리] 자이로를 통한 Yaw 측정 중단
			// eskf_estimator_.predict(imu_sample, bias_estimate_, 0.01f);

			// [주석 처리] MiniIMU의 절대 각도(Yaw) 업데이트 중단
			// latest_imu_yaw_ = yaw_lpf_.updateAngle(imu_sample.yaw);
		}

		// 루프 실행 시간 측정 시작
		auto loop_start = std::chrono::steady_clock::now();

		// 단일 제어 사이클 실행 
		processControlCycle(now_ms);

		auto loop_end = std::chrono::steady_clock::now();
		last_loop_elapsed_us_.store(static_cast<uint32_t>(
			std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count()));

		// 주기적 디버그 출력
		if (++loop_count >= 100) {
			SystemMode mode = mode_mux_.getMode();
			const auto& pos = rps_tracker_.getPosition();
			float speed = rps_tracker_.getSpeed();
			
			// 마지막 제어 명령 peek를 위해 Mailbox fetch 대신 Guidance 결과 직접 로깅 고려 가능하나,
			// 여기서는 간단히 상태만 출력
			std::cout << "[TCS Status] Mode: " << static_cast<int>(mode) 
					  << " | Pos: (" << pos.x() << ", " << pos.y() << ")"
					  << " | Spd: " << speed << " m/s" << std::endl;
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
		// [수정] 조향각 피드백 저장 (내부 표준: 오른쪽이 양수)
		// STM32 내부 명령은 (-)가 오른쪽이지만, 서보 피드백(servo_pos)은 오른쪽일 때 1500보다 커지므로 그대로 사용
		latest_steering_yaw_ = (static_cast<float>(stm_fb.servo_pos) - 1500.0f) * 0.08f * STEERING_SCALE_FACTOR;

		// [수정] 속도 계산
		float raw_speed = (stm_fb.m1_rps - stm_fb.m2_rps) * 0.5f * RPS_TO_MPS;
		float filtered_speed = speed_lpf_.update(raw_speed);

		// [수정] 정확한 dt 계산 (피드백 타임스탬프 기반)
		uint64_t current_fb_us = stm_ts * 1000;
		float dt = 0.01f;
		if (last_feedback_time_us_ != 0 && current_fb_us > last_feedback_time_us_) {
			dt = static_cast<float>(current_fb_us - last_feedback_time_us_) / 1000000.0f;
			if (dt > 0.1f) dt = 0.01f; // 통신 끊김 시 예외 처리
		}
		last_feedback_time_us_ = current_fb_us;

		// 자이로 대신 STM32 조향 피드백을 이용한 Yaw 누적 (Dead Reckoning)
		// CW-positive yaw를 사용하되, Nav Frame 변환 시에는 CCW로 변환하여 사용
		float steer_rad = latest_steering_yaw_ * (3.14159265f / 180.0f);
		
		if (std::abs(filtered_speed) > 0.01f) {
			float dyaw = (filtered_speed / WHEEL_BASE) * std::tan(steer_rad) * dt;
			cumulative_yaw_rad_ += dyaw;
		}

		// Nav Frame으로 변환하기 위한 쿼터니언 생성 (Eigen은 CCW 기준이므로 부호 반전)
		Eigen::Quaternionf q_nav = Eigen::Quaternionf(Eigen::AngleAxisf(-cumulative_yaw_rad_, Eigen::Vector3f::UnitZ()));

		// 위치 업데이트
		rps_tracker_.update(filtered_speed, q_nav, dt);
	}

	// 데이터 유효성 검사 (500ms 이상 지연 시 데이터 없음으로 처리)
	std::optional<Eigen::Vector2f> target_pos = std::nullopt;
	bool terminal_trigger = false;

	if (has_gcs && (current_time_ms - gcs_ts) < 500) {
		target_pos = Eigen::Vector2f(gcs_data.target_x, gcs_data.target_y);
		terminal_trigger = (gcs_data.flags & 0x01);
		
		// [수정] GCS flags를 기반으로 모드 전환 (최우선 순위: 3: FAILSAFE)
		if (gcs_data.flags == 0x03 || (gcs_data.flags & 0x03) == 0x03) {
			if (mode_mux_.getMode() != SystemMode::FAILSAFE) {
				mode_mux_.setMode(SystemMode::FAILSAFE);
				std::cout << "[MODE] GCS Emergency Failsafe Triggered! (Flag: 0x" << std::hex << (int)gcs_data.flags << std::dec << ")" << std::endl;
			}
		} else if (gcs_data.flags == 0x01) {
			if (mode_mux_.getMode() != SystemMode::MANUAL) {
				mode_mux_.setMode(SystemMode::MANUAL);
				std::cout << "[MODE] Switched to MANUAL via GCS" << std::endl;
			}
		} else if (gcs_data.flags == 0x02) {
			if (mode_mux_.getMode() != SystemMode::AUTO) {
				mode_mux_.setMode(SystemMode::AUTO);
				std::cout << "[MODE] Switched to AUTO via GCS" << std::endl;
			}
		}
	}

	// 3. 유도 알고리즘 실행 (GuidanceManager)
	// ESKF의 자세와 RPS 트래커의 위치를 조합한 하이브리드 상태 생성
	torpedo::domain::EskfState hybrid_state = eskf_estimator_.state();
	hybrid_state.p = rps_tracker_.getPosition();
	
	// [수정] 속도 벡터를 Nav Frame으로 변환 (사용자 요청: Y축이 전진축)
	float speed_val = rps_tracker_.getSpeed();
	hybrid_state.v = eskf_estimator_.state().q * Eigen::Vector3f(0.0f, speed_val, 0.0f);

	ControlState auto_cmd = guidance_manager_.update(
			hybrid_state, 
			target_pos, 
			terminal_trigger, 
			0.01f
	);

	// [수정] Guidance(CCW+) 출력을 내부/GCS 표준(CW+)으로 변환
	auto_cmd.rudder = -auto_cmd.rudder;

	// [중요] 타임스탬프를 명시적으로 현재 시간으로 설정하여 ModeMux 워치독 방지
	auto_cmd.last_update_time_ms = current_time_ms;
	auto_source_.updateTargetState(auto_cmd, current_time_ms);

	// 4. Mux에게 현재 유효한 메일박스 주소 물어봄
	SystemMode prev_mode = mode_mux_.getMode();
	auto* active_mailbox = mode_mux_.getActiveMailbox(current_time_ms);
	SystemMode current_mode = mode_mux_.getMode();

	if (prev_mode != current_mode) {
		std::cout << "[TCS] Mode Changed: " << static_cast<int>(prev_mode) << " -> " << static_cast<int>(current_mode) 
				  << " (Last manual update: " << manual_source_.getMailbox()->getLastUpdateTime() << ")" << std::endl;
	}

	// 결정된 메일박스에서 제어 명령 획득
	ControlState target_state;
	uint64_t timestamp;
	if (active_mailbox->fetch(target_state, timestamp)) {
		// 값 검증
		ControlDataValidator::sanitize(target_state);

		STMPacket stm_pkt;
		stm_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL; 
		stm_pkt.payload.velocity = target_state.velocity;
		stm_pkt.payload.rudder = -target_state.rudder; // [수정] 내부(CW+)를 STM32(CCW+) 표준으로 변환하여 송신
		stm_pkt.payload.elevator = target_state.elevator;

		
		// NetworkManager를 통한 송신
		if (!stm32_manager_.send(stm_pkt)) {
			// 큐가 가득 찼을 때만 로그 (매번 찍으면 너무 많음)
			static uint32_t drop_count = 0;
			if (++drop_count % 100 == 1) {
				std::cerr << "[COMM] Warning: STM32 TX Queue Full! (Total drops: " << drop_count << ")" << std::endl;
			}
		} else {
            // [Add] STM32 송신 성공 로그 (1초 주기로 출력하여 전송 확인)
            static uint64_t last_stm_tx_ms = 0;
            if (current_time_ms - last_stm_tx_ms >= 1000) {
                std::cout << "[COMM] TX -> STM32 | V: " << std::fixed << std::setprecision(1) << stm_pkt.payload.velocity 
                          << " | R: " << stm_pkt.payload.rudder 
                          << " | E: " << stm_pkt.payload.elevator << std::endl;
                last_stm_tx_ms = current_time_ms;
            }
        }

		// 로컬 액추에이터 구동
		actuator_manager_.applyControl(target_state, 0.01f);

		// [수정] 100Hz 실시간 로그 출력 대신 1초(100회)에 한 번만 출력
		static uint32_t cmd_log_count = 0;
		if (++cmd_log_count >= 100) {
			std::cout << "[TCS] Command | V: " << std::fixed << std::setprecision(1) << target_state.velocity 
					  << " | R: " << target_state.rudder 
					  << " | Mode: " << static_cast<int>(mode_mux_.getMode()) 
					  << " | Age: " << (current_time_ms - timestamp) << "ms" << std::endl;
			cmd_log_count = 0;
		}
	} else {
		// 메일박스 fetch 실패 (이 경우는 거의 없어야 함)
		static uint32_t fetch_fail_count = 0;
		if (++fetch_fail_count % 100 == 1) {
			std::cerr << "[TCS Error] Active Mailbox Fetch Failed! Mode: " << static_cast<int>(current_mode) << std::endl;
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
	
	// [수정] 자이로 기반 Yaw 대신 누적된 조향 Yaw 사용
	float yaw = cumulative_yaw_rad_; 

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
	// [수정] 커맨드 -60f가 전진이므로, RPS 결과에 -를 붙여 양수 속도로 변환
	[[maybe_unused]] float speed = -(payload.m1_rps - payload.m2_rps) * 0.5f * RPS_TO_MPS;
	
	// TODO: 팀원이 만든 ESKF를 수정하지 않고 속도를 반영할 별도의 Tracker를 사용할 예정입니다.
	// 현재는 속도 업데이트를 건너뜁니다.
}

void TorpedoControlSystem::onGcsDataReceived(const ControlStationPayload& payload, uint64_t timestamp_ms) {
	gcs_data_mb_.update(payload, timestamp_ms);
}
