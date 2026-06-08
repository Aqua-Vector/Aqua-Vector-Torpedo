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
		ManualSource& manual_source,
		AutoSource& auto_source,
		torpedo::IImu& imu,
		torpedo::domain::EskfEstimator& eskf,
		torpedo::domain::RpsPositionTracker& rps_tracker,
		ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>>& gcs_manager,
		ITxNetworkManager<STMPacket>& stm32_manager
		)
	: mode_mux_(mode_mux), 
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
	if (!imu_.init()) return false;
	if (!gcs_manager_.start() || !stm32_manager_.start()) return false;

	if (skip_calibration) {
		bias_estimate_.b_a.setZero();
		bias_estimate_.b_g.setZero();
		torpedo::domain::EskfInitParams params;
		eskf_estimator_.init(params, 0.01f);
	} else {
		std::cout << "[SYS] Starting Static Calibration (5 seconds)..." << std::endl;
		torpedo::domain::BiasCalibrator cal;
		cal.start(5.0f, 100);
		auto cal_start = std::chrono::steady_clock::now();
		uint64_t last_t_us = 0;
		int last_reported_sec = -1;
		float sum_yaw = 0.0f;
		int yaw_samples = 0;

		while (!cal.is_done()) {
			torpedo::ImuSample s;
			bool read_success = imu_.read(s);
			if (read_success && s.t_us != last_t_us) {
				cal.add_sample(s);
				if (s.valid) { sum_yaw += s.yaw; yaw_samples++; }
				last_t_us = s.t_us;
				int current_sec = static_cast<int>(cal.progress() * 5.0f);
				if (current_sec != last_reported_sec) {
					std::cout << "[SYS] Calibration Progress: " << (current_sec + 1) << " / 5s" << std::endl;
					last_reported_sec = current_sec;
				}
			}

			// [수정] IMU 타임아웃 로직 추가 (강제 종료 방지)
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cal_start).count();
			if (elapsed_ms > 10000) { // 10초 타임아웃
				std::cerr << "\n[SYS] CRITICAL: IMU Calibration Timeout (10s)!" << std::endl;
				std::cerr << "[SYS] No valid data received from IMU. Check connection/power." << std::endl;
				return false; // 초기화 실패
			}
			
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}

		if (cal.is_done()) {
			auto res = cal.finalize();
			if (res.success) {
				bias_estimate_ = res.bias;
				if (yaw_samples > 0) imu_yaw_offset_ = sum_yaw / static_cast<float>(yaw_samples);
				torpedo::domain::EskfInitParams params;
				params.q0 = res.q0;
				eskf_estimator_.init(params, 0.01f);
			}
		}
	}
	return true;
}

bool TorpedoControlSystem::start() {
	if (is_running_) return true;
	is_running_ = true;
	main_thread_ = std::thread(&TorpedoControlSystem::mainLoopTask, this);
	return true;
}

void TorpedoControlSystem::stop() {
	if (!is_running_) return;
	is_running_ = false;
	if (main_thread_.joinable()) main_thread_.join();
	gcs_manager_.stop(); stm32_manager_.stop(); imu_.shutdown();
}

void TorpedoControlSystem::mainLoopTask() {
	auto next_wakeup = std::chrono::steady_clock::now();
	uint32_t loop_count = 0;
	while (is_running_) {
		uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		const float dt = 0.01f;

		// 1. IMU 데이터 최신화 (버퍼 비우기)
		torpedo::ImuSample imu_sample;
		while (imu_.read(imu_sample)) {
			if (imu_sample.valid) {
				const float diff = (imu_sample.yaw - imu_yaw_offset_);
				float raw_yaw = -diff; 
				while (raw_yaw > M_PI) raw_yaw -= 2.0f * M_PI;
				while (raw_yaw < -M_PI) raw_yaw += 2.0f * M_PI;
				
				// [수정] STANDBY 모드에서는 Yaw를 누적하지 않고 대기
				if (mode_mux_.getMode() != SystemMode::STANDBY) {
					latest_imu_yaw_ = raw_yaw;
					imu_data_received_ = true;
				} else {
					latest_imu_yaw_ = 0.0f;
					cumulative_yaw_rad_ = 0.0f;
				}
			}
		}

		// 2. 제어 사이클 실행
		processControlCycle(now_ms);

		// 3. 상태 로그 (100Hz -> 1Hz)
		if (++loop_count >= 100) {
			const auto& pos = rps_tracker_.getPosition(); 
			std::cout << "[TCS Status] Mode: " << static_cast<int>(mode_mux_.getMode()) 
				<< " | Pos: (" << std::fixed << std::setprecision(2) << pos.x() << ", " << pos.y() << ")"
				<< " | Spd: " << rps_tracker_.getSpeed() << " m/s | Yaw: " << (cumulative_yaw_rad_ * 180.0f / 3.14159f) << " deg" << std::endl;
			loop_count = 0;
		}

		next_wakeup += std::chrono::microseconds(loop_period_us_);
		std::this_thread::sleep_until(next_wakeup);
	}
}

void TorpedoControlSystem::onStm32FeedbackReceived(const FeedbackPayload& payload, uint64_t timestamp_ms) {
	stm32_feedback_mb_.update(payload, timestamp_ms);
	
	// [수정] STANDBY 상태에서는 위치 적분을 수행하지 않음
	if (mode_mux_.getMode() == SystemMode::STANDBY) {
		last_feedback_time_us_ = timestamp_ms * 1000;
		return;
	}

	uint64_t current_fb_us = timestamp_ms * 1000;
	float dt = 0.01f;
	if (last_feedback_time_us_ != 0 && current_fb_us > last_feedback_time_us_) {
		dt = static_cast<float>(current_fb_us - last_feedback_time_us_) / 1000000.0f;
	}
	last_feedback_time_us_ = current_fb_us;

	float raw_speed = (std::abs(payload.m1_rps) + std::abs(payload.m2_rps)) * 0.5f * getRpsToMps();
	float filtered_speed = speed_lpf_.update(raw_speed);

	if (imu_data_received_) {
		cumulative_yaw_rad_ = latest_imu_yaw_;
	} else {
		float steer_rad = latest_steering_yaw_ * (M_PI / 180.0f);
		cumulative_yaw_rad_ += (filtered_speed / WHEEL_BASE) * std::tan(steer_rad) * dt;
	}
	
	// [복구] YAW_CORRECTION_GAIN 제거: 추정치가 실제보다 적게 나오는 현상(Under-travel) 해결을 위해 보정치 1.0으로 복구
	float integration_yaw = cumulative_yaw_rad_;

	Eigen::Quaternionf q_nav(Eigen::AngleAxisf(-integration_yaw, Eigen::Vector3f::UnitZ()));
	rps_tracker_.update(filtered_speed, q_nav, dt);
}

void TorpedoControlSystem::processControlCycle(uint64_t current_time_ms) {
	torpedo::domain::EskfState rps_state;
	rps_state.p = rps_tracker_.getPosition();
	rps_state.q = Eigen::Quaternionf(Eigen::AngleAxisf(-cumulative_yaw_rad_, Eigen::Vector3f::UnitZ()));
	rps_state.v = rps_state.q * Eigen::Vector3f(0.0f, rps_tracker_.getSpeed(), 0.0f);

	ControlStationPayload gcs_data; uint64_t gcs_ts;
	bool has_gcs = gcs_data_mb_.fetch(gcs_data, gcs_ts);
	std::optional<Eigen::Vector2f> target_pos = std::nullopt;
	uint64_t lidar_ts = 0; bool gcs_terminal_flag = false;

	if (has_gcs && (current_time_ms - gcs_ts) < 500) {
		target_pos = Eigen::Vector2f(gcs_data.target_x, gcs_data.target_y);
		lidar_ts = gcs_ts;
		gcs_terminal_flag = (gcs_data.flags & 0x02);
		if (!std::isnan(gcs_data.torpedo_x) && !std::isnan(gcs_data.torpedo_y)) {
			eskf_estimator_.update_lidar(Eigen::Vector2f(gcs_data.torpedo_x, gcs_data.torpedo_y));
		}
		if ((gcs_data.flags & 0x03) == 0x03) mode_mux_.setMode(SystemMode::FAILSAFE);
		else if (gcs_data.flags & 0x02) mode_mux_.setMode(SystemMode::AUTO);
		else if (gcs_data.flags & 0x01) mode_mux_.setMode(SystemMode::MANUAL);
	}

	ControlState auto_cmd = guidance_manager_.update(rps_state, target_pos, lidar_ts, gcs_terminal_flag, 0.01f);
	
	// [핵심] 조향 발산 억제를 위한 LPF 적용 (3.3도 오차 대응)
	static float filtered_rudder = 0.0f;
	filtered_rudder = 0.5f * auto_cmd.rudder + 0.5f * filtered_rudder;
	auto_cmd.rudder = filtered_rudder;

	if (guidance_manager_.getPhase() == GuidancePhase::INTERCEPTED) {
		if (mode_mux_.getMode() != SystemMode::LOCKDOWN) mode_mux_.setMode(SystemMode::LOCKDOWN);
	}
	auto_source_.updateTargetState(auto_cmd, current_time_ms);

	auto* active_mailbox = mode_mux_.getActiveMailbox(current_time_ms);
	ControlState target_state; uint64_t ts;
	if (active_mailbox->fetch(target_state, ts)) {
		ControlDataValidator::sanitize(target_state);
		STMPacket stm_pkt; stm_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
		stm_pkt.payload.velocity = target_state.velocity;
		stm_pkt.payload.rudder = -target_state.rudder; 
		stm_pkt.payload.elevator = target_state.elevator;
		if (stm32_manager_.send(stm_pkt)) {
			// TX 로그 복구 (1초 주기)
			static uint64_t last_tx_ms = 0;
			if (current_time_ms - last_tx_ms >= 1000) {
				std::cout << "[COMM] TX -> STM32 | V: " << stm_pkt.payload.velocity << " | R: " << stm_pkt.payload.rudder << std::endl;
				last_tx_ms = current_time_ms;
			}
		}
	}
	sendUplinkTelemetry(current_time_ms);
}

void TorpedoControlSystem::sendUplinkTelemetry(uint64_t current_time_ms) {
	static uint16_t seq_counter = 0; static uint64_t last_uplink_ms = 0;
	if (current_time_ms - last_uplink_ms < 100) return;
	last_uplink_ms = current_time_ms;
	const auto& pos = rps_tracker_.getPosition();
	GenericPacket<TorpedoUplinkPayload, uint16_t> pkt;
	pkt.header[0] = TelemetryPolicy::SYNC1; pkt.header[1] = TelemetryPolicy::SYNC2;
	pkt.msg_id = TelemetryPolicy::MSG_ID; pkt.length = sizeof(TorpedoUplinkPayload);
	pkt.payload.seq = seq_counter++; pkt.payload.p_x = pos.x(); pkt.payload.p_y = pos.y();
	pkt.payload.yaw = cumulative_yaw_rad_;
	pkt.payload.status_flags = static_cast<uint8_t>(guidance_manager_.getPhase());
	pkt.crc = TelemetryPolicy::calculateCrc(&pkt.msg_id, 2 + pkt.length);
	
	if (pkt.payload.seq % 100 == 0) {
		std::cout << "[TELEMETRY] Seq: " << pkt.payload.seq << " | Pos: (" << pkt.payload.p_x << ", " << pkt.payload.p_y << ")" << std::endl;
	}
	gcs_manager_.send(pkt);
}

void TorpedoControlSystem::onGcsDataReceived(const ControlStationPayload& payload, uint64_t ts) { gcs_data_mb_.update(payload, ts); }
