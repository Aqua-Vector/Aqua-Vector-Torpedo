#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Dense>

#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "utils/LowPassFilter.hpp"
#include "utils/Mailbox.hpp"
#include "utils/StaticRingBuffer.hpp"

// 통신 관련
#include "UartLink.hpp"
#include "STMControlParser.hpp"
#include "NetworkManager.hpp"
#include "Payloads.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"

using STMPacket = GenericPacket<ControlPayload, uint8_t>;

// STM32 피드백 핸들러
class TestFeedbackHandler : public IMessageHandler {
public:
    Mailbox<FeedbackPayload>& mb;
    explicit TestFeedbackHandler(Mailbox<FeedbackPayload>& mailbox) : mb(mailbox) {}

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(FeedbackPayload)) return false;
        FeedbackPayload data;
        Marshaller::deserialize(payload, length, data);
        mb.update(data, timestamp_ms);
        return true;
    }
};

int main(int argc, char** argv) {
    std::string imu_port = "/dev/ttyS3";
    std::string stm_port = "/dev/ttyPS1"; // Zynq UART PS1
    if (argc > 1) imu_port = argv[1];
    if (argc > 2) stm_port = argv[2];

    std::cout << "========== Dynamic Position Tracking with RPS Test ==========" << std::endl;
    
    // 1. 하드웨어 초기화
    torpedo::sensor::MiniImuUart imu(imu_port, 115200);
    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    TestFeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(0x0B, &fb_handler); // Feedback ID (수정됨: 0x02 -> 0x0B)

    if (!imu.init() || !stm_manager.start()) {
        std::cerr << "Hardware Init Failed" << std::endl;
        return -1;
    }

    // 2. 캘리브레이션 수행 (5초)
    std::cout << "Step 1: Starting Static Calibration (Keep it still for 5 seconds)..." << std::endl;
    torpedo::domain::BiasCalibrator calibrator;
    calibrator.start(5.0f, 100); // 5초, 100Hz

    uint64_t last_t_us = 0;
    while (!calibrator.is_done()) {
        torpedo::ImuSample s;
        if (imu.read(s)) {
            if (s.t_us != last_t_us) {
                calibrator.add_sample(s);
                last_t_us = s.t_us;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
        static int last_sec = -1;
        int current_sec = static_cast<int>(calibrator.progress() * 5.0f);
        if (current_sec != last_sec) {
            std::cout << "  Calibration Progress: " << current_sec + 1 << " / 5s" << std::endl;
            last_sec = current_sec;
        }
    }

    auto calib_result = calibrator.finalize();
    torpedo::domain::BiasEstimate bias_estimate = calib_result.bias;
    
    std::cout << "  Calibration Done! " << (calib_result.success ? "[SUCCESS]" : "[FAILED]") << std::endl;
    if (calib_result.success) {
        std::cout << "  - Bias Accel: " << bias_estimate.b_a.transpose() << std::endl;
        std::cout << "  - Bias Gyro: " << bias_estimate.b_g.transpose() << std::endl;
    }

    // 3. 위치 추정 루프 준비
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    if (calib_result.success) {
        eskf_params.q0 = calib_result.q0; // 캘리브레이션으로 찾은 초기 수평 자세 적용
    }
    eskf.init(eskf_params, 0.01f);

    torpedo::domain::RpsPositionTracker tracker;
    utils::LowPassFilter yaw_lpf(0.3f);
    utils::LowPassFilter speed_lpf(0.2f);
    const float WHEEL_RADIUS = 0.035f;
    const float RPS_TO_MPS = 2.0f * 3.14159265f * WHEEL_RADIUS;

    std::cout << "\nStep 2: Starting Control & Tracking Loop (100Hz)..." << std::endl;
    std::cout << "  Time [s] | M1/M2 RPS | Speed [m/s] | Yaw [deg] | Pos X [m] | Pos Y [m]" << std::endl;
    std::cout << "----------------------------------------------------------------------------" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    auto last_cmd_time = start_time;
    float current_yaw = 0.0f;
    float current_speed = 0.0f;
    float m1_rps = 0.0f, m2_rps = 0.0f;

    while (true) {
        auto now = std::chrono::steady_clock::now();

        // [수정] STM32 제어 명령 주기적 송신 (100Hz = 10ms)
        if (now - last_cmd_time >= std::chrono::milliseconds(10)) {
            ControlPayload cmd = {50.0f, 0.0f, 0.0f};
            STMPacket pkt;
            pkt.header[0] = 0xAA;
            pkt.header[1] = 0x55;
            pkt.msg_id = 0x0A;
            pkt.length = sizeof(ControlPayload);
            pkt.payload = cmd;
            pkt.crc = 0; 
            stm_tx_q.push(pkt);
            last_cmd_time = now;
        }

        // IMU 읽기 (비차단)
        torpedo::ImuSample imu_sample;
        if (imu.read(imu_sample)) {
            // ESKF로 자세 업데이트 (가속도는 제외하고 자이로 기반 자세 추정)
            imu_sample.ax = 0.0f; imu_sample.ay = 0.0f; imu_sample.az = 0.0f;
            eskf.predict(imu_sample, bias_estimate, 0.01f);
            
            current_yaw = yaw_lpf.updateAngle(imu_sample.yaw);
        }

        // STM32 Feedback 읽기
        FeedbackPayload fb;
        uint64_t ts;
        static uint64_t last_fb_ts = 0;
        if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
            last_fb_ts = ts;
            m1_rps = fb.m1_rps;
            m2_rps = fb.m2_rps;
            // [수정] 커맨드 -60f가 전진이므로, RPS 결과에 -를 붙여 양수 속도로 변환
            float raw_speed = -(m1_rps - m2_rps) * 0.5f * RPS_TO_MPS;
            current_speed = speed_lpf.update(raw_speed);
            
            // ESKF가 계산한 정교한 쿼터니언(q)을 사용하여 위치 업데이트
            tracker.update(current_speed, eskf.state().q, 0.01f);
        }

        if (now - last_log_time >= std::chrono::seconds(1)) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            const auto& pos = tracker.getPosition();
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(10) << elapsed << " | "
                      << std::setw(4) << m1_rps << "/" << std::setw(4) << m2_rps << " | "
                      << std::setw(11) << current_speed << " | "
                      << std::setw(9) << current_yaw * 180.0f / M_PI << " | "
                      << std::setw(9) << pos.x() << " | "
                      << std::setw(9) << pos.y() << std::endl;
            
            last_log_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 루프 속도 향상
    }

    stm_manager.stop();
    imu.shutdown();
    return 0;
}
