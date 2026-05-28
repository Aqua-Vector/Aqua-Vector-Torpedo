#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <vector>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "core/TorpedoControlSystem.hpp"
#include "core/NetworkManager.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "hal/IPwmChannel.hpp"
#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"
#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"

std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief 가짜 GCS NetworkManager
 * 물리적인 UART 연결 없이 내부적으로 TCS 코드가 정상 동작하도록 돕는 Mock 객체입니다.
 */
class MockGcsManager : public ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>> {
public:
    bool start() override { return true; }
    void stop() override {}
    bool send(const GenericPacket<TorpedoUplinkPayload, uint16_t>&) override { return true; }
};

/**
 * @brief STM32 Feedback Handler (메인 코드와 동일)
 */
class Stm32FeedbackHandler : public IMessageHandler {
private:
    TorpedoControlSystem& tcs_;
public:
    explicit Stm32FeedbackHandler(TorpedoControlSystem& tcs) : tcs_(tcs) {}
    bool handle(const uint8_t* payload, size_t len, uint64_t ts) override {
        if (len != sizeof(FeedbackPayload)) return false;
        FeedbackPayload data;
        Marshaller::deserialize(payload, len, data);
        tcs_.onStm32FeedbackReceived(data, ts);
        return true;
    }
};

/**
 * @brief 더미 액추에이터 (서보)
 */
class DummyPwm : public IPwmChannel {
public:
    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t) override { return ErrorCode::OK; }
    ErrorCode enable(bool) override { return ErrorCode::OK; }
};

int main() {
    std::signal(SIGINT, signalHandler);
    std::cout << "================================================================" << std::endl;
    std::cout << "=== PN Guidance Hardware Test (Standalone, No GCS Link) ======" << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. STM32 하드웨어 링크 설정
    UartLink stm_link("/dev/ttyPS1", 230400);
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    STMControlParser stm_parser;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    // 2. Mock GCS 매니저 설정
    MockGcsManager mock_gcs_nm;

    // 3. 로직 컴포넌트 설정
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());

    DummyPwm rudder_pwm, elevator_pwm; 
    ServoConfig servo_cfg = {20000000, 1000000, 2000000, 45.0f, 120.0f};
    ServoMotor rudder_servo(rudder_pwm, servo_cfg);
    ServoMotor elevator_servo(elevator_pwm, servo_cfg);
    ActuatorManager am(rudder_servo, elevator_servo);

    torpedo::sensor::MiniImuUart imu("/dev/ttyS3", 115200); 
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    TorpedoControlSystem tcs(mux, am, manual_source, auto_source, imu, eskf, rps_tracker, mock_gcs_nm, stm_nm);

    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    // 4. TCS 초기화 (5초 캘리브레이션 포함)
    std::cout << "[Step 1] Initializing TCS..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "FAILED: TCS Initialization!" << std::endl;
        return -1;
    }

    tcs.start();
    stm_nm.start();

    // 5. 가상의 타겟 설정 및 주행 시작
    float target_x = 5.0f;
    float target_y = 10.0f;

    std::cout << "\n[Step 2] Injecting Target Coordinate -> (" << target_x << ", " << target_y << ")" << std::endl;
    std::cout << "Starting simulation loop. Press Ctrl+C to exit." << std::endl;

    std::cout << "\n" << std::left << std::setw(6) << "Mode" 
              << std::setw(8) << "V_Cmd" << std::setw(8) << "R_Cmd" 
              << std::setw(15) << "Pos(x, y)" 
              << std::setw(10) << "Yaw(deg)"
              << std::setw(12) << "Dist_to_Tgt" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    uint32_t seq = 0;
    while (g_keep_running) {
        uint64_t now = utils::getCurrentTimeMs();

        // [핵심] 가짜 GCS 패킷을 생성하여 TCS에 주입 (10Hz 주기로 반복)
        // flags = 0x02 이면 TCS 내부에서 ModeMux를 AUTO 모드로 강제 전환함
        ControlStationPayload mock_gcs_cmd;
        mock_gcs_cmd.seq = seq++;
        mock_gcs_cmd.target_x = target_x;
        mock_gcs_cmd.target_y = target_y;
        mock_gcs_cmd.torpedo_x = 0; // RPS 트래커를 신뢰하므로 별도 LiDAR 보정 없이 0 처리
        mock_gcs_cmd.torpedo_y = 0;
        mock_gcs_cmd.steer = 0;
        mock_gcs_cmd.flags = 0x02;  // AUTO MODE 플래그

        tcs.onGcsDataReceived(mock_gcs_cmd, now);

        // 현재 제어 명령 및 상태 읽어오기
        ControlState cmd;
        uint64_t ts;
        auto_source.fetchLatestState(cmd, ts);

        const auto& pos = rps_tracker.getPosition();
        const auto& q = eskf.state().q;
        float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                               1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        float yaw_deg = yaw * 180.0f / M_PI;

        float dist = std::sqrt(std::pow(target_x - pos.x(), 2) + std::pow(target_y - pos.y(), 2));

        // 실시간 콘솔 모니터링 출력
        std::cout << "\r" << std::left 
                  << std::setw(6) << static_cast<int>(mux.getMode())
                  << std::setw(8) << std::fixed << std::setprecision(1) << cmd.velocity
                  << std::setw(8) << cmd.rudder
                  << "(" << std::setw(5) << pos.x() << ", " << std::setw(5) << pos.y() << ")   "
                  << std::setw(10) << yaw_deg
                  << std::setw(12) << dist << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz 업데이트
    }

    std::cout << "\nStopping system..." << std::endl;
    stm_nm.stop();
    tcs.stop();
    return 0;
}
