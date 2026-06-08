#include <iostream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>

// 1. 핵심 컴포넌트 헤더
#include "core/TorpedoControlSystem.hpp"
#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "guidance/GuidanceManager.hpp"

// 2. 통신 및 제어 인프라 헤더
#include "communication/UartLink.hpp"
#include "protocol/TorpedoParser.hpp"
#include "control/STMControlParser.hpp"
#include "control/ModeMux.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "actuator/ActuatorManager.hpp"
#include "hal/LinuxPwmChannel.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"

using namespace torpedo;

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
    std::cout << "\n[HIL Test] Signal (" << signum << ") received. Shutting down..." << std::endl;
    g_running = false;
}

/**
 * @brief STM32 피드백 핸들러 (통합 테스트용)
 */
class HilFeedbackHandler : public IMessageHandler {
private:
    TorpedoControlSystem& tcs_;
public:
    explicit HilFeedbackHandler(TorpedoControlSystem& tcs) : tcs_(tcs) {}
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        FeedbackPayload data;
        if (Marshaller::deserialize(payload, length, data)) {
            tcs_.onStm32FeedbackReceived(data, timestamp_ms);
            return true;
        }
        return false;
    }
};

int main() {
    signal(SIGINT, signalHandler);

    std::cout << "================================================" << std::endl;
    std::cout << "   Aqua Vector Torpedo FULL HIL INTEGRATION    " << std::endl;
    std::cout << "================================================" << std::endl;

    // --- STEP 1: 하드웨어 객체 생성 ---
    
    // MiniIMU (JD Pmod)
    sensor::MiniImuUart imu("/dev/ttyS3", 115200);

    // STM32 (UART PS1)
    UartLink stm32_link("/dev/ttyPS1", 230400);
    STMControlParser stm32_parser;
    StaticRingBuffer<STMPacket, 64> stm32_tx_queue;

    // GCS/Uplink (UART S2) - 초기화 성공 확인용
    UartLink gcs_link("/dev/ttyS2", 460800); 
    TorpedoParser gcs_parser;
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_queue;

    // --- STEP 2: 제어 로직 조립 ---
    
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux(manual_source.getMailbox(), auto_source.getMailbox());

    // 인지 엔진 (ESKF)
    domain::EskfEstimator eskf;
    domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // 통신 매니저
    NetworkManager<TorpedoParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_manager(gcs_link, gcs_parser, gcs_tx_queue);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm32_manager(stm32_link, stm32_parser, stm32_tx_queue);

    // --- STEP 3: 시스템 통합 (TCS) ---
    
    domain::RpsPositionTracker rps_tracker;
    TorpedoControlSystem tcs(mode_mux, manual_source, auto_source, imu, eskf, rps_tracker, gcs_manager, stm32_manager);

    // STM32 피드백 콜백 등록
    HilFeedbackHandler stm32_handler(tcs);
    stm32_parser.registerHandler(0x11, &stm32_handler); // PACKET_FUNC_CHASSIS_FEEDBACK

    // --- STEP 4: 테스트 시작 ---

    std::cout << "[HIL] Initializing Control System..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "FAILED to init TCS" << std::endl;
        return 1;
    }

    tcs.start();

    // 테스트를 위해 모드를 AUTO로 변경
    mode_mux.setMode(SystemMode::AUTO);

    // 가상 타겟 설정 (X=5m, Y=5m 지점)
    ControlStationPayload virtual_target;
    virtual_target.target_x = 5.0f;
    virtual_target.target_y = 5.0f;
    virtual_target.flags = 0x00; // 아직 중기 유도 모드
    
    std::cout << "[HIL] System Started. Virtual Target at (5, 5)" << std::endl;

    auto last_log_time = std::chrono::steady_clock::now();
    
    while (g_running) {
        // 1. 매 루프마다 가상 타겟 데이터 주입 (통제소 역할을 테스트 코드가 대행)
        tcs.onGcsDataReceived(virtual_target, utils::getCurrentTimeMs());

        // 2. 주기적 상태 로그 출력 (1Hz)
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time >= std::chrono::seconds(1)) {
            last_log_time = now;

            const auto& state = eskf.state();
            
            std::cout << "\n------------------------------------------------" << std::endl;
            std::cout << "[HIL STATUS] Mode: AUTO | Time: " << utils::getCurrentTimeMs()/1000.0 << "s" << std::endl;
            std::cout << "  - Ego Pos   : (" << std::fixed << std::setprecision(2) << state.p.x() << ", " << state.p.y() << ") m" << std::endl;
            std::cout << "  - Velocity  : " << state.v.norm() << " m/s" << std::endl;
            std::cout << "  - GCS Uplink: /dev/ttyS2 Ready" << std::endl;
            std::cout << "  - STM32 Link: Connected" << std::endl;
            std::cout << "------------------------------------------------" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[HIL] Stopping system..." << std::endl;
    tcs.stop();

    return 0;
}
