#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include "core/TorpedoControlSystem.hpp"
#include "core/NetworkManager.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "hal/IPwmChannel.hpp"
#include "communication/UartLink.hpp"
#include "protocol/ControlStationParser.hpp"
#include "control/ControlHandlers.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"
#include "guidance/EbImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"

// Global flag for graceful shutdown
std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief STM32 Feedback Handler
 */
/*
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
*/

/**
 * @brief Mock PWM Channel
 */
class DummyPwm : public IPwmChannel {
public:
    uint32_t current_duty = 0;
    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t duty_ns) override { 
        current_duty = duty_ns;
        return ErrorCode::OK; 
    }
    ErrorCode enable(bool) override { return ErrorCode::OK; }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    // Hardcoded ports as requested
    const std::string imu_port = "/dev/ttyUSB0";
    const std::string stm_port = "/dev/ttyPS1";
    const std::string gcs_port = "/dev/ttyS2";
    float test_velocity = 120.0f;

    // Only velocity is taken as an optional argument
    if (argc > 1) {
        try {
            test_velocity = std::stof(argv[1]);
        } catch (...) {
            std::cerr << "Invalid velocity argument, using default: 100.0" << std::endl;
        }
    }

    std::cout << "================================================================" << std::endl;
    std::cout << "   Mid-course Guidance Test: [EBIMU + RPS] -> [TCS]            " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << " IMU Port: " << imu_port << " (Fixed)" << std::endl;
    std::cout << " STM Port: " << stm_port << " (Fixed)" << std::endl;
    std::cout << " GCS Port: " << gcs_port << " (Fixed)" << std::endl;
    std::cout << " Velocity: " << test_velocity << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Hardware Links
    std::cout << "[SYS] Setting up Hardware Links..." << std::endl;
    UartLink stm_link(stm_port, 230400);
    UartLink gcs_link(gcs_port, 115200);

    // 2. Queues
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;

    // 3. Logic Components
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());

    // 4. Parsers
    ControlStationParser gcs_parser;
    STMControlParser stm_parser;

    // 5. Network Managers
    NetworkManager<ControlStationParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    // 7. Sensors
    torpedo::sensor::EbImuUart imu(imu_port, 921600); 
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // 8. Torpedo Control System
    TorpedoControlSystem tcs(mux, manual_source, auto_source, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    // 9. Handlers
    ControlStationHandler gcs_handler(tcs, manual_source, test_velocity);
    // GCS에서 보내는 제어 패킷의 MSG_ID는 0x00으로 변경 (사용자 요청 반영)
    gcs_parser.registerHandler(0x00, &gcs_handler); 

    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    // 10. System Initialization
    std::cout << "[SYS] Initializing TCS..." << std::endl;
    if (!tcs.init(false)) { // false means do calibration
        std::cerr << "[SYS] FAILED: TCS Initialization!" << std::endl;
        return -1;
    }

    // Seed Manual Mode AFTER init
    manual_source.onControlPacketReceived({test_velocity, 0, 0}, utils::getCurrentTimeMs());
    mux.setMode(SystemMode::MANUAL);

    tcs.start();
    
    std::cout << "[SYS] All Modules Started." << std::endl;
    std::cout << "[SYS] Monitoring Mid-course Guidance (Press Ctrl+C to stop)" << std::endl;
    
    std::cout << "\n" << std::left << std::setw(6) << "Mode" 
              << std::setw(7) << "V_Cmd" << std::setw(7) << "R_Cmd" 
              << std::setw(10) << "GCS_Age" 
              << std::setw(10) << "STM_Age"
              << std::setw(10) << "Yaw(deg)" 
              << std::setw(10) << "Pos_X"
              << std::setw(10) << "Pos_Y" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    while (g_keep_running) {
        ControlState current_cmd;
        uint64_t last_gcs_ts;
        manual_source.fetchLatestState(current_cmd, last_gcs_ts);
        
        uint64_t now = utils::getCurrentTimeMs();
        uint64_t gcs_age = (last_gcs_ts > 0) ? (now - last_gcs_ts) : 9999;
        
        uint64_t last_stm_ts = tcs.getLatestFeedbackTime();
        uint64_t stm_age = (last_stm_ts > 0) ? (now - last_stm_ts) : 9999;
        
        // [Modify] ESKF 대신 RPS 트래커 좌표 출력
        const auto& pos = rps_tracker.getPosition();
        // [Modify] ESKF Yaw 대신 TCS 내부의 cumulative_yaw_rad_와 동일한 로직 사용
        float yaw_deg = tcs.getLatestCumulativeYaw() * 180.0f / 3.14159265f;

        std::cout << std::left 
                  << std::setw(6) << static_cast<int>(mux.getMode())
                  << std::setw(7) << std::fixed << std::setprecision(1) << current_cmd.velocity
                  << std::setw(7) << current_cmd.rudder
                  << std::setw(10) << std::dec << gcs_age
                  << std::setw(10) << stm_age
                  << std::setw(10) << std::fixed << std::setprecision(1) << yaw_deg 
                  << std::setw(10) << std::setprecision(2) << pos.x()
                  << std::setw(10) << pos.y() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    tcs.stop();
    return 0;
}
