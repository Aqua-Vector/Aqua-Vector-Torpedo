#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "core/TorpedoControlSystem.hpp"
#include "core/NetworkManager.hpp"
#include "control/ModeMux.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/TimeUtils.hpp"
#include "guidance/EbImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"

std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

class DummyGcsParser : public IPacketParser {
public:
    void parseByte(uint8_t, uint64_t) override {}
    struct ProtocolPolicy { static constexpr size_t MAX_PAYLOAD_SIZE = 128; };
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>&, uint8_t*, size_t) { return 0; }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    float test_speed = 0.0f; // Default stationary
    if (argc > 1) {
        test_speed = std::stof(argv[1]);
    }

    std::cout << "================================================================" << std::endl;
    std::cout << "   [TEST 2] Steering Deadband Finder                            " << std::endl;
    std::cout << "   Target Speed: " << test_speed << std::endl;
    std::cout << "================================================================" << std::endl;

    UartLink stm_link("/dev/ttyPS1", 230400);
    UartLink gcs_link("/dev/ttyS2", 115200);
    STMControlParser stm_parser;
    DummyGcsParser gcs_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;

    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());
    torpedo::sensor::EbImuUart imu("/dev/ttyUSB0", 921600);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);
    NetworkManager<DummyGcsParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);

    TorpedoControlSystem tcs(mux, manual_source, auto_source, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    if (!tcs.init(true)) return -1;
    tcs.start();

    std::cout << "\n[READY] Keep the car stationary." << std::endl;
    std::cout << "[READY] Rudder will increase by 0.5 deg every 0.5s." << std::endl;
    std::cout << "[READY] Press Ctrl+C when wheels move or sound changes." << std::endl;
    std::cout << "[READY] Press ENTER to START..." << std::endl;
    std::cin.get();

    mux.setMode(SystemMode::MANUAL);
    float current_rudder = 0.0f;

    while (g_keep_running) {
        manual_source.onControlPacketReceived({0.0f, current_rudder, 0.0f}, utils::getCurrentTimeMs());
        
        std::cout << "\r   Current Rudder Command: " << std::fixed << std::setprecision(1) 
                  << std::setw(5) << current_rudder << " deg" << std::flush;
        
        current_rudder += 0.5f;
        if (current_rudder > 25.0f) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "\n\n================================================================" << std::endl;
    std::cout << "   CALIBRATION RESULT" << std::endl;
    std::cout << "   The last printed value is your environment's DEADBAND." << std::endl;
    std::cout << "================================================================" << std::endl;

    tcs.stop();
    return 0;
}
