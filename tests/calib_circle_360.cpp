#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cmath>
#include <string>

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

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    float test_speed = 100.0f; // Default circle speed
    if (argc > 1) {
        try {
            test_speed = std::stof(argv[1]);
        } catch (...) {
            std::cerr << "Invalid speed argument. Using default 100.0" << std::endl;
        }
    }

    std::cout << "================================================================" << std::endl;
    std::cout << "   [TEST 3] 360-Degree Turn & Slip Inspector                     " << std::endl;
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

    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    if (!tcs.init(true)) return -1;
    tcs.start();

    std::cout << "\n[READY] Ensure enough space for a circle (Radius ~0.5-1.0m)." << std::endl;
    std::cout << "[READY] Press ENTER to start 360-degree turn..." << std::endl;
    std::cin.get();

    rps_tracker.reset();
    mux.setMode(SystemMode::MANUAL);
    
    float test_rudder = 15.0f;
    manual_source.onControlPacketReceived({test_speed, test_rudder, 0.0f}, utils::getCurrentTimeMs());

    float start_yaw = tcs.getLatestCumulativeYaw();
    bool completed = false;

    while (g_keep_running) {
        float current_yaw = tcs.getLatestCumulativeYaw();
        float yaw_diff = std::abs(current_yaw - start_yaw);

        if (yaw_diff >= (2.0f * M_PI)) {
            manual_source.onControlPacketReceived({0.0f, 0.0f, 0.0f}, utils::getCurrentTimeMs());
            completed = true;
            break;
        }

        auto pos = rps_tracker.getPosition();
        std::cout << "\r   Yaw: " << std::fixed << std::setprecision(1) << (yaw_diff * 180.0f / M_PI) << " deg | "
                  << "Pos: (" << pos.x() << ", " << pos.y() << ")" << std::flush;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (completed) {
        auto final_pos = rps_tracker.getPosition();
        std::cout << "\n\n================================================================" << std::endl;
        std::cout << "   CALIBRATION CHECK" << std::endl;
        std::cout << "   1. Is the car pointing exactly where it started?" << std::endl;
        std::cout << "   2. Return Position Error (X, Y): (" << final_pos.x() << ", " << final_pos.y() << ")" << std::endl;
        std::cout << "   If X-error is large (>15cm), increase ESKF sigma_nhc." << std::endl;
        std::cout << "================================================================" << std::endl;
    }

    tcs.stop();
    return 0;
}
