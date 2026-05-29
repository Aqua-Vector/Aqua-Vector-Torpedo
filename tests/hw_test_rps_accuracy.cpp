#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <vector>
#include <iomanip>
#include <cmath>

#include "core/TorpedoControlSystem.hpp"
#include "core/NetworkManager.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "hal/IPwmChannel.hpp"
#include "communication/UartLink.hpp"
#include "protocol/TorpedoParser.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"

// Global flag for graceful shutdown
std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief Simple Dummy Parser for GCS (not used in this test, but needed for NetworkManager)
 */
class DummyGcsParser : public IPacketParser {
public:
    void parseByte(uint8_t, uint64_t) override {}
    struct ProtocolPolicy { static constexpr size_t MAX_PAYLOAD_SIZE = 128; };
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>&, uint8_t*, size_t) { return 0; }
};

/**
 * @brief STM32 Feedback Handler
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
 * @brief Mock PWM Channel
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
    std::cout << "   RPS Accuracy & Position Tracking Test (10 Seconds)           " << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Hardware Links
    UartLink stm_link("/dev/ttyPS1", 230400);
    UartLink gcs_link("/dev/ttyS2", 115200); // Actually dummy in this test

    // 2. Queues
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;

    // 3. Logic Components
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());

    // 4. Parsers
    DummyGcsParser gcs_parser;
    STMControlParser stm_parser;

    // 5. Network Managers
    NetworkManager<DummyGcsParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    // 6. Actuators
    DummyPwm rudder_pwm, elevator_pwm; 
    ServoConfig servo_cfg = {20000000, 1000000, 2000000, 45.0f, 120.0f};
    ServoMotor rudder_servo(rudder_pwm, servo_cfg);
    ServoMotor elevator_servo(elevator_pwm, servo_cfg);
    ActuatorManager am(rudder_servo, elevator_servo);

    // 7. Sensors
    torpedo::sensor::MiniImuUart imu("/dev/ttyS3", 115200); 
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // 8. Torpedo Control System
    TorpedoControlSystem tcs(mux, am, manual_source, auto_source, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    // 9. Register STM32 Handler
    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    // 10. Initialization
    if (!tcs.init(true)) { // Skip IMU calibration for faster testing
        std::cerr << "[SYS] Initialization FAILED!" << std::endl;
        return -1;
    }

    tcs.start();
    stm_nm.start();
    
    std::cout << "\n[READY] System initialized and background tasks running." << std::endl;
    std::cout << "[READY] Press ENTER to START 10-second straight run test..." << std::endl;
    std::cin.get();

    // Reset Tracker before starting
    rps_tracker.reset();
    
    // Start Moving
    float test_velocity = 60.0f; // User specified velocity in previous logs
    std::cout << "[RUN] Starting movement at V = " << test_velocity << "..." << std::endl;
    manual_source.onControlPacketReceived({test_velocity, 0.0f, 0.0f}, utils::getCurrentTimeMs());
    mux.setMode(SystemMode::MANUAL);

    auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;

    std::cout << "\n" << std::left 
              << std::setw(10) << "Time[s]" 
              << std::setw(12) << "Speed[m/s]" 
              << std::setw(12) << "Dist[m]" 
              << std::setw(10) << "X[m]" 
              << std::setw(10) << "Y[m]" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    while (g_keep_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        
        if (elapsed >= 10.0) break;

        // Log at 10Hz
        if (std::chrono::duration<double>(now - last_log_time).count() >= 0.1) {
            float speed = rps_tracker.getSpeed();
            float dist = rps_tracker.getOdometer();
            auto pos = rps_tracker.getPosition();

            std::cout << std::left << std::fixed << std::setprecision(2)
                      << std::setw(10) << elapsed
                      << std::setw(12) << speed
                      << std::setw(12) << dist
                      << std::setw(10) << pos.x()
                      << std::setw(10) << pos.y() << std::endl;
            
            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop Moving
    std::cout << "\n[STOP] Test finished. Stopping motors..." << std::endl;
    manual_source.onControlPacketReceived({0.0f, 0.0f, 0.0f}, utils::getCurrentTimeMs());
    
    // Final Results
    float final_dist = rps_tracker.getOdometer();
    auto final_pos = rps_tracker.getPosition();
    std::cout << "================================================================" << std::endl;
    std::cout << "   TEST SUMMARY" << std::endl;
    std::cout << "   Total Traveled Distance (Odometer): " << std::fixed << std::setprecision(3) << final_dist << " m" << std::endl;
    std::cout << "   Final Displacement (X, Y): (" << final_pos.x() << ", " << final_pos.y() << ") m" << std::endl;
    std::cout << "   Straight Line Error (X): " << std::abs(final_pos.x()) << " m" << std::endl;
    std::cout << "================================================================" << std::endl;

    tcs.stop();
    stm_nm.stop();
    return 0;
}
