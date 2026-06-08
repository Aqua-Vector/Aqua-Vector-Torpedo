#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "utils/LowPassFilter.hpp"
#include "utils/Mailbox.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"

// Communication
#include "UartLink.hpp"
#include "STMControlParser.hpp"
#include "NetworkManager.hpp"
#include "Payloads.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"

using STMPacket = GenericPacket<ControlPayload, uint8_t>;

class FeedbackHandler : public IMessageHandler {
public:
    Mailbox<FeedbackPayload>& mb;
    explicit FeedbackHandler(Mailbox<FeedbackPayload>& mailbox) : mb(mailbox) {}

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
    std::string stm_port = "/dev/ttyPS1";
    float test_velocity = -100.0f; // Default velocity updated from -60.0f to -100.0f

    if (argc > 1) imu_port = argv[1];
    if (argc > 2) stm_port = argv[2];
    if (argc > 3) test_velocity = std::stof(argv[3]);

    std::cout << "================================================================" << std::endl;
    std::cout << "   RPS & Position Estimation Accuracy Test (10 Seconds)         " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << " IMU Port: " << imu_port << std::endl;
    std::cout << " STM Port: " << stm_port << std::endl;
    std::cout << " Velocity: " << test_velocity << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Hardware Initialization (STM32 Only)
    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    FeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(0x0B, &fb_handler); // Feedback ID

    if (!stm_manager.start()) {
        std::cerr << "[ERROR] STM32 Link Start Failed on " << stm_port << std::endl;
        return -1;
    }

    std::cout << "[INFO] IMU bypassed. Assuming straight forward movement (Yaw=0)." << std::endl;

    // 2. Setup Estimators
    torpedo::domain::RpsPositionTracker tracker;
    utils::LowPassFilter speed_lpf(0.2f);
    Eigen::Quaternionf identity_q = Eigen::Quaternionf::Identity();
    
    const float WHEEL_RADIUS = 0.035f; 
    const float RPS_TO_MPS = 2.0f * M_PI * WHEEL_RADIUS;

    // 3. Wait for Start
    std::cout << "\n[STEP 1] Ready to move." << std::endl;
    std::cout << ">>> Press ENTER to START the 10-second test run <<<" << std::endl;
    std::cin.get();

    std::cout << "\n[RUNNING] Moving for 10 seconds..." << std::endl;
    std::cout << "----------------------------------------------------------------------------" << std::endl;
    std::cout << "  Time [s] | Speed [m/s] | Odometer [m] | Pos X [m] | Pos Y [m]" << std::endl;
    std::cout << "----------------------------------------------------------------------------" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    auto last_cmd_time = start_time;
    auto last_update_time = start_time;

    bool running = true;
    while (running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - start_time).count();

        if (elapsed_sec >= 10.0) {
            running = false;
        }

        // Send Control Command (100Hz)
        if (now - last_cmd_time >= std::chrono::milliseconds(10)) {
            ControlPayload cmd = {running ? test_velocity : 0.0f, 0.0f, 0.0f};
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

        // Feedback & Position Update (IMU bypassed, using identity_q)
        FeedbackPayload fb;
        uint64_t ts;
        static uint64_t last_fb_ts = 0;
        if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
            float dt = std::chrono::duration<float>(now - last_update_time).count();
            last_update_time = now;
            last_fb_ts = ts;

            float raw_speed = -(fb.m1_rps - fb.m2_rps) * 0.5f * RPS_TO_MPS;
            float filtered_speed = speed_lpf.update(raw_speed);
            
            // Fixed heading (identity_q)
            tracker.update(filtered_speed, identity_q, dt);
        }

        // Logging (10Hz)
        if (now - last_log_time >= std::chrono::milliseconds(100)) {
            const auto& pos = tracker.getPosition();
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(10) << elapsed_sec << " | "
                      << std::setw(11) << tracker.getSpeed() << " | "
                      << std::setw(12) << tracker.getOdometer() << " | "
                      << std::setw(9) << pos.x() << " | "
                      << std::setw(9) << pos.y() << std::endl;
            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Stop Everything
    for (int i = 0; i < 5; ++i) { // Send stop command multiple times to ensure it's received
        ControlPayload stop_cmd = {0.0f, 0.0f, 0.0f};
        STMPacket pkt;
        pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = 0x0A;
        pkt.length = sizeof(ControlPayload); pkt.payload = stop_cmd; pkt.crc = 0;
        stm_tx_q.push(pkt);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n[FINISHED] Test completed." << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << "   FINAL RESULTS" << std::endl;
    std::cout << "   Total Odometer: " << tracker.getOdometer() << " m" << std::endl;
    std::cout << "   Final Position: (" << tracker.getPosition().x() << ", " << tracker.getPosition().y() << ") m" << std::endl;
    std::cout << "   Linear Displacement: " << tracker.getPosition().norm() << " m" << std::endl;
    std::cout << "================================================================" << std::endl;

    stm_manager.stop();

    return 0;
}
