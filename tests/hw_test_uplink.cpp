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
 * @brief Legacy GCS Parser that uses full-packet CRC calculation
 * but satisfies the standard IPacketParser interface to be used with NetworkManager.
 */
class LegacyGcsParser : public IPacketParser {
private:
    ManualSource& ms_;
    TorpedoControlSystem* tcs_ = nullptr;
    uint8_t buffer_[128]; 
    size_t rx_idx_ = 0;
    int state_ = 0;
    uint8_t expected_length_ = 0;
    uint64_t last_rx_time_ms_ = 0;

public:
    explicit LegacyGcsParser(ManualSource& ms) : ms_(ms) {}

    void setTcs(TorpedoControlSystem* tcs) { tcs_ = tcs; }

    // IPacketParser Interface
    void parseByte(uint8_t byte, uint64_t timestamp_ms) override {
        // [Add] Timeout recovery: reset state if no data for 50ms
        if (state_ != 0 && (timestamp_ms - last_rx_time_ms_) > 50) {
            state_ = 0;
            rx_idx_ = 0;
        }
        last_rx_time_ms_ = timestamp_ms;

        if (state_ == 0) {
            if (byte == 0xAA) { buffer_[0] = byte; state_ = 1; }
        } else if (state_ == 1) {
            if (byte == 0x55) { buffer_[1] = byte; state_ = 2; }
            else if (byte == 0xAA) { state_ = 1; }
            else { state_ = 0; }
        } else if (state_ == 2) {
            buffer_[2] = byte; // msg_id
            state_ = 3;
        } else if (state_ == 3) {
            buffer_[3] = byte; // length
            expected_length_ = byte;
            
            if (expected_length_ > 100) {
                state_ = 0; 
                return;
            }
            
            state_ = 4;
            rx_idx_ = 4;
        } else if (state_ == 4) {
            buffer_[rx_idx_++] = byte;
            
            size_t total_expected = expected_length_ + 6; 
            
            if (rx_idx_ >= total_expected) {
                if (expected_length_ == sizeof(ControlStationPayload)) {
                    uint16_t calc_crc = CrcCalculator::CalculateCrc16Ccitt(buffer_, total_expected - 2);
                    uint16_t received_crc = buffer_[total_expected - 2] | (buffer_[total_expected - 1] << 8);
                    
                    if (calc_crc == received_crc) {
                        ControlStationPayload payload;
                        std::memcpy(&payload, &buffer_[4], sizeof(ControlStationPayload));
                        
                        // TCS에게 GCS 데이터 수신 알림
                        if (tcs_) {
                            tcs_->onGcsDataReceived(payload, timestamp_ms);
                        }

                        if (payload.seq % 100 == 0) {
                            std::cout << "\n[GCS RX Success] "
                                      << "Seq: " << payload.seq
                                      << " | Target: (" << payload.target_x << ", " << payload.target_y << ")"
                                      << " | Torpedo: (" << payload.torpedo_x << ", " << payload.torpedo_y << ")"
                                      << " | Steer: " << payload.steer 
                                      << " | Flags: 0x" << std::hex << static_cast<int>(payload.flags) << std::dec << std::endl;
                        }

                        ControlPayload cmd;
                        cmd.velocity = 60.0f; 
                        cmd.rudder = -1.0f * static_cast<float>(payload.steer);
                        cmd.elevator = 0.0f;
                        ms_.onControlPacketReceived(cmd, timestamp_ms);
                    } else {
                        // CRC 에러 로그 추가
                        static uint32_t crc_err_count = 0;
                        if (++crc_err_count % 10 == 1) {
                            std::cerr << "\n[GCS CRC ERR] Recv: 0x" << std::hex << received_crc 
                                      << " | Calc: 0x" << calc_crc << " (Count: " << std::dec << crc_err_count << ")" << std::endl;
                        }
                    }
                }
                state_ = 0; rx_idx_ = 0;
            }
        }
    }

    // NetworkManager의 TX 스레드에서 호출하는 직렬화 함수
    struct ProtocolPolicy { static constexpr size_t MAX_PAYLOAD_SIZE = 128; };
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>& pkt, uint8_t* buf, size_t max_len) {
        size_t payload_size = sizeof(TorpedoUplinkPayload);
        size_t total_size = 4 + payload_size + 2;
        if (max_len < total_size) return 0;

        std::memcpy(buf, &pkt, total_size);

        // GCS TX 로그 일시 중단
        if (pkt.payload.seq % 10 == 0) {
            std::cout << "\n[GCS TX Uplink] "
                      << "Seq: " << pkt.payload.seq
                      << " | Pos: (" << std::fixed << std::setprecision(2) << pkt.payload.p_x << ", " << pkt.payload.p_y << ")"
                      << " | Yaw: " << (pkt.payload.yaw * 180.0f / 3.14159265f) << " deg"
                      << " | Status: 0x" << std::hex << static_cast<int>(pkt.payload.status_flags) << std::dec << std::endl;
        }
        

        return total_size;
    }
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
        
        static bool first_fb = true;
        if (first_fb) {
            std::cout << "\n[STM32 FB] First feedback packet received!" << std::endl;
            first_fb = false;
        }
        
        tcs_.onStm32FeedbackReceived(data, ts);
        return true;
    }
};

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

int main() {
    std::signal(SIGINT, signalHandler);

    std::cout << "================================================================" << std::endl;
    std::cout << "   Zynq HW Pipeline Test: [NM FRAMEWORK] -> [TCS] -> [STM32]   " << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Hardware Links
    std::cout << "[Step 1] Setting up Hardware Links..." << std::endl;
    UartLink stm_link("/dev/ttyPS1", 230400);
    UartLink gcs_link("/dev/ttyS2", 115200);

    // 2. Queues
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;

    // 3. Logic Components
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());

    // 4. Parsers
    LegacyGcsParser gcs_parser(manual_source);
    STMControlParser stm_parser;

    // 5. Network Managers
    NetworkManager<LegacyGcsParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
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
    gcs_parser.setTcs(&tcs);

    // 9. Register STM32 Handler
    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    // 10. System Initialization
    std::cout << "[Step 2] Initializing TCS..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "  FAILED: TCS Initialization!" << std::endl;
        return -1;
    }

    // Seed Manual Mode AFTER init
    manual_source.onControlPacketReceived({60.0f, 0, 0}, utils::getCurrentTimeMs());
    mux.setMode(SystemMode::MANUAL);

    tcs.start();
    gcs_nm.start(); 
    stm_nm.start();
    
    std::cout << "  - All Modules (TCS, NM, RingBuffers) Started." << std::endl;

    // 11. Real-time Monitoring Loop
    std::cout << "[Step 3] Monitoring Pipeline & Uplink (Press Ctrl+C to stop)" << std::endl;
    
    std::cout << "\n" << std::left << std::setw(6) << "Mode" 
              << std::setw(7) << "V_Cmd" << std::setw(7) << "R_Cmd" 
              << std::setw(10) << "Age" 
              << std::setw(15) << "RPS(M1/M2)"
              << std::setw(12) << "Uplink" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    while (g_keep_running) {
        ControlState current_cmd;
        uint64_t last_ts;
        manual_source.fetchLatestState(current_cmd, last_ts);
        uint64_t now = utils::getCurrentTimeMs();
        uint64_t age = (last_ts > 0) ? (now - last_ts) : 9999;

        float speed = rps_tracker.getSpeed();
        const auto& pos = rps_tracker.getPosition();
        const auto& q = eskf.state().q;
        
        float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                               1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        float yaw_deg = yaw * 180.0f / 3.14159265f;

        std::cout << "\r" << std::left 
                  << std::setw(6) << static_cast<int>(mux.getMode())
                  << std::setw(7) << std::fixed << std::setprecision(1) << current_cmd.velocity
                  << std::setw(7) << current_cmd.rudder
                  << std::setw(10) << std::dec << age << "ms"
                  << " RPS:" << std::setw(10) << std::fixed << std::setprecision(2) << speed / 0.22f 
                  << " | P:(" << pos.x() << "," << pos.y() << ")"
                  << " Y:" << std::setw(6) << yaw_deg << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 12. Cleanup
    stm_nm.stop();
    gcs_nm.stop();
    tcs.stop();
    return 0;
}
