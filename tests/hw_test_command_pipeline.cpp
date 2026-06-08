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
#include "torpedo/domain/estimator/rps_tracker.hpp"

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
    uint8_t buffer_[128]; // 넉넉한 버퍼 크기
    size_t rx_idx_ = 0;
    int state_ = 0;
    uint8_t expected_length_ = 0;

public:
    explicit LegacyGcsParser(ManualSource& ms) : ms_(ms) {}

    // IPacketParser Interface
    void parseByte(uint8_t byte, uint64_t timestamp_ms) override {
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
            
            // 안전 장치: 허용 불가능한 길이면 처음부터 다시 동기화
            if (expected_length_ > 100) {
                state_ = 0; 
                return;
            }
            
            state_ = 4;
            rx_idx_ = 4;
        } else if (state_ == 4) {
            buffer_[rx_idx_++] = byte;
            
            size_t total_expected = expected_length_ + 6; // 헤더(4) + 페이로드(len) + CRC(2)
            
            // 전체 패킷을 다 받았을 때
            if (rx_idx_ >= total_expected) {
                // 우리가 원하는 수동제어 패킷(길이 21)인지 확인
                if (expected_length_ == sizeof(ControlStationPayload)) {
                    uint16_t calc_crc = CrcCalculator::CalculateCrc16Ccitt(buffer_, total_expected - 2);
                    
                    // 수신된 CRC 추출 (Little Endian 가정)
                    uint16_t received_crc = buffer_[total_expected - 2] | (buffer_[total_expected - 1] << 8);
                    
                    if (calc_crc == received_crc) {
                        ControlStationPayload payload;
                        std::memcpy(&payload, &buffer_[4], sizeof(ControlStationPayload));
                        
                        ControlPayload cmd;
                        cmd.velocity = 60.0f; 
                        cmd.rudder = static_cast<float>(payload.steer);
                        cmd.elevator = 0.0f;
                        ms_.onControlPacketReceived(cmd, timestamp_ms);
                    }
                }
                state_ = 0; rx_idx_ = 0;
            }
        }
    }

    // NetworkManager의 TX 스레드에서 호출하는 직렬화 함수
    struct ProtocolPolicy { static constexpr size_t MAX_PAYLOAD_SIZE = 128; };
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>& pkt, uint8_t* buf, size_t max_len) {
        if (max_len < sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>)) return 0;
        
        // GenericPacket<TorpedoUplinkPayload, uint16_t>은 이미 sync(0xBB), payload, crc16을 멤버로 가지고 있음
        // 그대로 복사만 하면 됨 (CRC는 TCS에서 finalizeCrc()를 통해 채워짐)
        std::memcpy(buf, &pkt, sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>));
        
        return sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>);
    }
};

/**
 * @brief STM32 Feedback Handler (Matches main.cpp)
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
    UartLink gcs_link("/dev/ttyS2", 460800);

    // 2. Queues
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;

    // 3. Logic Components
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mux(manual_source.getMailbox(), auto_source.getMailbox());

    // 4. Parsers
    // GCS: Custom parser to handle the specific CRC range, but inside NM framework
    LegacyGcsParser gcs_parser(manual_source);
    STMControlParser stm_parser;

    // 5. Network Managers (Now using standard NM with StaticRingBuffer internally)
    NetworkManager<LegacyGcsParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    // 7. Sensors
    torpedo::sensor::MiniImuUart imu("/dev/ttyS3", 115200); 
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // 8. Torpedo Control System
    torpedo::domain::RpsPositionTracker rps_tracker;
    TorpedoControlSystem tcs(mux, manual_source, auto_source, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    // 9. Register STM32 Handler
    Stm32FeedbackHandler stm_handler(tcs);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm_handler);

    // 10. System Initialization
    std::cout << "[Step 2] Initializing TCS..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "  FAILED: TCS Initialization!" << std::endl;
        return -1;
    }

    // Seed Manual Mode
    manual_source.onControlPacketReceived({60.0f, 0, 0}, utils::getCurrentTimeMs());
    mux.setMode(SystemMode::MANUAL);

    tcs.start();
    // [NM Framework] Now starting GCS NM which uses StaticRingBuffer and its own RX thread
    gcs_nm.start(); 
    
    std::cout << "  - All Modules (TCS, NM, RingBuffers) Started." << std::endl;

    // 11. Real-time Monitoring Loop
    std::cout << "[Step 3] Monitoring Pipeline (Press Ctrl+C to stop)" << std::endl;
    std::cout << std::left << std::setw(6) << "Mode" 
              << std::setw(7) << "V_Cmd" << std::setw(7) << "R_Cmd" 
              << std::setw(10) << "Age" 
              << std::setw(12) << "PosX" << std::setw(12) << "PosY" << std::endl;
    std::cout << "--------------------------------------------------------------------------------------------" << std::endl;

    while (g_keep_running) {
        ControlState current_cmd;
        uint64_t last_ts;
        manual_source.fetchLatestState(current_cmd, last_ts);
        uint64_t age = (last_ts > 0) ? (utils::getCurrentTimeMs() - last_ts) : 9999;

        std::cout << "\r" << std::left 
                  << std::setw(6) << static_cast<int>(mux.getMode())
                  << std::setw(7) << std::fixed << std::setprecision(1) << current_cmd.velocity
                  << std::setw(7) << current_cmd.rudder
                  << std::setw(10) << std::dec << age << "ms"
                  << std::setw(12) << "ACTIVE" 
                  << std::setw(12) << "ACTIVE" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 12. Cleanup
    gcs_nm.stop();
    tcs.stop();
    return 0;
}
