#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <iomanip>

#include "core/TorpedoControlSystem.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/GenericPacket.hpp"
#include "protocol/Marshaller.hpp"
#include "hal/IPwmChannel.hpp"
#include "communication/UartLink.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"

// --- Mock Hardware ---

class AsyncMockPwm : public IPwmChannel {
public:
    std::atomic<uint32_t> last_duty_ns{0};
    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t duty_ns) override { 
        last_duty_ns.store(duty_ns); 
        return ErrorCode::OK; 
    }
    ErrorCode enable(bool) override { return ErrorCode::OK; }
};

class AsyncMockUartLink : public UartLink {
public:
    mutable std::mutex mtx;
    std::vector<uint8_t> tx_buffer;
    std::vector<uint8_t> rx_buffer;
    std::condition_variable cv;

    AsyncMockUartLink() : UartLink("/dev/null", 115200) {}
    bool initialize() override { return true; }
    void close() override {}

    ssize_t send(const uint8_t* data, size_t len) override {
        std::lock_guard<std::mutex> lock(mtx);
        tx_buffer.assign(data, data + len);
        cv.notify_all();
        return static_cast<ssize_t>(len);
    }

    ssize_t receive(uint8_t* buf, size_t max_len) override {
        std::lock_guard<std::mutex> lock(mtx);
        if (rx_buffer.empty()) return 0;
        size_t len = std::min(max_len, rx_buffer.size());
        std::memcpy(buf, rx_buffer.data(), len);
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + len);
        return static_cast<ssize_t>(len);
    }

    void injectData(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(mtx);
        rx_buffer.insert(rx_buffer.end(), data, data + len);
    }
};

/**
 * @brief GCS Control Handler (Matching src/main.cpp logic)
 */
class GcsControlHandler : public IMessageHandler {
private:
    ManualSource& manual_source_;
public:
    explicit GcsControlHandler(ManualSource& source) : manual_source_(source) {}
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(ControlPayload)) return false;
        ControlPayload data;
        Marshaller::deserialize(payload, length, data);
        manual_source_.onControlPacketReceived(data, timestamp_ms);
        return true;
    }
};

// --- Test Scenarios ---

void run_async_integration_test() {
    std::cout << "========== [ Async GCS -> TCS -> STM32/PWM Integration Test ] ==========\n" << std::endl;

    AsyncMockUartLink gcs_link;
    AsyncMockUartLink stm_link;
    
    TorpedoParser gcs_parser;
    STMControlParser stm_parser;
    
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    
    NetworkManager<TorpedoParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    AsyncMockPwm pwm_rudder, pwm_elevator;
    ServoConfig cfg = {20000000, 1000000, 2000000, 45.0f, 1000.0f};
    ServoMotor rudder(pwm_rudder, cfg);
    ServoMotor elevator(pwm_elevator, cfg);
    ActuatorManager am(rudder, elevator);
    
    ManualSource ms;
    AutoSource as;
    ModeMux mux(ms.getMailbox(), as.getMailbox());

    torpedo::SystemClock clock;
    torpedo::FakeImu imu(clock);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    torpedo::domain::RpsPositionTracker rps_tracker;
    TorpedoControlSystem tcs(mux, am, ms, as, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    // Register GCS Handler (as in main.cpp)
    GcsControlHandler gcs_handler(ms);
    gcs_parser.registerHandler(PACKET_FUNC_CHASSIS_CTRL, &gcs_handler);

    std::cout << "[Step 1] Starting TCS and Network Managers..." << std::endl;
    tcs.start();

    std::cout << "[Step 2] Simulating GCS sending manual control packet..." << std::endl;
    
    GenericPacket<ControlPayload, uint16_t> gcs_pkt;
    gcs_pkt.header[0] = 0xAA; 
    gcs_pkt.header[1] = 0x55;
    gcs_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
    gcs_pkt.length = sizeof(ControlPayload);
    gcs_pkt.payload = {5.5f, 15.0f, -5.0f}; // V=5.5, R=15, E=-5
    
    uint8_t raw_serialized[64];
    size_t pkt_len = gcs_parser.serialize(gcs_pkt, raw_serialized, sizeof(raw_serialized));
    
    // Inject multiple times to keep it alive (Manual mode watchdog)
    for(int i=0; i<5; ++i) {
        gcs_link.injectData(raw_serialized, pkt_len);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "[Step 3] Monitoring STM32 output UART..." << std::endl;
    
    bool stm_success = false;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
        std::unique_lock<std::mutex> lock(stm_link.mtx);
        if (stm_link.cv.wait_for(lock, std::chrono::milliseconds(100), [&]{ return !stm_link.tx_buffer.empty(); })) {
            if (stm_link.tx_buffer.size() >= 4 + sizeof(ControlPayload)) {
                uint8_t msg_id = stm_link.tx_buffer[2];
                if (msg_id == PACKET_FUNC_CHASSIS_CTRL) {
                    ControlPayload out_payload;
                    std::memcpy(&out_payload, stm_link.tx_buffer.data() + 4, sizeof(ControlPayload));
                    
                    if (out_payload.velocity == 5.5f && out_payload.rudder == 15.0f) {
                        std::cout << "  -> STM32 Packet Captured: V=" << out_payload.velocity 
                                  << ", R=" << out_payload.rudder << " (SUCCESS)" << std::endl;
                        stm_success = true;
                        break;
                    }
                }
            }
            stm_link.tx_buffer.clear();
        }
    }

    uint32_t rudder_duty = pwm_rudder.last_duty_ns.load();
    std::cout << "[Step 4] Checking PWM Outputs..." << std::endl;
    std::cout << "  -> Rudder Duty: " << rudder_duty << " ns" << std::endl;
    
    // 15 deg -> 1.5ms + (15/45 * 0.5ms) = 1.5ms + 0.166ms = 1.666ms = 1666666ns
    bool pwm_success = (rudder_duty > 1660000 && rudder_duty < 1675000);

    tcs.stop();

    std::cout << "\n==============================================================\n";
    if (stm_success && pwm_success) {
        std::cout << " [SUCCESS] Async Integration Test Passed!" << std::endl;
        std::cout << " -> GCS Packet -> MS Mailbox -> TCS Loop -> STM32 UART (OK)" << std::endl;
        std::cout << " -> GCS Packet -> MS Mailbox -> TCS Loop -> Servo PWM (OK)" << std::endl;
    } else {
        std::cout << " [FAILURE] Integration Test Failed." << std::endl;
        if (!stm_success) std::cout << "  - Reason: STM32 UART packet not received or incorrect." << std::endl;
        if (!pwm_success) std::cout << "  - Reason: PWM output duty cycle incorrect." << std::endl;
    }
    std::cout << "==============================================================\n" << std::endl;
}

int main() {
    run_async_integration_test();
    return 0;
}
