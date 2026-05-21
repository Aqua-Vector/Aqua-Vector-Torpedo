#include <iostream>
#include <thread>
#include <chrono>
#include <cassert>
#include <vector>

#include "TorpedoControlSystem.hpp"
#include "ModeMux.hpp"
#include "ActuatorManager.hpp"
#include "ManualSource.hpp"
#include "AutoSource.hpp"
#include "NetworkManager.hpp"
#include "TorpedoParser.hpp"
#include "STMControlParser.hpp"
#include "UartLink.hpp"
#include "ThreadSafeQueue.hpp"
#include "IPwmChannel.hpp"
#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"

// Mock PWM for testing
class MockPwm : public IPwmChannel {
public:
    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t) override { return ErrorCode::OK; }
    ErrorCode enable(bool) override { return ErrorCode::OK; }
};

// Mock UartLink to avoid real hardware dependency during test
class MockUartLink : public UartLink {
public:
    MockUartLink() : UartLink("/dev/null", 115200) {}
    bool initialize() override { return true; }
    void close() override {}
    ssize_t send(const uint8_t* data, size_t len) override {
        (void)data;
        send_count_++;
        return static_cast<ssize_t>(len);
    }
    ssize_t receive(uint8_t* buf, size_t max_len) override {
        (void)buf; (void)max_len;
        return 0;
    }    size_t send_count_ = 0;
};

int main() {
    std::cout << "========== [ TorpedoControlSystem Integration Test ] ==========\n" << std::endl;

    // 1. Setup Mock Components
    MockUartLink gcs_link;
    MockUartLink stm_link;
    
    TorpedoParser gcs_parser;
    STMControlParser stm_parser;
    
    ThreadSafeQueue<TorpedoPacket> gcs_tx_q;
    ThreadSafeQueue<STMPacket> stm_tx_q;
    
    NetworkManager<TorpedoParser, UartLink, TorpedoPacket> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    MockPwm pwm1, pwm2;
    // ServoConfig: period_ns, min_pulse_ns, max_pulse_ns, max_angle_deg, max_deg_per_sec
    ServoConfig cfg = {20000000, 1000000, 2000000, 45.0f, 60.0f};
    ServoMotor rudder(pwm1, cfg);
    ServoMotor elevator(pwm2, cfg);
    ActuatorManager am(rudder, elevator);
    
    ManualSource ms;
    AutoSource as;
    ModeMux mux(ms.getMailbox(), as.getMailbox());

    // Sensors & Estimators for Test
    torpedo::sensor::MiniImuUart imu("/dev/null", 115200);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // 2. Initialize TCS
    TorpedoControlSystem tcs(mux, am, ms, as, imu, eskf, gcs_nm, stm_nm);

    if (!tcs.init()) {
        std::cerr << "TCS Init Failed" << std::endl;
        return -1;
    }

    // 3. Start TCS
    std::cout << "Starting TCS..." << std::endl;
    tcs.start();

    // 4. Scenario: Manual Input Injection
    std::cout << "Injecting Manual Control Data..." << std::endl;
    uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    ControlPayload cmd = {2.0f, 15.0f, -10.0f}; // V=2.0, R=15, E=-10
    ms.onControlPacketReceived(cmd, now);

    // Wait for a few cycles (TCS runs at 100Hz = 10ms per loop)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 5. Verification
    std::cout << "\nChecking results..." << std::endl;
    std::cout << " - STM32 packets sent: " << stm_link.send_count_ << " (Expected: > 40)" << std::endl;

    bool success = true;
    if (stm_link.send_count_ < 40) success = false;

    // 6. Stop TCS
    std::cout << "\nStopping TCS..." << std::endl;
    tcs.stop();

    if (success) {
        std::cout << "\n[SUCCESS] TorpedoControlSystem Test Passed!" << std::endl;
        return 0;
    } else {
        std::cout << "\n[FAILURE] TorpedoControlSystem Test Failed!" << std::endl;
        return -1;
    }
}
