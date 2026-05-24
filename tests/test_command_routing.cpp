#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <thread>

#include "core/TorpedoControlSystem.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "hal/IPwmChannel.hpp"
#include "communication/UartLink.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"

// Simple assertion macros
#define ASSERT_TRUE(cond) if (!(cond)) { std::cerr << "[FAIL] " << #cond << " at line " << __LINE__ << std::endl; return false; }
#define ASSERT_EQ(val1, val2) if ((val1) != (val2)) { std::cerr << "[FAIL] " << #val1 << " (" << (val1) << ") != " << #val2 << " (" << (val2) << ") at line " << __LINE__ << std::endl; return false; }
#define ASSERT_NEAR(val1, val2, eps) if (std::abs(static_cast<long long>(val1) - static_cast<long long>(val2)) > (eps)) { std::cerr << "[FAIL] " << #val1 << " (" << (val1) << ") not near " << #val2 << " (" << (val2) << ") with eps " << (eps) << " at line " << __LINE__ << std::endl; return false; }

/**
 * @brief Mock PWM Channel to capture output
 */
class MockPwm : public IPwmChannel {
public:
    uint32_t last_duty_ns = 0;
    bool is_enabled = false;

    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t duty_ns) override { 
        last_duty_ns = duty_ns; 
        return ErrorCode::OK; 
    }
    ErrorCode enable(bool en) override { 
        is_enabled = en; 
        return ErrorCode::OK; 
    }
};

/**
 * @brief Mock UartLink to capture STM32 packets
 */
class MockUartLink : public UartLink {
public:
    std::vector<uint8_t> last_sent_data;

    MockUartLink() : UartLink("/dev/null", 115200) {}
    bool initialize() override { return true; }
    void close() override {}
    ssize_t send(const uint8_t* data, size_t len) override {
        last_sent_data.assign(data, data + len);
        return static_cast<ssize_t>(len);
    }
    ssize_t receive(uint8_t*, size_t) override { return 0; }
};

/**
 * @brief Testable subclass to expose processControlCycle
 */
class TestableTCS : public TorpedoControlSystem {
public:
    using TorpedoControlSystem::TorpedoControlSystem;
    void tick(uint64_t current_time_ms) {
        processControlCycle(current_time_ms);
    }
};

bool run_test() {
    std::cout << "Starting Command Routing Integration Test..." << std::endl;

    // 1. Setup Components
    MockUartLink gcs_link, stm_link;
    TorpedoParser gcs_parser;
    STMControlParser stm_parser;
    StaticRingBuffer<UplinkPacket, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    
    NetworkManager<TorpedoParser, UartLink, UplinkPacket> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    MockPwm pwm_rudder, pwm_elevator;
    ServoConfig cfg = {20000000, 1000000, 2000000, 45.0f, 5000.0f}; // 5000 deg/sec = 50 deg per 10ms tick
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

    TestableTCS tcs(mux, am, ms, as, imu, eskf, gcs_nm, stm_nm);

    // Start managers to allow send() to work and threads to process
    stm_nm.start();
    gcs_nm.start();

    uint64_t virtual_time_ms = 1000;

    // --- Scenario 1: Manual Mode Delivery ---
    std::cout << "Scenario 1: Testing Manual Mode..." << std::endl;
    mux.setMode(SystemMode::MANUAL);
    ControlPayload manual_cmd = {3.5f, 15.0f, -10.0f};
    ms.onControlPacketReceived(manual_cmd, virtual_time_ms);
    
    tcs.tick(virtual_time_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Allow NM thread to process

    // Verify STM32 Packet
    ASSERT_TRUE(!stm_link.last_sent_data.empty());
    // Manual deserialize (simplified for check)
    // Packet structure: [STX][MSG_ID][LEN][PAYLOAD...][CRC][ETX] - depends on GenericParser/Marshaller
    // We can check the payload directly if we know the offset, but better to check if send was called.
    // In our case, stm32_manager_.send(stm_pkt) was called.
    
    // For duty cycle verification (15.0 deg -> 1.5ms pulse + middle offset)
    // 0 deg = 1.5ms = 1500000ns
    // +15 deg = 1.5ms + (15/45 * 0.5ms) = 1.5ms + 0.166ms = 1.666ms = 1666666ns
    ASSERT_NEAR(pwm_rudder.last_duty_ns, 1666666, 2000); 
    ASSERT_NEAR(pwm_elevator.last_duty_ns, 1388888, 2000); 

    // --- Scenario 2: Auto Mode & Isolation ---
    std::cout << "Scenario 2: Testing Auto Mode (via Guidance) & Isolation..." << std::endl;
    virtual_time_ms += 20;
    mux.setMode(SystemMode::AUTO);
    
    // To induce a non-zero AUTO command, we must provide a target via GCS
    // because TCS overwrites AutoSource with GuidanceManager output.
    ControlStationPayload gcs_packet;
    std::memset(&gcs_packet, 0, sizeof(gcs_packet));
    gcs_packet.target_x = 100.0f; // Target far ahead
    gcs_packet.target_y = 50.0f;  // and to the right
    tcs.onGcsDataReceived(gcs_packet, virtual_time_ms);
    
    // Inject extreme manual noise
    ControlPayload manual_noise = {99.0f, 99.0f, 99.0f};
    ms.onControlPacketReceived(manual_noise, virtual_time_ms);

    tcs.tick(virtual_time_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Guidance should produce a positive rudder to turn right
    // Neutral is 1500000. Right turn (positive rudder) should be > 1500000.
    ASSERT_TRUE(pwm_rudder.last_duty_ns > 1550000); 
    // And manual noise (99.0) should NOT be here (which would be 2000000)
    ASSERT_TRUE(pwm_rudder.last_duty_ns < 1950000); 

    // --- Scenario 3: Watchdog Failsafe ---
    std::cout << "Scenario 3: Testing Watchdog Failsafe (Manual Mode)..." << std::endl;
    mux.setMode(SystemMode::MANUAL);
    // Update once to have a valid last_update
    ms.onControlPacketReceived(manual_cmd, virtual_time_ms);
    tcs.tick(virtual_time_ms);
    ASSERT_EQ(static_cast<int>(mux.getMode()), static_cast<int>(SystemMode::MANUAL));

    virtual_time_ms += 600; // Timeout > 500ms
    
    tcs.tick(virtual_time_ms);

    ASSERT_EQ(static_cast<int>(mux.getMode()), static_cast<int>(SystemMode::FAILSAFE));
    ASSERT_NEAR(pwm_rudder.last_duty_ns, 1500000, 100); // Neutral
    ASSERT_NEAR(pwm_elevator.last_duty_ns, 1500000, 100); // Neutral

    std::cout << "\n[SUCCESS] All Command Routing Scenarios Passed!" << std::endl;
    return true;
}

int main() {
    if (run_test()) return 0;
    else return 1;
}
