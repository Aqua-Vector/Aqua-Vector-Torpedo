#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include "TorpedoControlSystem.hpp"
#include "NetworkManager.hpp"
#include "UartLink.hpp"
#include "TorpedoParser.hpp"
#include "STMControlParser.hpp"
#include "ManualSource.hpp"
#include "AutoSource.hpp"
#include "ModeMux.hpp"
#include "ActuatorManager.hpp"
#include "LinuxPwmChannel.hpp"
#include "ThreadSafeQueue.hpp"
#include "ProtocolIds.hpp"
#include "Payloads.hpp"
#include "Marshaller.hpp"

#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"

// Global pointer for signal handling to allow graceful shutdown
static TorpedoControlSystem* g_tcs = nullptr;

/**
 * @brief Signal handler for SIGINT (Ctrl+C) and SIGTERM
 * Ensures actuators are stopped and threads are joined safely.
 */
void signalHandler(int signum) {
    std::cout << "\n[Main] Signal (" << signum << ") received. Stopping system..." << std::endl;
    if (g_tcs) {
        g_tcs->stop();
    }
}

/**
 * @brief GCS Control Handler
 * Handles control commands from Ground Control Station via TorpedoParser.
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
        
        // Update manual control source
        manual_source_.onControlPacketReceived(data, timestamp_ms);
        return true;
    }
};

/**
 * @brief STM32 Feedback Handler
 * Handles feedback packets from STM32 motor controller via STMControlParser.
 */
class Stm32FeedbackHandler : public IMessageHandler {
private:
    TorpedoControlSystem& tcs_;
public:
    explicit Stm32FeedbackHandler(TorpedoControlSystem& tcs) : tcs_(tcs) {}
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(FeedbackPayload)) return false;
        
        FeedbackPayload data;
        Marshaller::deserialize(payload, length, data);
        
        // Update TCS with feedback
        tcs_.onStm32FeedbackReceived(data, timestamp_ms);
        return true;
    }
};

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "      Aqua Vector Torpedo Control System        " << std::endl;
    std::cout << "================================================" << std::endl;

    // 1. Hardware & Communication Link Setup
    // Zynq-STM32: UART PS1, 230400 baud
    UartLink stm32_link("/dev/ttyPS1", 230400);
    // Zynq-GCS/Radio: UART S2, 460800 baud
    UartLink gcs_link("/dev/ttyS2", 460800);

    // 2. Protocol Parser & TX Queue Preparation
    TorpedoParser gcs_parser;
    STMControlParser stm32_parser;

    ThreadSafeQueue<TorpedoPacket> gcs_tx_queue;
    ThreadSafeQueue<STMPacket> stm32_tx_queue;

    // 3. Logic Component Assembly
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux(manual_source.getMailbox(), auto_source.getMailbox());

    // Sensors & Estimators
    torpedo::sensor::MiniImuUart imu("/dev/ttyS3", 115200);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);

    // Hardware Actuators (Zynq Local PWM)
    // Mapping each servo to its own PWM chip as seen in Petalinux sysfs
    // Rudder on pwmchip0 (ch0), Elevator on pwmchip1 (ch0)
    LinuxPwmChannel rudder_pwm(0, 0);
    LinuxPwmChannel elevator_pwm(1, 0);

    // Servo Config: 20ms (50Hz), 1-2ms pulse, 60deg range, 120deg/s max speed
    ServoConfig servo_cfg = {20000000, 1000000, 2000000, 60.0f, 120.0f};
    ServoMotor rudder_servo(rudder_pwm, servo_cfg);
    ServoMotor elevator_servo(elevator_pwm, servo_cfg);

    ActuatorManager actuator_manager(rudder_servo, elevator_servo);

    // 4. Communication Manager Setup
    NetworkManager<TorpedoParser, UartLink, TorpedoPacket> gcs_manager(gcs_link, gcs_parser, gcs_tx_queue);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm32_manager(stm32_link, stm32_parser, stm32_tx_queue);

    // 5. Torpedo Control System (TCS) Creation
    TorpedoControlSystem tcs(mode_mux, actuator_manager, manual_source, auto_source, imu, eskf, gcs_manager, stm32_manager);
    g_tcs = &tcs;

    // 6. Callback Registration
    GcsControlHandler gcs_handler(manual_source);
    gcs_parser.registerHandler(PACKET_FUNC_CHASSIS_CTRL, &gcs_handler);

    Stm32FeedbackHandler stm32_handler(tcs);
    stm32_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &stm32_handler);

    // 7. Signal Handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 8. System Initialization & Startup
    std::cout << "[Main] Initializing system components..." << std::endl;
    if (!tcs.init()) {
        std::cerr << "[Main] CRITICAL: System initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "[Main] Starting control loops and communication..." << std::endl;
    if (!tcs.start()) {
        std::cerr << "[Main] CRITICAL: System startup failed!" << std::endl;
        return -1;
    }

    std::cout << "[Main] System is RUNNING. Press Ctrl+C to stop." << std::endl;

    // 9. Main Thread Wait Loop
    while (tcs.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Main] Exiting." << std::endl;
    return 0;
}
