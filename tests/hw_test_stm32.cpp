#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "UartLink.hpp"
#include "STMControlParser.hpp"
#include "ProtocolIds.hpp"
#include "Payloads.hpp"
#include "Marshaller.hpp"
#include "ThreadSafeQueue.hpp"
#include "TorpedoControlSystem.hpp"

std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    std::cout << "\nStopping test..." << std::endl;
    g_running = false;
}

// STM32 피드백을 처리할 핸들러
class Stm32FeedbackMonitor : public IMessageHandler {
public:
    void printHeader() {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "----------------------------------------------------------------" << std::endl;
        std::cout << " [Time]    M1_RPS    M2_RPS    ServoPos    Status" << std::endl;
        std::cout << "----------------------------------------------------------------" << std::endl;
    }

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        FeedbackPayload data;
        if (Marshaller::deserialize(payload, length, data)) {
            std::cout << "[" << std::setw(8) << timestamp_ms << "] "
                      << std::setw(8) << data.m1_rps << "  "
                      << std::setw(8) << data.m2_rps << "  "
                      << std::setw(8) << (int)data.servo_pos << "    "
                      << std::setw(6) << (int)data.status << std::endl;
            return true;
        }
        return false;
    }
};

int main() {
    std::signal(SIGINT, signalHandler);

    std::cout << "=== STM32 Hardware Integration Test (100Hz) ===" << std::endl;
    
    // 1. 하드웨어 설정 (UART PS1, 230400 bps)
    UartLink stm32_link("/dev/ttyPS1", 230400);
    if (!stm32_link.initialize()) {
        std::cerr << "Failed to initialize UART /dev/ttyPS1" << std::endl;
        return -1;
    }

    STMControlParser parser;
    Stm32FeedbackMonitor monitor;
    parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &monitor);

    monitor.printHeader();

    // 2. 송신 루프 스레드 (100Hz)
    std::thread tx_thread([&]() {
        float velocity = 0.0f;
        float direction = 1.0f;
        
        while (g_running) {
            STMPacket tx_pkt;
            tx_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
            
            // 임의의 제어 데이터 생성 (S-curve 형태)
            velocity += 0.05f * direction;
            if (velocity > 2.0f || velocity < -2.0f) direction *= -1.0f;
            
            tx_pkt.payload.velocity = velocity;
            tx_pkt.payload.rudder = 10.0f;   // 고정 조향
            tx_pkt.payload.elevator = -5.0f; // 고정 승강
            
            uint8_t tx_buf[64];
            size_t len = parser.serialize(tx_pkt, tx_buf, sizeof(tx_buf));
            if (len > 0) {
                stm32_link.send(tx_buf, len);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz
        }
    });

    // 3. 수신 루프 (메인 스레드)
    uint8_t rx_byte;
    while (g_running) {
        if (stm32_link.receive(&rx_byte, 1) > 0) {
            auto now = std::chrono::steady_clock::now();
            uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            parser.parseByte(rx_byte, ms);
        }
    }

    if (tx_thread.joinable()) tx_thread.join();
    stm32_link.close();
    
    std::cout << "Test Finished." << std::endl;
    return 0;
}
