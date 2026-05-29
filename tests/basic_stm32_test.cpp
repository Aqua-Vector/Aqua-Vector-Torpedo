#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <iomanip>

#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/GenericPacket.hpp"
#include "utils/TimeUtils.hpp"

// STM32 전송용 패킷 타입 정의
using STMPacket = GenericPacket<ControlPayload, uint8_t>;

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

// STM32 피드백을 모니터링할 핸들러
class FeedbackMonitor : public IMessageHandler {
public:
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        (void)timestamp_ms;
        FeedbackPayload data;
        if (Marshaller::deserialize(payload, length, data)) {
            std::cout << "[STM32 Feedback] "
                      << "RPS: " << std::fixed << std::setprecision(2) << data.m1_rps << "/" << data.m2_rps
                      << " | Servo: " << data.servo_pos
                      << " | Status: " << (int)data.status << std::endl;
            return true;
        }
        return false;
    }
};

int main() {
    std::signal(SIGINT, signalHandler);

    std::cout << "====================================================" << std::endl;
    std::cout << "   Simple STM32 Comm Test (100Hz, 115200bps)        " << std::endl;
    std::cout << "   Target: V=-60.0, R=0.0, E=0.0                   " << std::endl;
    std::cout << "====================================================" << std::endl;

    UartLink link("/dev/ttyPS1", 230400);
    if (!link.initialize()) {
        std::cerr << "[Error] Failed to initialize UART /dev/ttyPS1" << std::endl;
        return -1;
    }

    STMControlParser parser;
    FeedbackMonitor monitor;
    parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &monitor);

    // 디버그 모드 활성화 (파싱 과정을 자세히 보고 싶을 때)
    // parser.setDebug(true);

    while (g_running) {
        uint64_t now_ms = utils::getCurrentTimeMs();

        // 2. 명령 전송 (TX) - 100Hz
        STMPacket tx_pkt;
        tx_pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
        tx_pkt.payload.velocity = -60.0f;
        tx_pkt.payload.rudder = 0.0f;
        tx_pkt.payload.elevator = 0.0f;

        uint8_t tx_buf[64];
        size_t len = parser.serialize(tx_pkt, tx_buf, sizeof(tx_buf));
        if (len > 0) {
            link.send(tx_buf, len);
        }

        // 3. 피드백 수신 (RX)
        uint8_t rx_byte;
        // Non-blocking 모드이므로 데이터가 있는 만큼 전부 읽어서 파서에 전달
        while (link.receive(&rx_byte, 1) > 0) {
            parser.parseByte(rx_byte, now_ms);
        }

        // 100Hz 주기 유지
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    link.close();
    std::cout << "\nTest Stopped." << std::endl;
    return 0;
}
