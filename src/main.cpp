#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#include "NetworkManager.hpp"
#include "UartLink.hpp"
#include "TorpedoParser.hpp"
#include "ControlHandlers.hpp"
#include "ThreadSafeQueue.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"

// [테스트용 핸들러] 데이터를 받으면 화면에 출력하여 검증
class LoopbackTestHandler : public IMessageHandler {
public:
    bool handle(const uint8_t* payload, size_t payload_length, uint64_t timestamp_ms) override {
        if (payload_length != sizeof(TorpedoTelemetryPayload)) return false;

        TorpedoTelemetryPayload data;
        if (Marshaller::deserialize(payload, payload_length, data)) {
            std::cout << "\n[LOOPBACK SUCCESS]" << std::endl;
            std::cout << " - Timestamp: " << timestamp_ms << " ms" << std::endl;
            std::cout << " - Speed: " << data.speed << " m/s" << std::endl;
            std::cout << " - Heading: " << data.heading << " deg" << std::endl;
            std::cout << " - Position: (" << data.pos_x << ", " << data.pos_y << ")" << std::endl;
            return true;
        }
        return false;
    }
};

int main() {
    std::cout << "====== Torpedo Loopback Test Start ======" << std::endl;
    std::cout << "Connect Zynq TX and RX pins before starting." << std::endl;

    // 1. 부품 준비
    TorpedoParser parser;
    UartLink uart("/dev/ttyPS1", 115200); // 테스트용 115200bps

    // TX 큐 정의 (TorpedoTelemetry 전송 테스트)
    using TorpedoPacket = GenericPacket<TorpedoTelemetryPayload, uint16_t>;
    ThreadSafeQueue<TorpedoPacket> txQueue;

    // 2. NetworkManager 조립
    NetworkManager<TorpedoParser, UartLink, TorpedoPacket> manager(uart, parser, txQueue);

    // 3. 테스트 핸들러 등록 (MsgId 0x10 가정)
    LoopbackTestHandler testHandler;
    parser.registerHandler(0x10, &testHandler);

    // 4. 시스템 시작
    if (manager.start()) {
        std::cout << "[INFO] NetworkManager Started" << std::endl;
    } else {
        std::cerr << "[ERROR] Failed to start NetworkManager" << std::endl;
        return -1;
    }

    // 5. 테스트 루프: 1초마다 데이터 송신
    int test_count = 0;
    while (test_count < 10) {
        // 송신 데이터 생성
        TorpedoPacket packet;
        packet.msg_id = 0x10;
        packet.payload.speed = 1.23f * (test_count + 1);
        packet.payload.heading = 45.0f;
        packet.payload.acc_x = 0.1f;
        packet.payload.acc_y = 0.2f;
        packet.payload.pos_x = 100 + test_count;
        packet.payload.pos_y = 200 + test_count;

        std::cout << "\n[TX] Sending Packet #" << test_count + 1 << "..." << std::endl;
        if (manager.send(packet)) {
            std::cout << " - Packet pushed to TX Queue" << std::endl;
        } else {
            std::cerr << " - Failed to push to TX Queue" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        test_count++;
    }

    std::cout << "\nTest Complete. Stopping..." << std::endl;
    manager.stop();
    return 0;
}
