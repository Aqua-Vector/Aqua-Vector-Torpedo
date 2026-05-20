#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include "communication/UartLink.hpp"
#include "protocol/ControlStationParser.hpp"
#include "core/NetworkManager.hpp"
#include "utils/ThreadSafeQueue.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
    (void)signum;
    g_running = false;
}

class ControlStationHandler : public IMessageHandler {
public:
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        ControlStationPayload data;
        if (Marshaller::deserialize(payload, length, data)) {
            std::cout << "[RX][" << timestamp_ms << "ms] "
                      << "Seq: " << std::dec << data.seq << " | "
                      << "Target: (" << std::fixed << std::setprecision(2) << data.target_x << ", " << data.target_y << ") | "
                      << "Torpedo: (" << data.torpedo_x << ", " << data.torpedo_y << ") | "
                      << "Steer: " << (int)data.steer << " | "
                      << "Flags: 0x" << std::hex << (int)data.flags << std::dec << std::endl;
        } else {
            std::cerr << "[RX] Failed to deserialize payload" << std::endl;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    std::string port = "/dev/ttyS2";
    int baud = 460800;

    if (argc > 1) port = argv[1];
    if (argc > 2) baud = std::stoi(argv[2]);

    std::cout << "Starting Control Station Hardware Test" << std::endl;
    std::cout << "Port: " << port << ", Baud: " << baud << std::endl;

    UartLink link(port, baud);
    ControlStationParser parser;
    parser.setDebug(true); // Enable detailed parser logging
    ControlStationHandler handler;

    // Register handler for msg_id 0x00
    parser.registerHandler(0x00, &handler);

    // NetworkManager needs a TxPacket type. Even if we only receive, we need a queue.
    using TxPacket = GenericPacket<ControlStationPayload, uint16_t>;
    ThreadSafeQueue<TxPacket> tx_queue;

    NetworkManager<ControlStationParser, UartLink, TxPacket> net_manager(link, parser, tx_queue);

    if (!net_manager.start()) {
        std::cerr << "Failed to start NetworkManager (check " << port << " permissions)" << std::endl;
        return 1;
    }

    std::cout << "Test running. Debug mode ENABLED. Press Ctrl+C to stop." << std::endl;

    uint32_t count = 0;
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (g_running) {
            std::cout << "[Heartbeat] System running... (sec: " << ++count << ")" << std::endl;
        }
    }

    std::cout << "Stopping test..." << std::endl;
    net_manager.stop();

    return 0;
}
