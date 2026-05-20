#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <iomanip>
#include "protocol/ControlStationParser.hpp"
#include "utils/CrcCalculator.hpp"

void test_crc() {
    const char* data = "123456789";
    uint16_t crc = CrcCalculator::CalculateCrc16Ccitt((const uint8_t*)data, 9);
    std::cout << "CRC of '123456789': 0x" << std::hex << std::uppercase << crc << " (Expected: 0x29B1)" << std::endl;
    assert(crc == 0x29B1);
}

void test_serialization() {
    ControlStationPayload payload;
    payload.seq = 1234;
    payload.target_x = 100.5f;
    payload.target_y = 200.7f;
    payload.torpedo_x = 10.1f;
    payload.torpedo_y = 20.2f;
    payload.steer = -30;
    payload.flags = 0xAB;

    uint8_t buffer[128];
    bool ok = Marshaller::serialize(payload, buffer, sizeof(buffer));
    (void)ok;
    assert(ok);

    ControlStationPayload decoded;
    ok = Marshaller::deserialize(buffer, sizeof(payload), decoded);
    (void)ok;
    assert(ok);

    assert(decoded.seq == payload.seq);
    assert(std::abs(decoded.target_x - payload.target_x) < 1e-5);
    assert(std::abs(decoded.target_y - payload.target_y) < 1e-5);
    assert(std::abs(decoded.torpedo_x - payload.torpedo_x) < 1e-5);
    assert(std::abs(decoded.torpedo_y - payload.torpedo_y) < 1e-5);
    assert(decoded.steer == payload.steer);
    assert(decoded.flags == payload.flags);
    
    std::cout << "Serialization test passed!" << std::endl;
}

class MockHandler : public IMessageHandler {
public:
    bool handled = false;
    uint8_t last_msg_id = 0;
    std::vector<uint8_t> last_payload;

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        (void)timestamp_ms;
        handled = true;
        last_payload.assign(payload, payload + length);
        return true;
    }
};

void test_parser() {
    ControlStationParser parser;
    MockHandler handler;
    parser.registerHandler(0x00, &handler);

    ControlStationPayload payload;
    payload.seq = 42;
    payload.target_x = 1.0f;
    payload.target_y = 2.0f;
    payload.torpedo_x = 3.0f;
    payload.torpedo_y = 4.0f;
    payload.steer = 15;
    payload.flags = 1;

    GenericPacket<ControlStationPayload, uint16_t> packet;
    packet.msg_id = 0x00;
    packet.payload = payload;

    uint8_t buffer[128];
    size_t size = parser.serialize(packet, buffer, sizeof(buffer));
    assert(size > 0);

    std::cout << "Serialized packet size: " << size << std::endl;
    std::cout << "Packet data: ";
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
    }
    std::cout << std::dec << std::endl;

    for (size_t i = 0; i < size; ++i) {
        parser.parseByte(buffer[i], i * 10);
    }

    assert(handler.handled);
    
    ControlStationPayload parsed_payload;
    Marshaller::deserialize(handler.last_payload.data(), handler.last_payload.size(), parsed_payload);
    
    assert(parsed_payload.seq == payload.seq);
    assert(parsed_payload.steer == payload.steer);
    
    std::cout << "Parser test passed!" << std::endl;
}

int main() {
    try {
        test_crc();
        test_serialization();
        test_parser();
        std::cout << "All tests passed!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
