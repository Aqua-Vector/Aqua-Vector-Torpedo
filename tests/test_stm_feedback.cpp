#include <iostream>
#include <vector>
#include <cstring>
#include <iomanip>
#include "protocol/GenericParser.hpp"
#include "protocol/ProtocolPolicies.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/Marshaller.hpp"
#include "protocol/ProtocolIds.hpp"

class MockHandler : public IMessageHandler {
public:
    bool handled = false;
    FeedbackPayload received_data;
    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        handled = true;
        Marshaller::deserialize(payload, length, received_data);
        return true;
    }
};

int main() {
    std::cout << "Running STM Feedback Structure Test (13-byte CRC range)..." << std::endl;

    GenericParser<STMControlPolicy> parser;
    MockHandler handler;
    parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &handler);

    // 1. Prepare Data
    FeedbackPayload original;
    original.m1_rps = 1.2f;
    original.m2_rps = 3.4f;
    original.servo_pos = 1500;
    original.status = 1;

    // 2. Construct Packet manually
    // Total 16 bytes: Sync1(1), Sync2(1), Func(1), Len(1), Payload(11), CRC(1)
    uint8_t packet[16];
    packet[0] = STMControlPolicy::SYNC1; // 0xAA
    packet[1] = STMControlPolicy::SYNC2; // 0x55
    packet[2] = PACKET_FUNC_CHASSIS_FEEDBACK; // 0x0B
    packet[3] = 11; // Length (4 + 4 + 2 + 1)

    Marshaller::serialize(original, packet + 4, 11);

    // Calculate CRC including Function and Length: [Func, Len, Data...] = 13 bytes
    uint8_t expected_crc = STMControlPolicy::calculateCrc(packet + 2, 13);
    packet[15] = expected_crc;

    std::cout << "Manual Packet: ";
    for(int i=0; i<16; ++i) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)packet[i] << " ";
    std::cout << std::dec << std::endl;

    // 3. Feed to Parser
    for (int i = 0; i < 16; ++i) {
        parser.parseByte(packet[i], 1000 + i);
    }

    // 4. Verify
    bool success = true;
    if (handler.handled) {
        std::cout << "[PASS] Packet parsed successfully" << std::endl;
        if (handler.received_data.m1_rps != original.m1_rps) { std::cout << "[FAIL] M1 mismatch" << std::endl; success = false; }
        if (handler.received_data.m2_rps != original.m2_rps) { std::cout << "[FAIL] M2 mismatch" << std::endl; success = false; }
        if (handler.received_data.servo_pos != original.servo_pos) { std::cout << "[FAIL] ServoPos mismatch: " << handler.received_data.servo_pos << std::endl; success = false; }
        if (handler.received_data.status != original.status) { std::cout << "[FAIL] Status mismatch" << std::endl; success = false; }
    } else {
        std::cout << "[FAIL] Parser failed to recognize the packet" << std::endl;
        success = false;
    }

    // 5. Test Serialization
    std::cout << "\nTesting Parser Serialization..." << std::endl;
    GenericPacket<FeedbackPayload, uint8_t> tx_packet;
    tx_packet.msg_id = PACKET_FUNC_CHASSIS_FEEDBACK;
    tx_packet.payload = original;

    uint8_t tx_buf[64];
    size_t tx_len = parser.serialize(tx_packet, tx_buf, sizeof(tx_buf));

    if (tx_len == 16) {
        std::cout << "[PASS] Serialized length is 16" << std::endl;
        if (tx_buf[15] == expected_crc) {
            std::cout << "[PASS] Serialized CRC matches expected" << std::endl;
        } else {
            std::cout << "[FAIL] Serialized CRC mismatch. Got: " << (int)tx_buf[15] << ", Expected: " << (int)expected_crc << std::endl;
            success = false;
        }
    } else {
        std::cout << "[FAIL] Serialized length mismatch: " << tx_len << std::endl;
        success = false;
    }

    return success ? 0 : 1;
}
