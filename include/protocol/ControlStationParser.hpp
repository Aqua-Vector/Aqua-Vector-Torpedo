#ifndef CONTROL_STATION_PARSER_HPP_
#define CONTROL_STATION_PARSER_HPP_

#include <array>
#include <cstring>
#include "CommInterfaces.hpp"
#include "ProtocolPolicies.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"
#include "CrcCalculator.hpp"

/**
 * @brief ControlStationParser
 * Specialized parser for the Ground Control Station protocol.
 * Includes headers (SYNC1, SYNC2) in the CRC calculation.
 */
class ControlStationParser : public IPacketParser {
private:
    enum class ParseState {
        WAIT_SYNC1,
        WAIT_SYNC2,
        READ_MSG_ID,
        READ_LENGTH,
        READ_PAYLOAD,
        READ_CRC_L,
        READ_CRC_H
    };

    ParseState state_ = ParseState::WAIT_SYNC1;
    std::array<IMessageHandler*, 256> handlers_{};
    uint8_t buffer_[ControlStationPolicy::MAX_PAYLOAD_SIZE + 6]; // SYNC1, SYNC2, MSG_ID, LEN, PAYLOAD, CRC_L, CRC_H
    size_t index_ = 0;
    uint8_t msg_id_ = 0;
    uint8_t length_ = 0;
    uint16_t received_crc_ = 0;
    uint64_t last_rx_time_ms_ = 0;

    void resetState() {
        state_ = ParseState::WAIT_SYNC1;
        index_ = 0;
        received_crc_ = 0;
    }

public:
    using ProtocolPolicy = ControlStationPolicy;
    ControlStationParser() = default;

    void registerHandler(uint8_t msgId, IMessageHandler* handler) {
        if (msgId < handlers_.size()) handlers_[msgId] = handler;
    }

    void parseByte(uint8_t byte, uint64_t timestamp_ms) override {
        if (state_ != ParseState::WAIT_SYNC1 && (timestamp_ms - last_rx_time_ms_) > 50) {
            resetState();
        }
        last_rx_time_ms_ = timestamp_ms;

        switch (state_) {
            case ParseState::WAIT_SYNC1:
                if (byte == ControlStationPolicy::SYNC1) {
                    buffer_[0] = byte;
                    state_ = ParseState::WAIT_SYNC2;
                }
                break;

            case ParseState::WAIT_SYNC2:
                if (byte == ControlStationPolicy::SYNC2) {
                    buffer_[1] = byte;
                    state_ = ParseState::READ_MSG_ID;
                } else if (byte == ControlStationPolicy::SYNC1) {
                    state_ = ParseState::WAIT_SYNC2;
                } else {
                    resetState();
                }
                break;

            case ParseState::READ_MSG_ID:
                msg_id_ = byte;
                buffer_[2] = byte;
                state_ = ParseState::READ_LENGTH;
                break;

            case ParseState::READ_LENGTH:
                length_ = byte;
                buffer_[3] = byte;
                index_ = 0;
                if (length_ <= ControlStationPolicy::MAX_PAYLOAD_SIZE) {
                    if (length_ > 0) state_ = ParseState::READ_PAYLOAD;
                    else state_ = ParseState::READ_CRC_L;
                } else {
                    resetState();
                }
                break;

            case ParseState::READ_PAYLOAD:
                buffer_[4 + index_] = byte;
                index_++;
                if (index_ >= length_) {
                    state_ = ParseState::READ_CRC_L;
                }
                break;

            case ParseState::READ_CRC_L:
                received_crc_ = byte;
                state_ = ParseState::READ_CRC_H;
                break;

            case ParseState::READ_CRC_H:
                received_crc_ |= (static_cast<uint16_t>(byte) << 8);

                // [Modify] RX는 싱크 바이트(0xAA, 0x55)를 포함하여 전체 계산
                uint16_t calculated = CrcCalculator::CalculateCrc16Ccitt(buffer_, length_ + 4);
                if (calculated == received_crc_) {
                    if (handlers_[msg_id_]) {
                        handlers_[msg_id_]->handle(buffer_ + 4, length_, timestamp_ms);
                    } else if (handlers_[0x00]) {
                        // 0x00 핸들러로 폴백
                        handlers_[0x00]->handle(buffer_ + 4, length_, timestamp_ms);
                    }
                } else {
                    static uint32_t crc_err_cnt = 0;
                    if (++crc_err_cnt % 100 == 1) {
                        std::printf("[Parser] GCS CRC Error! Calc: 0x%04X, Recv: 0x%04X\n", calculated, received_crc_);
                    }
                }
                resetState();
                break;
        }
    }

    template <typename T>
    size_t serialize(const GenericPacket<T, uint16_t>& packet, uint8_t* buffer, size_t max_len) {
        size_t payload_size = sizeof(T);
        size_t total_size = 4 + payload_size + 2;
        if (max_len < total_size) return 0;

        buffer[0] = ControlStationPolicy::SYNC1;
        buffer[1] = ControlStationPolicy::SYNC2;
        buffer[2] = packet.msg_id;
        buffer[3] = static_cast<uint8_t>(payload_size);
        Marshaller::serialize(packet.payload, buffer + 4, payload_size);

        // [Modify] TX는 싱크 바이트를 제외하고 ID(index 2)부터 계산
        uint16_t crc = CrcCalculator::CalculateCrc16Ccitt(buffer + 2, 2 + payload_size);
        buffer[4 + payload_size] = static_cast<uint8_t>(crc);
        buffer[5 + payload_size] = static_cast<uint8_t>(crc >> 8);

        return total_size;
    }

};

#endif /* CONTROL_STATION_PARSER_HPP_ */
