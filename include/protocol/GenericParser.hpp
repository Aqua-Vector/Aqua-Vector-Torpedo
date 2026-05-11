#ifndef GENERIC_PARSER_HPP_
#define GENERIC_PARSER_HPP_

#include <array>
#include "CommInterfaces.hpp"
#include "ControlTypes.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"

template <typename Policy>
class GenericParser : public IPacketParser {
private:
	void resetState() {
		state_ = ParseState::WAIT_SYNC1;
		index_ = 0;
		received_crc_ = 0;
		crc_byte_count_ = 0;
	}

	ParseState state_;
	std::array<IMessageHandler*, 256> handlers_;

	uint8_t buffer_[Policy::MAX_PAYLOAD_SIZE + 2];
	size_t index_;
	uint8_t msg_id_;
	uint8_t length_;

	typename Policy::CrcType received_crc_;
	size_t crc_byte_count_;
	uint64_t last_rx_time_ms_;

public:
	GenericParser() : state_(ParseState::WAIT_SYNC1), handlers_{}, index_(0), msg_id_(0), length_(0), received_crc_(0), crc_byte_count_(0), last_rx_time_ms_(0) {}
	
	using ProtocolPolicy = Policy;

	void registerHandler(uint8_t msgId, IMessageHandler* handler) {
		if (msgId < handlers_.size()) handlers_[msgId] = handler;
	}

	template <typename T> size_t serialize(const GenericPacket<T, typename Policy::CrcType>& packet, uint8_t* buffer, size_t max_len);
	void parseByte(uint8_t byte, uint64_t timestamp_ms) override;
};

template <typename Policy>
template <typename T>
size_t GenericParser<Policy>::serialize(const GenericPacket<T, typename Policy::CrcType>& packet, uint8_t* buffer, size_t max_len) {
	size_t payload_size = sizeof(T);
	size_t required_size = 4 + payload_size + sizeof(typename Policy::CrcType);

	if (max_len < required_size) return 0;

	buffer[0] = Policy::SYNC1;
	buffer[1] = Policy::SYNC2;
	buffer[2] = packet.msg_id;
	buffer[3] = static_cast<uint8_t>(payload_size);

	if (!Marshaller::serialize(packet.payload, buffer + 4, payload_size)) {
		return 0;
	}

	typename Policy::CrcType crc = Policy::calculateCrc(buffer + 2, 2 + payload_size);

	for (size_t i = 0; i < sizeof(typename Policy::CrcType); ++i) {
		buffer[4 + payload_size + i] = static_cast<uint8_t>((crc >> (i * 8)) & 0xFF);
	}

	return required_size;
}

template <typename Policy>
void GenericParser<Policy>::parseByte(uint8_t byte, uint64_t timestamp_ms) {
	if (state_ != ParseState::WAIT_SYNC1 && (timestamp_ms - last_rx_time_ms_) > 50) {
		resetState();
	}
	last_rx_time_ms_ = timestamp_ms;

	switch (state_) {
		case ParseState::WAIT_SYNC1:
			if (byte == Policy::SYNC1) state_ = ParseState::WAIT_SYNC2;
			break;

		case ParseState::WAIT_SYNC2:
			if (byte == Policy::SYNC2) state_ = ParseState::READ_MSG_ID;
			else resetState();
			break;

		case ParseState::READ_MSG_ID:
			msg_id_ = byte;
			buffer_[0] = byte;
			state_ = ParseState::READ_LENGTH;
			break;

		case ParseState::READ_LENGTH:
			length_ = byte;
			buffer_[1] = byte;
			index_ = 0;
			if (length_ > 0 && length_ <= Policy::MAX_PAYLOAD_SIZE) {
				state_ = ParseState::READ_PAYLOAD;
			} else if (length_ == 0) {
				state_ = ParseState::READ_CRC;
			} else {
				resetState();
			}
			break;

		case ParseState::READ_PAYLOAD:
			if (index_ < Policy::MAX_PAYLOAD_SIZE) {
				buffer_[2 + index_] = byte;
				index_++;
			}
			if (index_ >= length_) state_ = ParseState::READ_CRC;
			break;

		case ParseState::READ_CRC:
			received_crc_ |= (static_cast<typename Policy::CrcType>(byte) << (crc_byte_count_ * 8));

			if (++crc_byte_count_ >= sizeof(typename Policy::CrcType)) {
				if (Policy::calculateCrc(buffer_, length_ +2) == received_crc_) {
					if (handlers_[msg_id_]) {
						handlers_[msg_id_]->handle(buffer_ + 2, length_, timestamp_ms);
					}
				}
				resetState();
			}
			break;
	}
}

#endif /* GENERIC_PARSER_HPP_ */
