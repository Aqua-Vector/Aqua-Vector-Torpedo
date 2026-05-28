#ifndef PROTOCOL_POLICES_HPP_
#define PROTOCOL_POLICES_HPP_

#include <cstdint>
#include "CrcCalculator.hpp"

struct TorpedoPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 128;
	using CrcType = uint16_t;
	static constexpr size_t CRC_SIZE = 2;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc16Ccitt(data, length);
	}
};

struct STMControlPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 64;
	using CrcType = uint8_t;
	static constexpr size_t CRC_SIZE = 1;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return static_cast<uint8_t>(CrcCalculator::CalculateCrc8(data, length) & 0xFF);
	}
};

struct ControlStationPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 64;
	using CrcType = uint16_t;
	static constexpr size_t CRC_SIZE = 2;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc16Ccitt(data, length);
	}
};

struct TelemetryPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 64;
	using CrcType = uint16_t;
	static constexpr size_t CRC_SIZE = 2;
	static constexpr uint8_t MSG_ID = 0x40;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc16Ccitt(data, length);
	}
};

#endif /* PROTOCOL_POLICES_HPP_ */
