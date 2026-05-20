#ifndef PROTOCOL_POLICES_HPP_
#define PROTOCOL_POLICES_HPP_

#include <cstdint>
#include "CrcCalculator.hpp"

struct TorpedoPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 128;
	using CrcType = uint16_t;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc16(data, length);
	}
};

struct STMControlPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 64;
	using CrcType = uint8_t;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc8(data, length);
	}
};

struct ControlStationPolicy {
	static constexpr uint8_t SYNC1 = 0xAA;
	static constexpr uint8_t SYNC2 = 0x55;
	static constexpr size_t MAX_PAYLOAD_SIZE = 64;
	using CrcType = uint16_t;

	static CrcType calculateCrc(const uint8_t* data, size_t length) {
		return CrcCalculator::CalculateCrc16Ccitt(data, length);
	}
};

#endif /* PROTOCOL_POLICES_HPP_ */
