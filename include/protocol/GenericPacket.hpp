#ifndef GENERIC_PACKET_HPP_
#define GENERIC_PACKET_HPP_

#include <cstdint>
#include "Payloads.hpp"

#pragma pack(push, 1)

template <typename PayloadType, typename CrcType = uint16_t>
struct GenericPacket {
	uint8_t header[2];
	uint8_t msg_id;
	uint8_t length;
	PayloadType payload;
	CrcType crc;
};

template <typename PayloadType>
struct GenericPacket<PayloadType, NoCrc> {
	uint8_t header[2];
	uint8_t msg_id;
	uint8_t length;
	PayloadType payload;
};

#pragma pack(pop)

#endif /* GENERIC_PACKET_HPP_ */
