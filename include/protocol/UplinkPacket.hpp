#ifndef UP_LINK_PACKET_HPP_
#define UP_LINK_PACKET_HPP_

#include <cstdint>
#include "Payloads.hpp"
#include "CrcCalculator.hpp"

#pragma pack(push, 1)
struct UplinkPacket {
	uint8_t sync = 0xBB;
	TorpedoUplinkPayload payload;
	uint16_t crc16;

	void finalizeCrc() {
		this->crc16 = CrcCalculator::CalculateCrc16Ccitt(reinterpret_cast<const uint8_t*>(this), sizeof(sync) + sizeof(payload));
	}
};
#pragma pack(pop)

#endif /* UP_LINK_PACKET_HPP_ */
