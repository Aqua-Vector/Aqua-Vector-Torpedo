#include "torpedo/comm/packet_io.hpp"
#include "torpedo/comm/crc16.hpp"

#include <cstring>

namespace torpedo {

bool parse_downlink(const uint8_t* buf, std::size_t len, DownlinkPacket& out) {
    // 길이 확인
    if (len < sizeof(DownlinkPacket)) {
        return false;
    }
    
    // sync byte 확인
    if (buf[0] != SYNC_DOWNLINK) {
        return false;
    }

    // CRC 검증 — sync ~ torpedo_y (23 byte)
    uint16_t computed_crc = crc16_ccitt(buf, 23);
    uint16_t received_crc;
    std::memcpy(&received_crc, buf + 23, 2);
    if (computed_crc != received_crc) {
        return false;
    }
    
    // memcpy (packed struct + little-endian ARM)
    std::memcpy(&out, buf, sizeof(DownlinkPacket));
    return true;
}

std::size_t serialize_uplink(const UplinkPacket& in, uint8_t* buf) {
    std::memcpy(buf, &in, sizeof(UplinkPacket));

    // CRC 재계산 (sync ~ reserved까지)
    // Uplink 31 byte: CRC 범위 = 0 ~ 28 = 29 byte
    uint16_t crc = crc16_ccitt(buf, sizeof(UplinkPacket) - 2);
    std::memcpy(buf + sizeof(UplinkPacket) - 2, &crc, 2);

    return sizeof(UplinkPacket);
}

} // namespace torpedo