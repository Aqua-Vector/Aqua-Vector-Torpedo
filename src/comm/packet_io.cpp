#include "torpedo/comm/packet_io.hpp"

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
    
    // memcpy (packed struct + little-endian ARM)
    std::memcpy(&out, buf, sizeof(DownlinkPacket));
    return true;
}

std::size_t serialize_uplink(const UplinkPacket& in, uint8_t* buf) {
    std::memcpy(buf, &in, sizeof(UplinkPacket));
    return sizeof(UplinkPacket);
}

} // namespace torpedo