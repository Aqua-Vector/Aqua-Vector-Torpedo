#include "torpedo/comm/crc16.hpp"

namespace torpedo {

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

} // namespace torpedo