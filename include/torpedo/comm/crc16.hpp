// crc16.hpp — CRC16-CCITT 계산
//
// Polynomial: 0x1021, Initial: 0xFFFF
// 표준 검증 벡터: "123456789" → 0x29B1
//

#pragma once

#include <cstdint>
#include <cstddef>

namespace torpedo {

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len);

} // namespace torpedo
