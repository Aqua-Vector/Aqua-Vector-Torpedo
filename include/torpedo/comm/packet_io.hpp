// packet_io.hpp — 패킷 직렬화/파싱 인터페이스
//
// byte buffer ↔ struct 변환.
// 단순 memcpy 기반 (packed struct + little-endian ARM 가정).
// CRC 검증은 일단 생략 (P1, 시간 남으면 추가).


#pragma once

#include "torpedo/comm/packet.hpp"

#include <cstdint>
#include <cstddef>

namespace torpedo {

/**
 * Downlink 패킷 파싱.
 *
 * @param buf  byte buffer (최소 25 byte)
 * @param len  buffer 길이
 * @param out  파싱된 결과
 * @return true if sync byte 정상 + 길이 충분
 */
bool parse_downlink(const uint8_t* buf, std::size_t len, DownlinkPacket& out);

/**
 * Uplink 패킷 직렬화.
 *
 * @param in   직렬화할 패킷
 * @param buf  출력 buffer (최소 31 byte)
 * @return 쓴 byte 수
 */
std::size_t serialize_uplink(const UplinkPacket& in, uint8_t* buf);

} // namespace torpedo