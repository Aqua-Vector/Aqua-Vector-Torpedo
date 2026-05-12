// packet.hpp — RS-485 통신 패킷 구조 정의
//
// 통제소(발사대) ↔ 어뢰 양방향 통신.
// Downlink (통제소 → 어뢰): 목표 좌표 + LiDAR 측정 어뢰 좌표
// Uplink (어뢰 → 통제소): ESKF 추정 위치/속도/yaw
//
// 단위: m, m/s, rad (SI)
// LiDAR 측정 실패 시 torpedo_xy = NaN

#pragma once

#include <cstdint>

namespace torpedo {

// -------- 동기 바이트 --------
constexpr uint8_t SYNC_DOWNLINK = 0xAA;  // 통제소 → 어뢰
constexpr uint8_t SYNC_UPLINK   = 0xBB;  // 어뢰 → 통제소

// -------- 어뢰 상태 플래그 (uplink) --------
enum class StatusFlag : uint8_t {
    EskfOk      = 0x01,
    LidarFresh  = 0x02,
    BiasReady   = 0x04,
};

// -------- Downlink (통제소 → 어뢰) --------
struct __attribute__((packed)) DownlinkPacket {
    uint8_t  sync;                  // 0xAA
    uint32_t timestamp_us;
    uint16_t seq;
    float    target_x, target_y;    // 가야할 좌표 (m)
    float    torpedo_x, torpedo_y;  // LiDAR 측정 (m), NaN = 측정 실패
    uint16_t crc16;
};

// -------- Uplink (어뢰 → 통제소) --------
struct __attribute__((packed)) UplinkPacket {
    uint8_t  sync;                  // 0xBB
    uint32_t timestamp_us;
    uint16_t seq;
    float    p_x, p_y;              // ESKF 추정 위치 (m), 2D
    float    yaw;                   // yaw (rad)
    uint8_t  status_flags;          // StatusFlag OR-bitmap
    uint8_t  reserved;
    uint16_t crc16;
};

// -------- 패킷 크기 검증 --------
static_assert(sizeof(DownlinkPacket) == 25, "DownlinkPacket size mismatch");
static_assert(sizeof(UplinkPacket) == 23, "UplinkPacket size mismatch");

} // namespace torpedo