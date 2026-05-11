// 패킷 직렬화/파싱 검증

#include "torpedo/comm/packet.hpp"
#include "torpedo/comm/packet_io.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>

int main() {
    using namespace torpedo;

    // ─── Test 1: 패킷 크기 ───
    printf("[Test 1] 패킷 크기 검증\n");
    printf("  DownlinkPacket = %zu byte (expected 25)\n", sizeof(DownlinkPacket));
    printf("  UplinkPacket   = %zu byte (expected 31)\n", sizeof(UplinkPacket));
    if (sizeof(DownlinkPacket) != 25 || sizeof(UplinkPacket) != 31) {
        printf("[FAIL] 패킷 크기 불일치\n");
        return 1;
    }

    // ─── Test 2: Downlink round-trip ───
    printf("\n[Test 2] Downlink 직렬화 → 파싱 round-trip\n");
    {
        DownlinkPacket original;
        original.sync = SYNC_DOWNLINK;
        original.timestamp_us = 1234567;
        original.seq = 42;
        original.target_x = 1.5f;
        original.target_y = 2.5f;
        original.torpedo_x = 1.55f;
        original.torpedo_y = 2.48f;
        original.crc16 = 0x0000;

        uint8_t buf[64];
        std::memcpy(buf, &original, sizeof(original));

        DownlinkPacket parsed;
        bool ok = parse_downlink(buf, sizeof(original), parsed);
        if (!ok) {
            printf("[FAIL] 파싱 실패\n");
            return 1;
        }

        if (parsed.timestamp_us != original.timestamp_us) { printf("[FAIL] ts\n"); return 1; }
        if (parsed.seq != original.seq) { printf("[FAIL] seq\n"); return 1; }
        if (std::abs(parsed.target_x - original.target_x) > 1e-6f) { printf("[FAIL] tx\n"); return 1; }
        if (std::abs(parsed.torpedo_x - original.torpedo_x) > 1e-6f) { printf("[FAIL] mx\n"); return 1; }
        printf("  OK\n");
    }

    // ─── Test 3: NaN 측정값 (LiDAR 실패) ───
    printf("\n[Test 3] NaN 측정값 처리 (LiDAR 실패)\n");
    {
        DownlinkPacket pkt;
        pkt.sync = SYNC_DOWNLINK;
        pkt.timestamp_us = 1000;
        pkt.seq = 1;
        pkt.target_x = 1.0f;
        pkt.target_y = 2.0f;
        pkt.torpedo_x = std::nanf("");
        pkt.torpedo_y = std::nanf("");
        pkt.crc16 = 0;

        uint8_t buf[64];
        std::memcpy(buf, &pkt, sizeof(pkt));

        DownlinkPacket parsed;
        parse_downlink(buf, sizeof(pkt), parsed);
        
        if (std::abs(parsed.target_x - 1.0f) > 1e-6f) {
            printf("[FAIL] target 손상\n"); return 1;
        }
        
        if (!std::isnan(parsed.torpedo_x)) {
            printf("[FAIL] NaN 보존 실패\n"); return 1;
        }
        printf("  OK (NaN = LiDAR 실패 표시)\n");
    }

    // ─── Test 4: 잘못된 sync ───
    printf("\n[Test 4] 잘못된 sync byte 거부\n");
    {
        uint8_t bad_buf[64] = {0};
        bad_buf[0] = 0xFF;
        DownlinkPacket out;
        if (parse_downlink(bad_buf, 25, out)) {
            printf("[FAIL] 잘못된 sync 통과\n"); return 1;
        }
        printf("  OK\n");
    }

    // ─── Test 5: 짧은 buffer ───
    printf("\n[Test 5] 짧은 buffer 거부\n");
    {
        uint8_t short_buf[10] = {SYNC_DOWNLINK};
        DownlinkPacket out;
        if (parse_downlink(short_buf, 10, out)) {
            printf("[FAIL] 짧은 buffer 통과\n"); return 1;
        }
        printf("  OK\n");
    }

    // ─── Test 6: Uplink 직렬화 ───
    printf("\n[Test 6] Uplink 직렬화 (2D)\n");
    {
        UplinkPacket uplink;
        uplink.sync = SYNC_UPLINK;
        uplink.timestamp_us = 9999;
        uplink.seq = 100;
        uplink.p_x = 1.0f;
        uplink.p_y = 2.0f;
        uplink.v_x = 0.5f;
        uplink.v_y = 0.1f;
        uplink.yaw = 1.57f;
        uplink.status_flags = static_cast<uint8_t>(StatusFlag::EskfOk);
        uplink.reserved = 0;
        uplink.crc16 = 0;

        uint8_t buf[64];
        std::size_t n = serialize_uplink(uplink, buf);
        if (n != sizeof(UplinkPacket)) {
            printf("[FAIL] 길이 잘못 (%zu, expected 31)\n", n);
            return 1;
        }
        if (buf[0] != SYNC_UPLINK) {
            printf("[FAIL] sync 잘못\n"); return 1;
        }
        printf("  OK (sync=0x%02X, 길이=%zu)\n", buf[0], n);
    }

    printf("\n[OK] 패킷 직렬화/파싱 검증 통과\n");
    return 0;
}