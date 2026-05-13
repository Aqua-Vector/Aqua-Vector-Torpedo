// CRC16-CCITT 검증
//
// 표준 벡터: "123456789" → 0x29B1
// 단일 byte 차이 → 다른 CRC
// 빈 buffer → 0xFFFF (초기값)

#include "torpedo/comm/crc16.hpp"

#include <cstdio>
#include <cstring>

int main() {
    using namespace torpedo;
    
    // ─── Test 1: 표준 벡터 ───
    printf("[Test 1] 표준 벡터 \"123456789\" → 0x29B1\n");
    const char* test = "123456789";
    uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(test), 9);
    printf("  결과: 0x%04X (기대 0x29B1)\n", crc);
    if (crc != 0x29B1) {
        printf("[FAIL] 표준 벡터 불일치\n");
        return 1;
    }
    printf("  OK\n");
    
    // ─── Test 2: 빈 buffer → 0xFFFF ───
    printf("\n[Test 2] 빈 buffer → 0xFFFF (initial)\n");
    crc = crc16_ccitt(nullptr, 0);
    printf("  결과: 0x%04X\n", crc);
    if (crc != 0xFFFF) {
        printf("[FAIL]\n");
        return 1;
    }
    printf("  OK\n");
    
    // ─── Test 3: 단일 byte 차이 ───
    printf("\n[Test 3] 1 byte 차이 → 다른 CRC\n");
    uint8_t buf1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t buf2[] = {0x01, 0x02, 0x03, 0x05};
    uint16_t c1 = crc16_ccitt(buf1, 4);
    uint16_t c2 = crc16_ccitt(buf2, 4);
    printf("  CRC1 = 0x%04X, CRC2 = 0x%04X\n", c1, c2);
    if (c1 == c2) {
        printf("[FAIL] 같은 CRC\n");
        return 1;
    }
    printf("  OK (차이 감지)\n");
    
    // ─── Test 4: 일관성 (같은 입력 → 같은 CRC) ───
    printf("\n[Test 4] 일관성 — 같은 입력 → 같은 CRC\n");
    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    uint16_t r1 = crc16_ccitt(data, 5);
    uint16_t r2 = crc16_ccitt(data, 5);
    if (r1 != r2) {
        printf("[FAIL]\n");
        return 1;
    }
    printf("  OK (CRC=0x%04X 일관)\n", r1);
    
    printf("\n[OK] CRC16-CCITT 검증 통과\n");
    return 0;
}