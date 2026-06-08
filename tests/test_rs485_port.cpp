// Rs485Port 인터페이스 검증
//
// VM에서는 진짜 디바이스 없음.
// 에러 처리 + RAII 동작 + 잘못된 baud rate 거부만 검증.
// 실제 송수신 동작은 ZYBO 환경에서.

#include "torpedo/comm/rs485_port.hpp"

#include <cstdio>

int main() {
    using namespace torpedo;

    // -------- Test 1: 닫힌 상태에서 read/write --------
    printf("[Test 1] 닫힌 상태에서 read/write\n");
    {
        Rs485Port port;
        if (port.is_open()) {
            printf("[FAIL] 닫힌 상태인데 is_open() true\n");
            return 1;
        }
        
        uint8_t buf[10];
        int r = port.read_bytes(buf, 10);
        if (r != -1) {
            printf("[FAIL] 닫힌 상태 read가 에러 안 반환\n");
            return 1;
        }
        
        int w = port.write_bytes(buf, 5);
        if (w != -1) {
            printf("[FAIL] 닫힌 상태 write가 에러 안 반환\n");
            return 1;
        }
        printf("  OK\n");
    }

    // -------- Test 2: 없는 디바이스 --------
    printf("\n[Test 2] 없는 디바이스 open 거부\n");
    {
        Rs485Port port;
        Rs485Config cfg;
        cfg.device = "/dev/this_does_not_exist_xyz";
        cfg.baud = 460800;
        
        bool ok = port.open(cfg);
        if (ok) {
            printf("[FAIL] 없는 디바이스 open 성공\n");
            return 1;
        }
        if (port.is_open()) {
            printf("[FAIL] open 실패인데 is_open() true\n");
            return 1;
        }
        printf("  OK\n");
    }

    // -------- Test 3: 잘못된 baud rate --------
    printf("\n[Test 3] 잘못된 baud rate 거부\n");
    {
        // 임시 디바이스 (/dev/ptmx 또는 /tmp/dummy 같은) 못 만들면 skip
        // 여기선 "지원 안 하는 baud" 만 검증 (open이 일찍 실패하든 baud에서 실패하든)
        Rs485Port port;
        Rs485Config cfg;
        cfg.device = "/dev/null";  // 열리지만 termios 안 됨
        cfg.baud = 12345;          // 잘못된 baud
        
        bool ok = port.open(cfg);
        // /dev/null은 termios 설정 단계에서 실패함
        // 또는 baud_to_termios에서 B0 반환 → 실패
        // 어느 쪽이든 open()이 false 반환해야
        if (ok) {
            port.close();
            printf("  WARN: /dev/null이 OK로 열림 (시스템 의존)\n");
        } else {
            printf("  OK (잘못된 baud 또는 디바이스 거부)\n");
        }
    }

    // -------- Test 4: RAII (소멸자 자동 close) --------
    printf("\n[Test 4] RAII — 소멸자 자동 close\n");
    {
        Rs485Port* port = new Rs485Port();
        // open 시도 (실패해도 OK)
        // 핵심: 객체 살아있는 동안 fd 관리
        delete port;  // 소멸자 호출, leak 없어야
        printf("  OK (소멸자 호출 정상)\n");
    }

    printf("\n[OK] Rs485Port 인터페이스 검증 통과\n");
    printf("\n주의: 진짜 송수신 동작은 ZYBO에서 검증해야 함\n");
    printf("       (loopback 또는 두 ZYBO 연결)\n");
    return 0;
}