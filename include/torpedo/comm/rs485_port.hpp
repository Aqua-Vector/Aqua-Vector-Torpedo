// rs485_port.hpp — RS-485 시리얼 포트 (termios 래퍼)
//
// Linux serial 디바이스 (/dev/ttyPS1 등)를 RS-485로 사용.
// MAX3485 transceiver, DE/RE 고정 (GPIO 전환 X), 460,800 bps.
//
// RAII: 생성자에서 open + 설정, 소멸자에서 close.
// non-blocking I/O (timeout 처리).

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace torpedo {

/**
 * RS-485 통신 설정.
 */
struct Rs485Config {
    std::string device = "/dev/ttyPS1";   // ZYBO UART PORT
    int         baud   = 460800;
    int         read_timeout_ms = 50;      // poll 타임아웃
};

/**
 * RS-485 시리얼 포트.
 *
 * Thread-safety: 단일 스레드 사용 가정.
 *                또는 read/write를 다른 스레드에서 (각자 락 X).
 */
class Rs485Port {
public:
    /// 기본 생성자 — 닫힌 상태
    Rs485Port() = default;
    
    /// 소멸자 — 자동 close
    ~Rs485Port();
    
    // 복사 금지 (파일 핸들 단일 소유)
    Rs485Port(const Rs485Port&) = delete;
    Rs485Port& operator=(const Rs485Port&) = delete;
    
    /**
     * 포트 열기 + 설정.
     * @return true if 성공
     */
    bool open(const Rs485Config& cfg);
    
    /**
     * 포트 닫기.
     */
    void close();
    
    /**
     * 데이터 읽기 (non-blocking, timeout).
     * @return 읽은 byte 수, 0 = timeout, -1 = 에러
     */
    int read_bytes(uint8_t* buf, std::size_t max_len);
    
    /**
     * 데이터 쓰기.
     * @return 쓴 byte 수, -1 = 에러
     */
    int write_bytes(const uint8_t* buf, std::size_t len);
    
    /**
     * 포트 열려있는지.
     */
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;
    int read_timeout_ms_ = 50;
};

} // namespace torpedo