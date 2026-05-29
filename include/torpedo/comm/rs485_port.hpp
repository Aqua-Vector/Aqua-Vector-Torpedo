// rs485_port.hpp — RS-485 시리얼 포트 (termios 래퍼)

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace torpedo {

struct Rs485Config {
    std::string device = "/dev/ttyPS1";
    int         baud   = 460800;
    int         read_timeout_ms = 50;
};

class Rs485Port {
public:
    Rs485Port() = default;
    ~Rs485Port();

    Rs485Port(const Rs485Port&) = delete;
    Rs485Port& operator=(const Rs485Port&) = delete;

    bool open(const Rs485Config& cfg);
    void close();
    int read_bytes(uint8_t* buf, std::size_t max_len);
    int write_bytes(const uint8_t* buf, std::size_t len);
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;
    int read_timeout_ms_ = 50;
};

} // namespace torpedo