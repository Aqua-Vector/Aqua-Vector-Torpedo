// spidev.hpp — Linux spidev 래퍼
//
// /dev/spidev0.0 디바이스를 통한 SPI 통신.

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace torpedo {

struct SpiConfig {
    std::string device      = "/dev/spidev0.0";
    uint32_t    speed_hz    = 10000000;   // 10 MHz (ISM330DHCX 최대 10 MHz)
    uint8_t     mode        = 0;          // SPI mode 0 (CPOL=0, CPHA=0)
    uint8_t     bits        = 8;
};

/**
 * SPI 디바이스 래퍼 (RAII).
 */
class SpiDevice {
public:
    SpiDevice() = default;
    ~SpiDevice();
    
    SpiDevice(const SpiDevice&) = delete;
    SpiDevice& operator=(const SpiDevice&) = delete;
    
    bool open(const SpiConfig& cfg);
    void close();
    bool is_open() const { return fd_ >= 0; }
    
    /**
     * SPI transfer (full-duplex).
     * tx와 rx 동시. 같은 길이.
     */
    bool transfer(const uint8_t* tx, uint8_t* rx, std::size_t len);
    
    /**
     * 레지스터 1 byte 쓰기.
     */
    bool write_reg(uint8_t reg, uint8_t value);
    
    /**
     * 레지스터 1 byte 읽기.
     */
    bool read_reg(uint8_t reg, uint8_t& out);
    
    /**
     * 레지스터 burst 읽기 (auto-increment).
     */
    bool read_regs(uint8_t reg, uint8_t* buf, std::size_t len);
    
private:
    int      fd_       = -1;
    uint32_t speed_hz_ = 0;
};

} // namespace torpedo