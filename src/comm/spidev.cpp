#include "torpedo/comm/spidev.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <cstring>
#include <cstdio>

namespace torpedo {

SpiDevice::~SpiDevice() {
    close();
}

bool SpiDevice::open(const SpiConfig& cfg) {
    if (fd_ >= 0) {
        std::fprintf(stderr, "[spi] 이미 열려있음\n");
        return false;
    }
    
    fd_ = ::open(cfg.device.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::fprintf(stderr, "[spi] open(%s) 실패: %s\n",
                     cfg.device.c_str(), std::strerror(errno));
        return false;
    }
    
    // SPI mode
    if (ioctl(fd_, SPI_IOC_WR_MODE, &cfg.mode) < 0) {
        std::fprintf(stderr, "[spi] mode 설정 실패: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }
    
    // Bits per word
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &cfg.bits) < 0) {
        std::fprintf(stderr, "[spi] bits 설정 실패: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }
    
    // Speed
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &cfg.speed_hz) < 0) {
        std::fprintf(stderr, "[spi] speed 설정 실패: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }
    
    speed_hz_ = cfg.speed_hz;
    return true;
}

void SpiDevice::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SpiDevice::transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) {
    if (fd_ < 0) return false;
    
    struct spi_ioc_transfer xfer;
    std::memset(&xfer, 0, sizeof(xfer));
    
    xfer.tx_buf = reinterpret_cast<unsigned long>(tx);
    xfer.rx_buf = reinterpret_cast<unsigned long>(rx);
    xfer.len    = len;
    xfer.speed_hz = speed_hz_;
    xfer.bits_per_word = 8;
    
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::fprintf(stderr, "[spi] transfer 실패: %s\n", std::strerror(errno));
        return false;
    }
    
    return true;
}

bool SpiDevice::write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg, value };  // bit 7 = 0 (write)
    uint8_t rx[2] = { 0, 0 };
    return transfer(tx, rx, 2);
}

bool SpiDevice::read_reg(uint8_t reg, uint8_t& out) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0 };  // bit 7 = 1 (read)
    uint8_t rx[2] = { 0, 0 };
    if (!transfer(tx, rx, 2)) return false;
    out = rx[1];
    return true;
}

bool SpiDevice::read_regs(uint8_t reg, uint8_t* buf, std::size_t len) {
    // multi-byte read: 1 byte 명령 + len byte 응답
    std::size_t total = len + 1;
    uint8_t tx[64] = {0};
    uint8_t rx[64] = {0};
    
    if (total > sizeof(tx)) {
        std::fprintf(stderr, "[spi] read_regs len 너무 큼: %zu\n", len);
        return false;
    }
    
    tx[0] = reg | 0x80;  // read
    if (!transfer(tx, rx, total)) return false;
    
    std::memcpy(buf, &rx[1], len);
    return true;
}

} // namespace torpedo