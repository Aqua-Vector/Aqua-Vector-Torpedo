#include "torpedo/comm/rs485_port.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>
#include <cstring>
#include <cstdio>

namespace torpedo {

// -------- baud rate 정수 → termios 상수 매핑 --------
static speed_t baud_to_termios(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;  // 잘못된 값
    }
}

Rs485Port::~Rs485Port() {
    close();
}

bool Rs485Port::open(const Rs485Config& cfg) {
    if (fd_ >= 0) {
        std::fprintf(stderr, "[rs485] 이미 열려있음\n");
        return false;
    }
    
    // -------- 1. 디바이스 열기 (non-blocking) --------
    fd_ = ::open(cfg.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[rs485] open(%s) 실패: %s\n",
                     cfg.device.c_str(), std::strerror(errno));
        return false;
    }
    
    // -------- 2. termios 설정 가져오기 --------
    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0) {
        std::fprintf(stderr, "[rs485] tcgetattr 실패: %s\n", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // -------- 3. Raw mode 설정 --------
    // cfmakeraw 대신 명시적 설정 (호환성)
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag |= CS8 | CREAD | CLOCAL;  // 8 bit, receive enable, ignore modem control
    
    tio.c_cc[VMIN]  = 0;  // non-blocking read
    tio.c_cc[VTIME] = 0;
    
    // -------- 4. Baud rate 설정 --------
    speed_t spd = baud_to_termios(cfg.baud);
    if (spd == B0) {
        std::fprintf(stderr, "[rs485] 지원 안 하는 baud rate: %d\n", cfg.baud);
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    
    // -------- 5. 설정 적용 --------
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "[rs485] tcsetattr 실패: %s\n", std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // 버퍼 비우기 (이전 데이터 클리어)
    tcflush(fd_, TCIOFLUSH);
    
    read_timeout_ms_ = cfg.read_timeout_ms;
    
    return true;
}

void Rs485Port::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int Rs485Port::read_bytes(uint8_t* buf, std::size_t max_len) {
    if (fd_ < 0) return -1;
    
    // poll로 timeout 체크
    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    
    int ret = poll(&pfd, 1, read_timeout_ms_);
    if (ret < 0) return -1;       // 에러
    if (ret == 0) return 0;       // timeout
    
    if (pfd.revents & POLLIN) {
        ssize_t n = ::read(fd_, buf, max_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        return static_cast<int>(n);
    }
    
    return 0;
}

int Rs485Port::write_bytes(const uint8_t* buf, std::size_t len) {
    if (fd_ < 0) return -1;
    
    ssize_t n = ::write(fd_, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return static_cast<int>(n);
}

} // namespace torpedo