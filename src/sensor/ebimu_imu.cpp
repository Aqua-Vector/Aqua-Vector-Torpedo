#include "ebimu/sensor/ebimu_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <errno.h>

namespace ebimu {

namespace {
constexpr float G_TO_MS2   = 9.80665f;
constexpr float DEG_TO_RAD = 0.017453292519943295f;  // π/180

// 921600 baud 매핑
speed_t baud_to_speed(uint32_t baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return B921600;
    }
}
}  // namespace

bool EbimuImu::init() {
    if (fd_ >= 0) {
        std::fprintf(stderr, "[ebimu] 이미 열려있음\n");
        return false;
    }
    
    // [Fix] O_RDWR로 열기 (Petalinux 등 일부 시스템에서 O_RDONLY 시 tcsetattr이 제대로 안 먹힐 수 있음)
    fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::fprintf(stderr, "[ebimu] open(%s) 실패: %s\n",
                     cfg_.device.c_str(), std::strerror(errno));
        return false;
    }
    
    struct termios opts;
    if (tcgetattr(fd_, &opts) < 0) {
        std::fprintf(stderr, "[ebimu] tcgetattr 실패: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }
    
    // [Fix] cfmakeraw를 사용하여 깨끗한 raw mode 상태로 시작
    cfmakeraw(&opts);
    
    speed_t sp = baud_to_speed(cfg_.baud);
    cfsetispeed(&opts, sp);
    cfsetospeed(&opts, sp);
    
    opts.c_cflag |= (CLOCAL | CREAD);
    opts.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    opts.c_cflag |= CS8;
    
    // [Fix] 하드웨어 흐름 제어 명시적 비활성화
    opts.c_cflag &= ~CRTSCTS;
    
    // 100ms timeout
    opts.c_cc[VMIN]  = 1; // [Fix] 최소 1바이트는 기다리도록 설정
    opts.c_cc[VTIME] = static_cast<cc_t>(cfg_.read_timeout_ms / 100); 
    if (opts.c_cc[VTIME] < 1) opts.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd_, TCSANOW, &opts) < 0) {
        std::fprintf(stderr, "[ebimu] tcsetattr 실패: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }
    
    // [Fix] 버퍼 비우기
    tcflush(fd_, TCIOFLUSH);
    
    // 초기 노이즈 비우기 (5번 dummy read)
    char tmp[256];
    for (int i = 0; i < 5; i++) {
        // VMIN=1 이므로 데이터가 없으면 VTIME만큼 기다림
        ::read(fd_, tmp, sizeof(tmp));
        usleep(10000);
    }
    
    buf_len_ = 0;
    
    std::printf("[ebimu] OK: %s @ %u bps (VMIN=1, VTIME=%d)\n", 
                cfg_.device.c_str(), cfg_.baud, opts.c_cc[VTIME]);
    return true;
}

bool EbimuImu::read_sample(ImuSample& out) {
    if (fd_ < 0) return false;
    
    char c;
    // 1바이트씩 읽으며 완결된 줄을 찾음.
    // O_RDWR/VMIN=1 설정으로 인해 데이터가 있으면 즉시 반환, 없으면 timeout까지 대기.
    while (::read(fd_, &c, 1) > 0) {
        if (c == '\n' || c == '\r') {
            if (buf_len_ > 0) {
                buf_[buf_len_] = 0;
                bool ok = parse_line(buf_, out);
                buf_len_ = 0;
                if (ok) {
                    out.t_us = monotonic_us();
                    return true;
                }
            }
        } else {
            if (buf_len_ < static_cast<int>(sizeof(buf_)) - 1) {
                buf_[buf_len_++] = c;
            } else {
                buf_len_ = 0; // 오버플로우
            }
        }
    }
    return false;
}

void EbimuImu::flush() {
    if (fd_ < 0) return;
    tcflush(fd_, TCIFLUSH);
    buf_len_ = 0;
}

bool EbimuImu::parse_line(const char* line, ImuSample& out) {
    if (!line) return false;
    
    // "100-0," 헤더 건너뛰기
    const char* p = std::strchr(line, ',');
    if (!p) return false;
    p++;
    
    float qw, qx, qy, qz;
    float gx_dps, gy_dps, gz_dps;
    float ax_g, ay_g, az_g;
    
    int matched = std::sscanf(p, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                              &qw, &qx, &qy, &qz,
                              &gx_dps, &gy_dps, &gz_dps,
                              &ax_g, &ay_g, &az_g);
    if (matched != 10) return false;
    
    // 가속도: g → m/s²
    out.ax = ax_g * G_TO_MS2;
    out.ay = ay_g * G_TO_MS2;
    out.az = az_g * G_TO_MS2;
    
    // 자이로: dps → rad/s
    out.gx = gx_dps * DEG_TO_RAD;
    out.gy = gy_dps * DEG_TO_RAD;
    out.gz = gz_dps * DEG_TO_RAD;
    
    // EBIMU AHRS quaternion (그대로)
    out.qw = qw;
    out.qx = qx;
    out.qy = qy;
    out.qz = qz;
    
    // t_us는 read_sample에서 박음 (parse_line은 시간 모름)
    
    return true;
}

void EbimuImu::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace ebimu
