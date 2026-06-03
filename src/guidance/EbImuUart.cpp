#include "guidance/EbImuUart.hpp"
#include "utils/TimeUtils.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <algorithm>

namespace torpedo::sensor {

EbImuUart::EbImuUart(const std::string& dev, int baud)
    : device_path_(dev), baud_rate_(baud) {
    std::memset(&latest_sample_, 0, sizeof(ImuSample));
}

EbImuUart::~EbImuUart() {
    shutdown();
}

int EbImuUart::openSerial(const char* dev, int baud) {
    // Non-blocking open to prevent hanging on certain hardware conditions
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    // Optional: Try to set exclusive access
#ifdef TIOCEXCL
    ioctl(fd, TIOCEXCL);
#endif

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) {
        ::close(fd);
        return -1;
    }

    cfmakeraw(&tio);

    speed_t speed = B115200;
    switch(baud) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:     speed = B115200; break;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1; // 0.1s timeout

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        ::close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

bool EbImuUart::init() {
    fd_ = openSerial(device_path_.c_str(), baud_rate_);
    if (fd_ < 0) {
        std::cerr << "Failed to open EBIMU port: " << device_path_ << std::endl;
        return false;
    }

    if (!is_running_) {
        is_running_ = true;
        rx_thread_ = std::thread(&EbImuUart::rxLoop, this);
    }
    return true;
}

void EbImuUart::shutdown() {
    if (is_running_) {
        is_running_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }
    if (fd_ >= 0) {
        // Release exclusive access if it was set
#ifdef TIOCNXCL
        ioctl(fd_, TIOCNXCL);
#endif
        tcflush(fd_, TCIOFLUSH);
        ::close(fd_);
        fd_ = -1;
        // Small delay to let the OS clean up the descriptor
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool EbImuUart::read(ImuSample& out_sample) {
    uint32_t seq_before = 0;
    uint32_t seq_after = 0;
    const int MAX_RETRY = 3;

    for (int retry = 0; retry < MAX_RETRY; ++retry) {
        seq_before = seq_counter_.load(std::memory_order_acquire);
        if (seq_before % 2 != 0) continue;

        out_sample = latest_sample_;
        seq_after = seq_counter_.load(std::memory_order_acquire);

        if (seq_before == seq_after) {
            return out_sample.valid;
        }
    }
    return false;
}

std::vector<std::string> EbImuUart::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

void EbImuUart::parseLine(const std::string& line) {
    // Expected format: ID,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,bat
    auto tokens = split(line, ',');
    if (tokens.size() < 11) return;

    try {
        float qw = std::stof(tokens[1]);
        float qx = std::stof(tokens[2]);
        float qy = std::stof(tokens[3]);
        float qz = std::stof(tokens[4]);

        float gx_dps = std::stof(tokens[5]);
        float gy_dps = std::stof(tokens[6]);
        float gz_dps = std::stof(tokens[7]);

        float ax_g = std::stof(tokens[8]);
        float ay_g = std::stof(tokens[9]);
        float az_g = std::stof(tokens[10]);

        // Quaternion to Euler (Roll, Pitch, Yaw)
        float roll = std::atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
        float pitch = std::asin(std::clamp(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f));
        float yaw = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));

        uint32_t current_seq = seq_counter_.load(std::memory_order_relaxed);
        seq_counter_.store(current_seq + 1, std::memory_order_release);

        latest_sample_.t_us = utils::getCurrentTimeUs();
        latest_sample_.ax = ax_g * 9.80665f;
        latest_sample_.ay = ay_g * 9.80665f;
        latest_sample_.az = az_g * 9.80665f;
        latest_sample_.gx = gx_dps * (M_PI / 180.0f);
        latest_sample_.gy = gy_dps * (M_PI / 180.0f);
        latest_sample_.gz = gz_dps * (M_PI / 180.0f);
        latest_sample_.roll = roll;
        latest_sample_.pitch = pitch;
        latest_sample_.yaw = yaw;
        latest_sample_.valid = true;

        seq_counter_.store(current_seq + 2, std::memory_order_release);
    } catch (...) {
        // Parsing error, ignore line
    }
}

void EbImuUart::rxLoop() {
    std::string buffer;
    char c;
    while (is_running_) {
        ssize_t n = ::read(fd_, &c, 1);
        if (n > 0) {
            if (c == '\n') {
                if (!buffer.empty()) {
                    parseLine(buffer);
                    buffer.clear();
                }
            } else if (c != '\r') {
                buffer += c;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

} // namespace torpedo::sensor
