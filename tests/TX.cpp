#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <chrono>

struct TorpedoTelemetry {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float vel_x,   vel_y,   vel_z;
    float pos_x,   pos_y,   pos_z;
    float distance;
    uint32_t timestamp;
    uint32_t seq;
};

int openSerial(const char* port, speed_t baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { perror("open"); return -1; }
    termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

int main() {
    int fd = openSerial("/dev/ttyS2", B460800);
    if (fd < 0) return 1;

    const float TARGET_DIST = 50.0f;
    const float ACCEL_PHASE = 2.0f;
    const float CRUISE_VEL  = 5.0f;
    const float DT          = 0.00125f;  // 800Hz

    float t = 0.0f, pos_x = 0.0f, vel_x = 0.0f, accel_x = 0.0f;
    float gyro_base = 0.002f;
    uint32_t seq = 0;

    auto start = std::chrono::steady_clock::now();
    std::cout << "=== Torpedo TX start ===" << std::endl;

    while (pos_x < TARGET_DIST) {
        auto loop_start = std::chrono::steady_clock::now();

        TorpedoTelemetry data{};
        accel_x = (t < ACCEL_PHASE) ? CRUISE_VEL / ACCEL_PHASE : 0.0f;
        vel_x += accel_x * DT;
        if (vel_x > CRUISE_VEL) vel_x = CRUISE_VEL;
        pos_x += vel_x * DT;

        float noise = 0.02f * sinf(t * 3.14f);
        data.accel_x  = accel_x;
        data.accel_y  = noise;
        data.accel_z  = 9.8f + noise;
        data.gyro_x   = gyro_base * sinf(t * 1.5f);
        data.gyro_y   = gyro_base * cosf(t * 1.2f);
        data.gyro_z   = noise * 0.5f;
        data.vel_x    = vel_x;
        data.vel_y    = noise * 0.1f;
        data.vel_z    = noise * 0.1f;
        data.pos_x    = pos_x;
        data.pos_y    = noise * 0.5f;
        data.pos_z    = noise * 0.5f;
        data.distance = TARGET_DIST - pos_x;
        data.timestamp = static_cast<uint32_t>(t * 1000);
        data.seq      = seq++;

        write(fd, &data, sizeof(data));
        t += DT;

        auto loop_end = loop_start + std::chrono::microseconds(static_cast<int>(DT * 1000000));
        while (std::chrono::steady_clock::now() < loop_end) {}
    }

    float total = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "=== Target hit ===" << std::endl;
    std::cout << "Total flight time: " << total << "s" << std::endl;

    close(fd);
    return 0;
}
