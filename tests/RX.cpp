#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
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
    int fd = openSerial("/dev/ttyPS1", B460800);
    if (fd < 0) return 1;

    uint8_t buf[256];
    int buf_len = 0;
    const int PKT_SIZE = sizeof(TorpedoTelemetry);
    TorpedoTelemetry data{};

    uint32_t expected_seq = 0;
    int lost_packets = 0;
    int pkt_count = 0;
    auto prev_rx_time = std::chrono::steady_clock::now();
    float max_interval = 0.0f, sum_interval = 0.0f;

    std::cout << "=== Launcher RX waiting ===" << std::endl;

    while (true) {
        int n = read(fd, buf + buf_len, sizeof(buf) - buf_len);
        if (n <= 0) continue;
        buf_len += n;

        while (buf_len >= PKT_SIZE) {
            auto rx_time = std::chrono::steady_clock::now();
            memcpy(&data, buf, PKT_SIZE);
            buf_len -= PKT_SIZE;
            memmove(buf, buf + PKT_SIZE, buf_len);

            pkt_count++;
            if (pkt_count > 1 && data.seq != expected_seq) {
                lost_packets += static_cast<int>(data.seq - expected_seq);
            }
            expected_seq = data.seq + 1;

            float interval_ms = std::chrono::duration<float, std::milli>(
                rx_time - prev_rx_time).count();
            prev_rx_time = rx_time;
            if (pkt_count > 1) {
                sum_interval += interval_ms;
                if (interval_ms > max_interval) max_interval = interval_ms;
            }

            if (data.distance <= 0.0f) {
                std::cout << "=== Target hit ===" << std::endl;
                std::cout << "Total packets  : " << pkt_count << std::endl;
                std::cout << "Lost packets   : " << lost_packets << std::endl;
                std::cout << "Loss rate      : "
                          << (float)lost_packets / (pkt_count + lost_packets) * 100.0f
                          << "%" << std::endl;
                std::cout << "Avg interval   : " << sum_interval / (pkt_count - 1) << "ms" << std::endl;
                std::cout << "Max interval   : " << max_interval << "ms" << std::endl;
                close(fd);
                return 0;
            }
        }
    }
}
