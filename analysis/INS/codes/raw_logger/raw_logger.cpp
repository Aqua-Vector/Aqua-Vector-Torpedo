// raw_logger.cpp - Allan Variance용 raw IMU 데이터 수집
//
// 빌드: g++ -O2 -std=c++17 -o raw_logger raw_logger.cpp
// 실행: ./raw_logger 3600 imu_1h.csv     # 3600초 = 1시간
//
// 출력 CSV 형식:
//   t_us, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
//   t_us:   monotonic 마이크로초 (측정 시작 후)
//   ax/ay/az: g 단위 (raw, 보정 없음)
//   gx/gy/gz: deg/s 단위 (raw, 보정 없음)

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <signal.h>

// ─── SPI 설정 ──────────────────────────────────────
// IMPORTANT: v3.4 코드와 동일한 경로 사용
// 모르면 ZYBO에서: ls /dev/spidev*
constexpr const char* SPI_DEV = "/dev/spidev1.0";   // ← 환경에 맞게 수정
constexpr uint32_t SPI_HZ     = 5000000;
constexpr uint8_t  SPI_MODE   = SPI_MODE_3;
constexpr uint8_t  SPI_BITS   = 8;

// ─── ISM330DHCX 레지스터 ──────────────────────────
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G  = 0x11;
constexpr uint8_t CTRL3_C  = 0x12;
constexpr uint8_t CTRL4_C  = 0x13;
constexpr uint8_t CTRL6_C  = 0x15;
constexpr uint8_t CTRL8_XL = 0x17;
constexpr uint8_t OUTX_L_G = 0x22;

constexpr uint8_t CFG_CTRL1_XL = 0x70;  // 833Hz, ±2g
constexpr uint8_t CFG_CTRL2_G  = 0x70;  // 833Hz, ±250dps
constexpr uint8_t CFG_CTRL3_C  = 0x44;
constexpr uint8_t CFG_CTRL4_C  = 0x02;
constexpr uint8_t CFG_CTRL6_C  = 0x03;
constexpr uint8_t CFG_CTRL8_XL = 0x40;

constexpr float ACC_SENS_G   = 2.0f / 32768.0f;
constexpr float GYR_SENS_DPS = 250.0f / 32768.0f;

volatile sig_atomic_t g_running = 1;
void on_signal(int) { g_running = 0; }

int spi_read(int fd, uint8_t addr, uint8_t* buf, size_t n) {
    uint8_t tx[1 + 16] = {0};
    uint8_t rx[1 + 16] = {0};
    if (n > 16) return -1;
    tx[0] = addr | 0x80;

    spi_ioc_transfer xfer{};
    xfer.tx_buf = (uintptr_t)tx;
    xfer.rx_buf = (uintptr_t)rx;
    xfer.len = 1 + n;
    xfer.speed_hz = SPI_HZ;
    xfer.bits_per_word = SPI_BITS;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) return ret;
    memcpy(buf, rx + 1, n);
    return 0;
}

int spi_write(int fd, uint8_t addr, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), val };
    uint8_t rx[2] = {0};

    spi_ioc_transfer xfer{};
    xfer.tx_buf = (uintptr_t)tx;
    xfer.rx_buf = (uintptr_t)rx;
    xfer.len = 2;
    xfer.speed_hz = SPI_HZ;
    xfer.bits_per_word = SPI_BITS;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
}

uint64_t now_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <duration_sec> <output.csv>\n", argv[0]);
        fprintf(stderr, "Example: %s 3600 imu_1h.csv  # 1시간 측정\n", argv[0]);
        return 1;
    }
    int duration_sec = atoi(argv[1]);
    const char* csv_path = argv[2];

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int fd = open(SPI_DEV, O_RDWR);
    if (fd < 0) { perror("SPI open"); return 1; }

    uint32_t hz = SPI_HZ;
    uint8_t mode = SPI_MODE, bits = SPI_BITS;
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz);
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);

    uint8_t who = 0;
    spi_read(fd, WHO_AM_I, &who, 1);
    fprintf(stderr, "[INIT] WHO_AM_I = 0x%02X (expect 0x6B)\n", who);
    if (who != 0x6B) {
        fprintf(stderr, "[ERROR] IMU not detected. Check SPI_DEV path.\n");
        close(fd);
        return 1;
    }

    spi_write(fd, CTRL3_C,  CFG_CTRL3_C);
    usleep(1000);
    spi_write(fd, CTRL1_XL, CFG_CTRL1_XL);
    spi_write(fd, CTRL2_G,  CFG_CTRL2_G);
    spi_write(fd, CTRL4_C,  CFG_CTRL4_C);
    spi_write(fd, CTRL6_C,  CFG_CTRL6_C);
    spi_write(fd, CTRL8_XL, CFG_CTRL8_XL);
    fprintf(stderr, "[INIT] IMU configured at 833Hz\n");

    // 5분 워밍업
    fprintf(stderr, "[WARMUP] 5 minutes... DO NOT TOUCH the board!\n");
    for (int i = 5; i > 0 && g_running; i--) {
        fprintf(stderr, "  warmup %d min remaining...\n", i);
        sleep(60);
    }
    if (!g_running) { close(fd); return 0; }
    fprintf(stderr, "[WARMUP] Done. Starting measurement.\n");

    FILE* csv = fopen(csv_path, "w");
    if (!csv) { perror("CSV open"); close(fd); return 1; }
    fprintf(csv, "t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");

    uint64_t t_start = now_us();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000ULL;
    uint64_t sample_count = 0;
    uint64_t error_count = 0;
    uint64_t last_print = t_start;

    while (g_running) {
        uint64_t t_now = now_us();
        if (t_now >= t_end) break;

        uint8_t buf[12];
        if (spi_read(fd, OUTX_L_G, buf, 12) < 0) {
            error_count++;
            usleep(100);
            continue;
        }

        int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);
        int16_t ax_raw = (int16_t)((buf[7] << 8) | buf[6]);
        int16_t ay_raw = (int16_t)((buf[9] << 8) | buf[8]);
        int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

        float ax = ax_raw * ACC_SENS_G;
        float ay = ay_raw * ACC_SENS_G;
        float az = az_raw * ACC_SENS_G;
        float gx = gx_raw * GYR_SENS_DPS;
        float gy = gy_raw * GYR_SENS_DPS;
        float gz = gz_raw * GYR_SENS_DPS;

        fprintf(csv, "%lu,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
                t_now - t_start, ax, ay, az, gx, gy, gz);
        sample_count++;

        if (t_now - last_print >= 60000000ULL) {
            int elapsed_min = (t_now - t_start) / 60000000ULL;
            int total_min = duration_sec / 60;
            fprintf(stderr, "[PROGRESS] %d/%d min, samples=%lu, errors=%lu\n",
                    elapsed_min, total_min, sample_count, error_count);
            fflush(csv);
            last_print = t_now;
        }

        uint64_t loop_dt = now_us() - t_now;
        if (loop_dt < 1200) usleep(1200 - loop_dt);
    }

    fclose(csv);
    close(fd);
    fprintf(stderr, "[DONE] Total samples=%lu, errors=%lu\n",
            sample_count, error_count);
    fprintf(stderr, "[DONE] Output: %s\n", csv_path);
    return 0;
}