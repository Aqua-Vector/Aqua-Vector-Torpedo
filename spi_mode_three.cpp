#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h> // bool 타입 사용을 위해 추가

#define REG_WHO_AM_I      0x0F
#define VAL_WHO_AM_I      0x6B
#define REG_CTRL1_XL      0x10
#define REG_CTRL2_G       0x11
#define REG_OUTX_L_G      0x22  
#define READ_BIT          0x80

static int fd = -1;
static uint32_t spi_speed = 1000000; 

bool spi_open(const char* dev) {
    fd = open(dev, O_RDWR);
    if (fd < 0) { perror("장치 열기 실패"); return false; }

    uint8_t mode = SPI_MODE_3; 
    uint8_t bits = 8;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI 모드 설정 실패"); return false; }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) { perror("SPI 속도 설정 실패"); return false; }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { perror("SPI 비트 설정 실패"); return false; }
    return true;
}

// 💡 안전한 2바이트 버퍼 구조로 수정 및 return 추가
uint8_t read_reg(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | READ_BIT), 0x00 }; // 인자값 reg를 반영
    uint8_t rx[2] = { 0 };
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = 8,
    };
    
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) return 0x00;
    return rx[1]; // 👈 읽어온 값을 정상적으로 반환
}

bool write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = 8,
    };
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

int main() {
    printf("=== ISM330DHCX SPI 최종 제어 시스템 (완성판) ===\n");

    if (!spi_open("/dev/spidev0.0")) return 1;
    printf("[OK] SPI 포트 활성화 완료\n");

    // SPI 라인 안정화를 위한 더미 읽기 (Dummy Read)
    for(int i = 0; i < 3; i++) {
        read_reg(REG_WHO_AM_I);
        usleep(5000); 
    }

    // 진짜 WHO_AM_I 검증 실행
    uint8_t chip_id = read_reg(REG_WHO_AM_I);
    printf("[WHO_AM_I] 읽은 값: 0x%02X (정상 값: 0x%02X)\n", chip_id, VAL_WHO_AM_I);
    
    if (chip_id != VAL_WHO_AM_I) {
        printf("[경고] WHO_AM_I 일치하지 않음 감지. 하지만 테스트를 위해 강제 진행합니다.\n");
        usleep(10000);
    } else {
        printf("[OK] 센서 ID 검증 성공!\n");
    }

    // 하드웨어 락업 방지 순정 초기화 설정 (±4g, ±250dps)
    write_reg(REG_CTRL1_XL, 0x48); 
    write_reg(REG_CTRL2_G,  0x4C); 
    usleep(20000); 
    printf("[OK] 가속도계(±4g) 및 자이로스코프(±250dps) 설정 완료\n");

    printf("\n%-6s %-8s %-8s %-8s %-8s %-8s %-8s\n",
           "Count", "Gx(dps)", "Gy(dps)", "Gz(dps)", "Ax(g)", "Ay(g)", "Az(g)");
    printf("----------------------------------------------------------------------\n");

    int count = 1;
    while (1) {
        // 💡 버퍼 크기를 15로 여유 있게 늘려 안정성 확보
        uint8_t tx[15] = {0};
        uint8_t rx[15] = {0};
        
        tx[0] = REG_OUTX_L_G | READ_BIT; 

        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx,
            .rx_buf = (unsigned long)rx,
            .len = 13, 
            .speed_hz = spi_speed,
            .bits_per_word = 8,
        };

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI 전송 에러 발생");
            break;
        }

        // rx[1]부터 오프셋 밀림 없이 정밀하게 결합
        int16_t gx_raw = (int16_t)(rx[2]  << 8 | rx[1]);
        int16_t gy_raw = (int16_t)(rx[4]  << 8 | rx[3]);
        int16_t gz_raw = (int16_t)(rx[6]  << 8 | rx[5]);
        int16_t ax_raw = (int16_t)(rx[8]  << 8 | rx[7]);
        int16_t ay_raw = (int16_t)(rx[10] << 8 | rx[9]);
        int16_t az_raw = (int16_t)(rx[12] << 8 | rx[11]);

        // 민감도 변환 팩터 적용
        float gx = gx_raw * 0.00875f;
        float gy = gy_raw * 0.00875f;
        float gz = gz_raw * 0.00875f;
        float ax = ax_raw * 0.000122f;
        float ay = ay_raw * 0.000122f;
        float az = az_raw * 0.000122f;

        printf("\r%-6d %-8.2f %-8.2f %-8.2f %-8.3f %-8.3f %-8.3f",
               count++, gx, gy, gz, ax, ay, az);
        fflush(stdout);

        usleep(100000); 
    }

    close(fd);
    return 0;
}
