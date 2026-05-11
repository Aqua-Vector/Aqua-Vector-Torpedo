// ISM330DHCX 드라이버 — 컴파일 + 에러 처리 검증
//
// VM에서는 진짜 SPI 디바이스 없음.
// init()이 디바이스 없을 때 적절히 실패하는지만 검증.
// 진짜 동작은 ZYBO에서.

#include "torpedo/sensor/ism330dhcx_imu.hpp"
#include "torpedo/sensor/ism330dhcx_regs.hpp"

#include <cstdio>

int main() {
    using namespace torpedo;
    using namespace torpedo::ism330dhcx;

    // -------- Test 1: 레지스터 상수 검증 --------
    printf("[Test 1] 레지스터 상수\n");
    printf("  WHO_AM_I     = 0x%02X (expected 0x0F)\n", WHO_AM_I);
    printf("  WHO_AM_I_VAL = 0x%02X (expected 0x6B)\n", WHO_AM_I_VAL);
    printf("  CTRL1_XL     = 0x%02X (expected 0x10)\n", CTRL1_XL);
    printf("  OUTX_L_G     = 0x%02X (expected 0x22)\n", OUTX_L_G);
    printf("  OUTX_L_A     = 0x%02X (expected 0x28)\n", OUTX_L_A);
    
    if (WHO_AM_I != 0x0F || WHO_AM_I_VAL != 0x6B) {
        printf("[FAIL] 레지스터 상수 잘못\n");
        return 1;
    }
    printf("  OK\n");

    // -------- Test 2: ODR 비트 검증 --------
    printf("\n[Test 2] ODR 비트\n");
    printf("  Hz833 = 0x%02X (expected 0x70)\n", static_cast<uint8_t>(Odr::Hz833));
    if (static_cast<uint8_t>(Odr::Hz833) != 0x70) {
        printf("[FAIL]\n"); return 1;
    }
    printf("  OK\n");

    // -------- Test 3: Sensitivity 상수 --------
    printf("\n[Test 3] Sensitivity\n");
    printf("  ACCEL_SENS_4G    = %.6f m/s²/LSB\n", ACCEL_SENS_4G);
    printf("  GYRO_SENS_250DPS = %.6f rad/s/LSB\n", GYRO_SENS_250DPS);
    
    // 검증: ±4g range, 16-bit 출력 → full scale ≈ 4 × 9.81 = 39.24 m/s²
    float accel_full_scale = ACCEL_SENS_4G * 32767.0f;
    printf("  Accel full scale = %.2f m/s² (expected ~39 m/s² = 4g)\n", accel_full_scale);
    if (accel_full_scale < 30.0f || accel_full_scale > 50.0f) {
        printf("[FAIL] Accel sensitivity 잘못\n"); return 1;
    }
    
    float gyro_full_scale = GYRO_SENS_250DPS * 32767.0f * 180.0f / 3.14159f;
    printf("  Gyro full scale  = %.2f dps (expected ~250)\n", gyro_full_scale);
    if (gyro_full_scale < 200.0f || gyro_full_scale > 300.0f) {
        printf("[FAIL] Gyro sensitivity 잘못\n"); return 1;
    }
    printf("  OK\n");

    // -------- Test 4: 디바이스 없을 때 init 실패 --------
    printf("\n[Test 4] 없는 SPI 디바이스 거부\n");
    {
        Ism330dhcxImu imu;
        Ism330dhcxConfig cfg;
        cfg.spi_device = "/dev/this_spidev_does_not_exist";
        imu.set_config(cfg);
        
        bool ok = imu.init();
        if (ok) {
            printf("[FAIL] 없는 디바이스 init 성공\n");
            return 1;
        }
        printf("  OK (없는 디바이스 거부)\n");
    }

    // -------- Test 5: 초기화 전 read 거부 --------
    printf("\n[Test 5] init 전 read 거부\n");
    {
        Ism330dhcxImu imu;
        ImuSample s;
        if (imu.read(s)) {
            printf("[FAIL] init 전 read 성공\n");
            return 1;
        }
        printf("  OK\n");
    }

    printf("\n[OK] ISM330DHCX 드라이버 인터페이스 검증 통과\n");
    printf("\n주의: 진짜 SPI 통신 + chip 인식 + 데이터 읽기는 ZYBO에서 검증\n");
    return 0;
}