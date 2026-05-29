#include "torpedo/sensor/ism330dhcx_imu.hpp"
#include "torpedo/sensor/ism330dhcx_regs.hpp"
#include "torpedo/hal/system_clock.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace torpedo {

using namespace ism330dhcx;

bool Ism330dhcxImu::init() {
    // -------- 1. SPI 열기 --------
    SpiConfig spi_cfg;
    spi_cfg.device   = cfg_.spi_device;
    spi_cfg.speed_hz = cfg_.spi_speed_hz;
    spi_cfg.mode     = 3;
    spi_cfg.bits     = 8;
    
    if (!spi_.open(spi_cfg)) {
        std::fprintf(stderr, "[ism330] SPI open 실패\n");
        return false;
    }
    
    // -------- 2. Chip ID 검증 --------
    if (!verify_chip_id()) {
        std::fprintf(stderr, "[ism330] chip ID 검증 실패\n");
        spi_.close();
        return false;
    }
    
    // -------- 3. 센서 설정 --------
    if (!configure_sensors()) {
        std::fprintf(stderr, "[ism330] 센서 설정 실패\n");
        spi_.close();
        return false;
    }
    
    accel_sens_ = ACCEL_SENS_4G;
    gyro_sens_  = GYRO_SENS_250DPS;
    
    initialized_ = true;
    std::printf("[ism330] 초기화 OK (ODR 833Hz, ±4g, ±250 dps)\n");
    return true;
}

void Ism330dhcxImu::shutdown() {
    if (initialized_) {
        spi_.close();
        initialized_ = false;
    }
}

bool Ism330dhcxImu::verify_chip_id() {
    uint8_t id = 0;
    if (!spi_.read_reg(WHO_AM_I, id)) {
        std::fprintf(stderr, "[ism330] WHO_AM_I 읽기 실패\n");
        return false;
    }
    
    if (id != WHO_AM_I_VAL) {
        std::fprintf(stderr, "[ism330] WHO_AM_I 불일치: 읽음=0x%02X, 기대=0x%02X\n",
                     id, WHO_AM_I_VAL);
        return false;
    }
    
    std::printf("[ism330] WHO_AM_I = 0x%02X OK\n", id);
    return true;
}

bool Ism330dhcxImu::configure_sensors() {
    if (!spi_.write_reg(CTRL3_C, CTRL3_SW_RESET)) return false;
    usleep(20000);
    
    if (!spi_.write_reg(CTRL3_C, CTRL3_BDU | CTRL3_IF_INC)) return false;
    
    uint8_t ctrl1 = static_cast<uint8_t>(Odr::Hz833) 
                  | static_cast<uint8_t>(AccelRange::G4);
    if (!spi_.write_reg(CTRL1_XL, ctrl1)) return false;
    
    uint8_t ctrl2 = static_cast<uint8_t>(Odr::Hz833)
                  | static_cast<uint8_t>(GyroRange::Dps250);
    if (!spi_.write_reg(CTRL2_G, ctrl2)) return false;
    
    usleep(20000);
    return true;

    // ── 확인용 readback 추가 ──
    uint8_t check1 = 0, check2 = 0, check3 = 0;
    spi_.read_reg(CTRL1_XL, check1);
    spi_.read_reg(CTRL2_G,  check2);
    spi_.read_reg(CTRL3_C,  check3);
    std::printf("[DEBUG] CTRL1_XL=0x%02X CTRL2_G=0x%02X CTRL3_C=0x%02X\n",
        check1, check2, check3);
    std::printf("[DEBUG] 기대값: CTRL1_XL=0x%02X CTRL2_G=0x%02X\n",
        ctrl1, ctrl2);

    return true;
}

bool Ism330dhcxImu::read(ImuSample& out) {
    if (!initialized_) return false;
    
    uint8_t status = 0;
    if (!spi_.read_reg(STATUS_REG, status)) return false;
    
    if ((status & 0x03) != 0x03) {
        return false;  // 아직 새 데이터 X
    }
    
    uint8_t buf[12];
    if (!spi_.read_regs(OUTX_L_G, buf, 12)) return false;
    
    int16_t gx_raw = static_cast<int16_t>(buf[0] | (buf[1] << 8));
    int16_t gy_raw = static_cast<int16_t>(buf[2] | (buf[3] << 8));
    int16_t gz_raw = static_cast<int16_t>(buf[4] | (buf[5] << 8));
    int16_t ax_raw = static_cast<int16_t>(buf[6] | (buf[7] << 8));
    int16_t ay_raw = static_cast<int16_t>(buf[8] | (buf[9] << 8));
    int16_t az_raw = static_cast<int16_t>(buf[10] | (buf[11] << 8));
    
    out.ax = ax_raw * accel_sens_;
    out.ay = ay_raw * accel_sens_;
    out.az = az_raw * accel_sens_;
    out.gx = gx_raw * gyro_sens_;
    out.gy = gy_raw * gyro_sens_;
    out.gz = gz_raw * gyro_sens_;
    
    out.t_us = SystemClock().now_us();  // ← torpedo::hal:: 떼고 그냥 SystemClock
    out.valid = true;
    
    return true;
}



} // namespace torpedo