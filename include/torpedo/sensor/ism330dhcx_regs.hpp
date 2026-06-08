// ism330dhcx_regs.hpp — ISM330DHCX 레지스터 맵 + 비트 정의
//
// 출처: ISM330DHCX Datasheet (ST Microelectronics)
//       DS12860 Rev 5, January 2020
//
// 6축 IMU (3-axis accel + 3-axis gyro), SPI/I2C 인터페이스.
// SPI 4-wire mode 사용.

#pragma once

#include <cstdint>

namespace torpedo::ism330dhcx {

// -------- Identification --------
constexpr uint8_t WHO_AM_I     = 0x0F;
constexpr uint8_t WHO_AM_I_VAL = 0x6B;  // 기대값 (chip 인식 검증)

// -------- Control Registers --------
constexpr uint8_t CTRL1_XL = 0x10;  // 가속도 제어: ODR + FS
constexpr uint8_t CTRL2_G  = 0x11;  // 자이로 제어: ODR + FS
constexpr uint8_t CTRL3_C  = 0x12;  // BDU, IF_INC, SW_RESET, BOOT
constexpr uint8_t CTRL4_C  = 0x13;
constexpr uint8_t CTRL5_C  = 0x14;
constexpr uint8_t CTRL6_C  = 0x15;
constexpr uint8_t CTRL7_G  = 0x16;
constexpr uint8_t CTRL8_XL = 0x17;
constexpr uint8_t CTRL9_XL = 0x18;
constexpr uint8_t CTRL10_C = 0x19;

// -------- Status / Data Ready --------
constexpr uint8_t STATUS_REG = 0x1E;  // bit0 = XLDA (accel), bit1 = GDA (gyro)

// -------- 데이터 출력 레지스터 --------
// Gyro (16-bit signed, little-endian)
constexpr uint8_t OUTX_L_G = 0x22;
constexpr uint8_t OUTX_H_G = 0x23;
constexpr uint8_t OUTY_L_G = 0x24;
constexpr uint8_t OUTY_H_G = 0x25;
constexpr uint8_t OUTZ_L_G = 0x26;
constexpr uint8_t OUTZ_H_G = 0x27;

// Accel (16-bit signed, little-endian)
constexpr uint8_t OUTX_L_A = 0x28;
constexpr uint8_t OUTX_H_A = 0x29;
constexpr uint8_t OUTY_L_A = 0x2A;
constexpr uint8_t OUTY_H_A = 0x2B;
constexpr uint8_t OUTZ_L_A = 0x2C;
constexpr uint8_t OUTZ_H_A = 0x2D;

// -------- CTRL3_C 비트 --------
constexpr uint8_t CTRL3_BDU    = 0x40;  // Block Data Update
constexpr uint8_t CTRL3_IF_INC = 0x04;  // Auto-increment (multi-byte read)
constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_BOOT     = 0x80;

// -------- ODR (Output Data Rate) --------
// CTRL1_XL [7:4] / CTRL2_G [7:4]
enum class Odr : uint8_t {
    PowerDown = 0x00,
    Hz12_5    = 0x10,
    Hz26      = 0x20,
    Hz52      = 0x30,
    Hz104     = 0x40,
    Hz208     = 0x50,
    Hz416     = 0x60,
    Hz833     = 0x70,
    Hz1666    = 0x80,
    Hz3333    = 0x90,
    Hz6667    = 0xA0,
};

// -------- 가속도 Range (CTRL1_XL [3:2]) --------
enum class AccelRange : uint8_t {
    G2  = 0x00,
    G16 = 0x04,
    G4  = 0x08,  // 우리 선택
    G8  = 0x0C,
};

// -------- 자이로 Range (CTRL2_G [3:1]) --------
enum class GyroRange : uint8_t {
    Dps125  = 0x02,
    Dps250  = 0x00,  // 우리 선택
    Dps500  = 0x04,
    Dps1000 = 0x08,
    Dps2000 = 0x0C,
};

// -------- Sensitivity (단위 변환) --------
// LSB → 물리 단위
// ±4g: 0.122 mg/LSB = 0.122e-3 × 9.80665 m/s²/LSB
constexpr float ACCEL_SENS_4G    = 0.122e-3f * 9.80665f;   // m/s²/LSB
// ±250 dps: 8.75 mdps/LSB = 8.75e-3 × (π/180) rad/s/LSB
constexpr float GYRO_SENS_250DPS = 8.75e-3f * 0.017453293f; // rad/s/LSB

// -------- SPI 명령 비트 --------
constexpr uint8_t SPI_READ_FLAG = 0x80;  // 읽기 명령 = 레지스터 주소 | 0x80

} // namespace torpedo::ism330dhcx