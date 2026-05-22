// ism330dhcx_imu.hpp — ISM330DHCX 6축 IMU 드라이버 (SPI)
//
// IImu 인터페이스 구현. SpiDevice를 통한 spidev 접근.
// ODR 833Hz, ±4g, ±250 dps (ADR-007 기반).


#pragma once

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/comm/spidev.hpp"

#include <string>

namespace torpedo {

struct Ism330dhcxConfig {
    std::string spi_device = "/dev/spidev0.0";
    uint32_t    spi_speed_hz = 1000000;  // 1 MHz
};

class Ism330dhcxImu : public IImu {
public:
    Ism330dhcxImu() = default;
    ~Ism330dhcxImu() override = default;
    
    /// 설정 보관 (init()에서 사용)
    void set_config(const Ism330dhcxConfig& cfg) { cfg_ = cfg; }
    
    // -------- IImu interface --------
    bool init() override;
    bool read(ImuSample& out) override;
    void shutdown() override;
    
private:
    Ism330dhcxConfig cfg_;
    SpiDevice        spi_;
    bool             initialized_ = false;
    
    float accel_sens_ = 0.0f;
    float gyro_sens_  = 0.0f;
    
    bool verify_chip_id();
    bool configure_sensors();
};

} // namespace torpedo