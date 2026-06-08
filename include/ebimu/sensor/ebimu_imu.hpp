// ebimu_imu.hpp — e2box EBIMU24GV5 UART 드라이버
//
// 출력 포맷 (ASCII, 99Hz @ 921600 bps):
//   "100-0,qw,qx,qy,qz,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,xx\n"
//
// 단위 변환:
//   가속도 g → m/s² (× 9.80665)
//   자이로 dps → rad/s (× π/180)
//
// EBIMU 자체 quaternion 융합 결과 (AHRS) ImuSample에 그대로 박힘.

#pragma once

#include "ebimu/sensor/iimu.hpp"
#include <cstdint>
#include <string>

namespace ebimu {

struct EbimuConfig {
    std::string device       = "/dev/ttyUSB0";
    uint32_t    baud         = 921600;
    int         read_timeout_ms = 100;
};

class EbimuImu : public IImu {
public:
    explicit EbimuImu(const EbimuConfig& cfg) : cfg_(cfg) {}
    ~EbimuImu() override { close(); }

    EbimuImu(const EbimuImu&) = delete;
    EbimuImu& operator=(const EbimuImu&) = delete;

    bool init() override;
    bool read_sample(ImuSample& out) override;
    void flush() override;
    void close() override;

    /// 라인 파싱 — 테스트용 public 노출.
    /// "100-0,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,xx" 형태.
    /// @return 파싱 성공 여부
    static bool parse_line(const char* line, ImuSample& out);

private:
    EbimuConfig cfg_;
    int  fd_ = -1;
    char buf_[256];
    int  buf_len_ = 0;
};

}  // namespace ebimu