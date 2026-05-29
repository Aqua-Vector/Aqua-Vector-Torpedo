#pragma once
#include "ebimu/sensor/imu_sample.hpp"
#include <cstdint>
namespace ebimu {
struct BiasCalibration {
    bool valid=false;
    float bx=0,by=0,bz=0;
    float q_start_w=1,q_start_x=0,q_start_y=0,q_start_z=0;
    int n_samples=0;
};
class BiasCalibrator {
public:
    explicit BiasCalibrator(uint64_t duration_us=5000000ULL, float expected_g=9.80665f)
        : duration_us_(duration_us), g_(expected_g) {}
    void start();
    bool is_collecting() const { return collecting_; }
    void add_sample(const ImuSample& s);
    BiasCalibration finalize();
    float progress() const;
private:
    uint64_t duration_us_=5000000ULL; float g_=9.80665f;
    bool collecting_=false; uint64_t t_start_us_=0;
    int n_=0;
    // 내부 누적은 double — 장시간/고ODR에서 float 정밀도 소실 방지
    double ax_nav_sum_=0.0, ay_nav_sum_=0.0, az_nav_sum_=0.0;
    double qw_sum_=0.0, qx_sum_=0.0, qy_sum_=0.0, qz_sum_=0.0;
};
}