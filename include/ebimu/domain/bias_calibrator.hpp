// bias_calibrator.hpp — Nav frame bias + 시작 자세 캘리브
#pragma once

#include "ebimu/sensor/imu_sample.hpp"
#include <cstdint>

namespace ebimu {

struct BiasCalibration {
    bool   valid = false;
    
    // Nav frame bias (m/s²)
    float  bx = 0.0f;
    float  by = 0.0f;
    float  bz = 0.0f;
    
    // 시작 자세 (정렬용) — q_start.conjugate() × q_current = 상대 자세
    float  q_start_w = 1.0f;
    float  q_start_x = 0.0f;
    float  q_start_y = 0.0f;
    float  q_start_z = 0.0f;
    
    int    n_samples = 0;
};

class BiasCalibrator {
public:
    explicit BiasCalibrator(uint64_t duration_us = 5000000ULL,
                            float expected_g = 9.80665f)
        : duration_us_(duration_us), g_(expected_g) {}
    
    void start();
    bool is_collecting() const { return collecting_; }
    void add_sample(const ImuSample& s);
    BiasCalibration finalize();
    float progress() const;

private:
    uint64_t duration_us_ = 5000000ULL;
    float    g_           = 9.80665f;
    
    bool     collecting_ = false;
    uint64_t t_start_us_ = 0;
    
    int   n_ = 0;
    float ax_nav_sum_ = 0.0f;
    float ay_nav_sum_ = 0.0f;
    float az_nav_sum_ = 0.0f;
    
    // 평균 quaternion (시작 자세)
    float qw_sum_ = 0.0f, qx_sum_ = 0.0f, qy_sum_ = 0.0f, qz_sum_ = 0.0f;
};

}  // namespace ebimu