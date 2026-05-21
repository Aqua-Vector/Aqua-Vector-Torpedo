#include "torpedo/sensor/imu_downsampler.hpp"

#include <algorithm>

namespace torpedo {

void ImuDownsampler::reset() {
    count_ = 0;
}

bool ImuDownsampler::add(const ImuSample& s) {
    if (count_ < IMU_SUB_SAMPLES) {
        buffer_[count_++] = s;
    }
    return is_full();
}

// Trimmed Mean: 8개 값 → 최대/최소 1개씩 제외 → 6개 평균
static float trimmed_mean_8(const float values[8]) {
    float sorted[8];
    for (int i = 0; i < 8; i++) sorted[i] = values[i];
    std::sort(sorted, sorted + 8);
    
    // 가운데 6개 (인덱스 1~6) 평균
    float sum = 0.0f;
    for (int i = 1; i <= 6; i++) sum += sorted[i];
    return sum / 6.0f;
}

ImuSample ImuDownsampler::finalize() const {
    ImuSample out;
    out.valid = false;
    out.t_us  = 0;
    
    if (count_ < IMU_SUB_SAMPLES) {
        // 불완전 — 빈 결과
        out.ax = out.ay = out.az = 0.0f;
        out.gx = out.gy = out.gz = 0.0f;
        out.roll = out.pitch = out.yaw = 0.0f;
        return out;
    }
    
    // 각 축 8개 값 분리
    float ax[8], ay[8], az[8];
    float gx[8], gy[8], gz[8];
    float roll[8], pitch[8], yaw[8];
    for (int i = 0; i < 8; i++) {
        ax[i] = buffer_[i].ax;
        ay[i] = buffer_[i].ay;
        az[i] = buffer_[i].az;
        gx[i] = buffer_[i].gx;
        gy[i] = buffer_[i].gy;
        gz[i] = buffer_[i].gz;
        roll[i]  = buffer_[i].roll;
        pitch[i] = buffer_[i].pitch;
        yaw[i]   = buffer_[i].yaw;
    }
    
    // 각 축 Trimmed Mean
    out.ax = trimmed_mean_8(ax);
    out.ay = trimmed_mean_8(ay);
    out.az = trimmed_mean_8(az);
    out.gx = trimmed_mean_8(gx);
    out.gy = trimmed_mean_8(gy);
    out.gz = trimmed_mean_8(gz);
    out.roll  = trimmed_mean_8(roll);
    out.pitch = trimmed_mean_8(pitch);
    out.yaw   = trimmed_mean_8(yaw);
    
    // 타임스탬프 = 마지막 sub-sample
    out.t_us = buffer_[IMU_SUB_SAMPLES - 1].t_us;
    out.valid = true;
    
    return out;
}

} // namespace torpedo