// imu_downsampler.hpp — IMU 다운샘플러 + 신호 처리
//
// 100Hz 메인 루프 한 사이클 안에서 8 sub-samples를 수집한 뒤,
// Trimmed Mean (각 축 최대/최소 1개 제외 후 남은 6개 평균)으로
// spike 제거 + 노이즈 감소.
//
// 사용:
//   ImuDownsampler down;
//   down.reset();
//   for (int i = 0; i < 8; i++) {
//       ImuSample s; imu.read(s);
//       down.add(s);
//   }
//   ImuSample filtered = down.finalize();  // 신호 처리된 1샘플
//

#pragma once

#include "torpedo/sensor/imu_sample.hpp"

namespace torpedo {

constexpr int IMU_SUB_SAMPLES = 8;

class ImuDownsampler {
public:
    ImuDownsampler() { reset(); }
    
    /// 버퍼 초기화 (사이클 시작 시 호출)
    void reset();
    
    /// sub-sample 추가
    /// @return true if 가득 참 (8개)
    bool add(const ImuSample& s);
    
    /// Trimmed Mean 결과 (가득 찼을 때 호출)
    /// 8개 sub-samples 각 축마다:
    ///   - 정렬 후 최대/최소 1개씩 제외
    ///   - 남은 6개 평균
    /// 타임스탬프 = 가장 최근 sub-sample의 t_us
    ImuSample finalize() const;
    
    /// 현재 모인 개수
    int count() const { return count_; }
    bool is_full() const { return count_ >= IMU_SUB_SAMPLES; }
    
private:
    ImuSample buffer_[IMU_SUB_SAMPLES];
    int       count_ = 0;
};

} // namespace torpedo