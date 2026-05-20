#pragma once

#include "torpedo/domain/estimator/eskf_estimator.hpp"

namespace torpedo::domain {

/**
 * @brief EnhancedEskfEstimator
 * 팀원이 작성한 EskfEstimator를 수정하지 않고, 
 * RPS 피드백을 통한 속도 업데이트 기능을 추가하기 위해 별도로 분리한 클래스입니다.
 */
class EnhancedEskfEstimator : public EskfEstimator {
public:
    using EskfEstimator::EskfEstimator;

    /**
     * @brief 속도 측정 업데이트 (Body frame, m/s)
     * RPS 데이터를 MPS로 변환한 값을 받아 ESKF 상태를 보정합니다.
     */
    void update_velocity(const Eigen::Vector3f& z_body, float sigma_v);

private:
    // 부모 클래스의 private 멤버에 접근할 수 없으므로, 
    // 필요한 경우 EskfEstimator의 로직을 확장하거나 
    // 별도의 상태 관리 방식을 고려해야 합니다.
    // 하지만 EskfEstimator의 멤버가 private이라 상속으로도 접근이 불가능합니다.
};

} // namespace torpedo::domain
