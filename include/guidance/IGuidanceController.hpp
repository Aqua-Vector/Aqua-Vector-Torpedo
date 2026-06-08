#ifndef IGUIDANCE_CONTROLLER_HPP_
#define IGUIDANCE_CONTROLLER_HPP_

#include <Eigen/Dense>
#include "torpedo/domain/estimator/eskf_state.hpp"

namespace guidance {

/**
 * @brief 유도 알고리즘 인터페이스 (Strategy Pattern)
 */
class IGuidanceController {
public:
    virtual ~IGuidanceController() = default;

    /**
     * @brief 조향각 계산
     * @param my_state 현재 기체 상태
     * @param target_pos 타겟 좌표 (또는 PIP)
     * @param dt 경과 시간
     * @return 조향각 (Degree)
     */
    virtual float calculateSteering(const torpedo::domain::EskfState& my_state, 
                                    const Eigen::Vector2f& target_pos, 
                                    float dt) = 0;

    virtual void reset() = 0;
};

} // namespace guidance

#endif /* IGUIDANCE_CONTROLLER_HPP_ */
