#ifndef TARGET_STATE_ESTIMATOR_HPP_
#define TARGET_STATE_ESTIMATOR_HPP_

#include "GuidanceTypes.hpp"
#include <Eigen/Dense>

class TargetStateEstimator {
private:
	TargetState state_;
	float lpf_alpha_ = 0.2f;

public:
	TargetStateEstimator() = default;
	~TargetStateEstimator() = default;

	void updateFromLidar(const Eigen::Vector2f& raw_pos, float dt);
	void predictVirtualState(float dt);
	Eigen::Vector2f calculatePIP(const Eigen::Vector2f& my_pos, float my_speed) const;
	const TargetState& getState() const { return state_; }
};

#endif /* TARGET_STATE_ESTIMATOR_HPP_ */
