#ifndef TARGET_STATE_ESTIMATOR_HPP_
#define TARGET_STATE_ESTIMATOR_HPP_

#include "GuidanceTypes.hpp"
#include <Eigen/Dense>

namespace guidance {

/**
 * @brief Kinematic Target State Estimator (Pure Dead Reckoning)
 * 
 * 설계 간소화 및 고정밀화 (Project Specific Optimization):
 * - 타겟의 등속도/정지 상태 특성을 반영하여 복잡한 KF 대신 Pure DR(추측항법) 채택
 * - 중기 유도 시 타겟 속도 벡터($V_{target}$) 확정
 * - 종말 유도(Blind Phase) 시 수학적으로 완벽한 선형 보간($P = P_{cut} + V \cdot dt$) 수행
 * - 조향 떨림(Chattering) 원천 차단 및 연산 부하 최소화
 */
class TargetStateEstimator {
private:
	TargetState current_state_;
	bool is_initialized_;

public:
	TargetStateEstimator();
	~TargetStateEstimator() = default;

	/**
	 * @brief 필터 초기화 및 리셋
	 */
	void reset();

	/**
	 * @brief 통제소 데이터 수신 시 타겟 상태 업데이트 (속도 추출 및 위치 갱신)
	 * @param raw_pos 타겟의 XY 좌표
	 * @param dt_sec 경과 시간
	 */
	void updateFromLidar(const Eigen::Vector2f& raw_pos, float dt_sec);

	/**
	 * @brief 종말 유도 시 가상 타겟 상태 예측 (수학적 직선 방정식 적용)
	 * @param dt_sec 제어 주기
	 */
	void predictVirtualState(float dt_sec);

	/**
	 * @brief 예상 요격 지점(PIP) 산출
	 */
	Eigen::Vector2f calculatePIP(const Eigen::Vector2f& my_pos, float my_speed) const;

	/**
	 * @brief 현재 추정된 타겟의 상태 객체 반환
	 */
	const TargetState& getState() const { return current_state_; }
};

} // namespace guidance

#endif /* TARGET_STATE_ESTIMATOR_HPP_ */
