#ifndef RPS_TRACKER_HPP_
#define RPS_TRACKER_HPP_

#include <Eigen/Dense>

namespace torpedo::domain {

/**
 * @brief RpsPositionTracker
 * RPS 기반의 속도와 IMU의 자세 정보를 결합하여 위치를 추정하는 클래스입니다.
 * 팀원의 ESKF 코드를 수정하지 않고 별도의 위치 추산이 필요할 때 사용합니다.
 */
class RpsPositionTracker {
private:
    Eigen::Vector3f position_;
    float last_speed_ = 0.0f;

public:
    RpsPositionTracker() : position_(Eigen::Vector3f::Zero()) {}

    /**
     * @brief 위치 초기화
     */
    void reset(const Eigen::Vector3f& p0 = Eigen::Vector3f::Zero()) {
        position_ = p0;
    }

    /**
     * @brief 위치 업데이트
     * @param speed RPS로부터 계산된 전진 속도 (m/s)
     * @param q 현재 자세 (Body to Nav Quaternion)
     * @param dt 경과 시간 (s)
     */
    void update(float speed, const Eigen::Quaternionf& q, float dt) {
        // Body frame 전진 속도 벡터
        Eigen::Vector3f v_body(speed, 0.0f, 0.0f);
        
        // Nav frame으로 변환
        Eigen::Vector3f v_nav = q * v_body;
        
        // 위치 적분 (단순 오일러 적분)
        position_ += v_nav * dt;
        last_speed_ = speed;
    }

    const Eigen::Vector3f& getPosition() const { return position_; }
    float getSpeed() const { return last_speed_; }
};

} // namespace torpedo::domain

#endif /* RPS_TRACKER_HPP_ */
