#ifndef CONTROL_HANDLERS_HPP_
#define CONTROL_HANDLERS_HPP_

#include "CommInterfaces.hpp"
#include "ControlTypes.hpp"
#include "Payloads.hpp"
#include "Marshaller.hpp"
#include "control/ManualSource.hpp"

// Forward declaration to avoid circular dependency
class TorpedoControlSystem;

// 섀시 제어 명령 처리 핸들러
class ChassisControlHandler : public IMessageHandler {
private:
    ControlState& target_state_;

public:
    explicit ChassisControlHandler(ControlState& state) : target_state_(state) {}

    bool handle(const uint8_t* payload, size_t payload_length, uint64_t timestamp_ms) override {
        if (payload_length != sizeof(ControlPayload)) return false;

        ControlPayload data;
        Marshaller::deserialize(payload, payload_length, data);

        target_state_.velocity = data.velocity;
        target_state_.rudder = data.rudder;
        target_state_.elevator = data.elevator;
        target_state_.last_update_time_ms = timestamp_ms;

        return true;
    }
};

/**
 * @brief 통제소(GCS) 명령 처리 핸들러
 * 수신된 GCS 데이터를 TCS와 ManualSource에 배분합니다.
 */
class ControlStationHandler : public IMessageHandler {
private:
    TorpedoControlSystem& tcs_;
    ManualSource& ms_;
    float target_velocity_; // [Add] 가변 속도 지원

public:
    ControlStationHandler(TorpedoControlSystem& tcs, ManualSource& ms, float velocity = 100.0f) 
        : tcs_(tcs), ms_(ms), target_velocity_(velocity) {}

    void setTargetVelocity(float velocity) { target_velocity_ = velocity; }

    bool handle(const uint8_t* payload, size_t len, uint64_t ts) override;
};

#endif /* CONTROL_HANDLERS_HPP_ */
