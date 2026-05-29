#ifndef CONTROL_HANDLERS_HPP_
#define CONTROL_HANDLERS_HPP_

#include "CommInterfaces.hpp"
#include "ControlTypes.hpp"
#include "Payloads.hpp"
#include "Marshaller.hpp"

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

#endif /* CONTROL_HANDLERS_HPP_ */
