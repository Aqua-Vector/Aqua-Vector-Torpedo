#include "ControlDataValidator.hpp"
#include "MathUtils.hpp"

void ControlDataValidator::sanitize(ControlState& state) {
    // NaN 또는 Inf 처리
    if (!utils::isValid(state.velocity)) state.velocity = 0.0f;
    if (!utils::isValid(state.rudder)) state.rudder = 0.0f;
    if (!utils::isValid(state.elevator)) state.elevator = 0.0f;

    // 범위 제한 (Clamping)
    state.velocity = utils::clamp(state.velocity, MIN_VELOCITY, MAX_VELOCITY);
    state.rudder = utils::clamp(state.rudder, MIN_RUDDER, MAX_RUDDER);
    state.elevator = utils::clamp(state.elevator, MIN_ELEVATOR, MAX_ELEVATOR);
}
