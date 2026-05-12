#ifndef CONTROL_DATA_VALIDATOR_HPP_
#define CONTROL_DATA_VALIDATOR_HPP_

#include "ControlTypes.hpp"

class ControlDataValidator {
public:
    static constexpr float MAX_VELOCITY = 100.0f;
    static constexpr float MIN_VELOCITY = -100.0f;
    
    static constexpr float MAX_RUDDER = 45.0f;
    static constexpr float MIN_RUDDER = -45.0f;
    
    static constexpr float MAX_ELEVATOR = 45.0f;
    static constexpr float MIN_ELEVATOR = -45.0f;

    static void sanitize(ControlState& state); 
};

#endif /* CONTROL_DATA_VALIDATOR_HPP_ */
