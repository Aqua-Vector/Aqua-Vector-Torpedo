#ifndef ACTUATOR_MANAGER_HPP_
#define ACTUATOR_MANAGER_HPP_

#include "ControlTypes.hpp"
#include "ErrorCodes.hpp"
#include "ServoMotor.hpp"

class ActuatorManager {
private:
	ServoMotor& rudder_servo_;
	ServoMotor& elevator_servo_;

public:
	ActuatorManager(ServoMotor& rudder, ServoMotor& elevator);

	ErrorCode initAll();
	ErrorCode applyControl(const ControlState& target_state, float dt_sec);
	void triggerFailSafe();
};

#endif /* ACTUATOR_MANAGER_HPP_ */
