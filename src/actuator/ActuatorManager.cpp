#include "ActuatorManager.hpp"

ActuatorManager::ActuatorManager(ServoMotor& rudder, ServoMotor& elevator)
	: rudder_servo_(rudder), elevator_servo_(elevator) {}

ErrorCode ActuatorManager::initAll() {
	ErrorCode err;

	if ((err = rudder_servo_.init()) != ErrorCode::OK) {
		return err;
	}

	if ((err = elevator_servo_.init()) != ErrorCode::OK) {
		return err;
	}

	return ErrorCode::OK;
}

ErrorCode ActuatorManager::applyControl(const ControlState& target_state, float dt_sec) {
	ErrorCode err;

	if ((err = rudder_servo_.setAngle(target_state.rudder, dt_sec)) != ErrorCode::OK) {
		return err;
	}

	if ((err = elevator_servo_.setAngle(target_state.elevator, dt_sec)) != ErrorCode::OK) {
		return err;
	}

	return ErrorCode::OK;
}

void ActuatorManager::triggerFailSafe() {
	rudder_servo_.disable();
	elevator_servo_.disable();
}
