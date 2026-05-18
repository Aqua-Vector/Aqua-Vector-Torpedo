#ifndef SERVO_MOTOR_HPP_
#define SERVO_MOTOR_HPP_

#include "IPwmChannel.hpp"
#include "ActuatorTypes.hpp"
#include "ErrorCodes.hpp"

class ServoMotor {
private:
	IPwmChannel& pwm_;
	const ServoConfig config_;
	float current_angle_;

	uint32_t calculatePulseWidth(float degree);

public:
	ServoMotor(IPwmChannel& pwm, const ServoConfig& config);

	ErrorCode init();
	ErrorCode setAngle(float normalized_cmd, float dt_sec);
	ErrorCode disable();
};

#endif /* SERVO_MOTOR_HPP_ */
