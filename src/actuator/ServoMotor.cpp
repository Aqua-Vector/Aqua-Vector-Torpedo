#include "ServoMotor.hpp"
#include "MathUtils.hpp"
#include <cmath>

ServoMotor::ServoMotor(IPwmChannel& pwm, const ServoConfig& config)
	: pwm_(pwm), config_(config), current_angle_(0.0f) {}

ErrorCode ServoMotor::init() {
	ErrorCode err;
	if ((err = pwm_.init(config_.period_ns)) != ErrorCode::OK) return err;
	if ((err = pwm_.enable(true)) != ErrorCode::OK) return err;

	return setAngle(0.0f, 0.0f);
}

ErrorCode ServoMotor::setAngle(float target_degree, float dt_sec) {
	if (!utils::isValid(target_degree)) {
		return ErrorCode::ERR_INVALID_PARAMETER;
	}

	target_degree = utils::clamp(target_degree, -config_.max_angle_deg, config_.max_angle_deg);

	if (dt_sec > 0.0001f) {
		float max_step = config_.max_deg_per_sec * dt_sec;
		float diff = target_degree - current_angle_;

		if (std::abs(diff) > max_step) {
			current_angle_ += (diff > 0.0f) ? max_step : -max_step;
		} else {
			current_angle_ = target_degree;
		}
	} else {
		current_angle_ = target_degree;
	}

	uint32_t target_ns = calculatePulseWidth(current_angle_);
	return pwm_.setDutyCycle(target_ns);
}

ErrorCode ServoMotor::disable() {
	return pwm_.enable(false);
}

uint32_t ServoMotor::calculatePulseWidth(float degree) {
	float pulse_ns = utils::map(degree, -config_.max_angle_deg, config_.max_angle_deg, static_cast<float>(config_.min_pulse_ns), static_cast<float>(config_.max_pulse_ns));

	pulse_ns = utils::clamp(pulse_ns, static_cast<float>(config_.min_pulse_ns), static_cast<float>(config_.max_pulse_ns));

	return static_cast<uint32_t>(pulse_ns);
}
