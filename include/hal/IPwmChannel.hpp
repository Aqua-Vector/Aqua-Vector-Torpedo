#ifndef I_PWM_CHANNEL_HPP_
#define I_PWM_CHANNEL_HPP_

#include "ErrorCodes.hpp"
#include <cstdint>

class IPwmChannel {
public:
	virtual ~IPwmChannel() = default;

	virtual ErrorCode init(uint32_t period_ns) = 0;
	virtual ErrorCode setDutyCycle(uint32_t duty_ns) = 0;
	virtual ErrorCode enable(bool state) = 0;
};

#endif /* I_PWM_CHANNEL_HPP_ */
