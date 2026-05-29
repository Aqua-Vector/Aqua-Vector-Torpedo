#ifndef ACTUATOR_TYPES_HPP_
#define ACTUATOR_TYPES_HPP_

#include <cstdint>

struct ServoConfig {
	uint32_t period_ns;
	uint32_t min_pulse_ns;
	uint32_t max_pulse_ns;
	float max_angle_deg;
	float max_deg_per_sec;
};

#endif /* ACTUATOR_TYPES_HPP_ */
