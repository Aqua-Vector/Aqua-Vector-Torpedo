#ifndef MATH_UTILS_HPP_
#define MATH_UTILS_HPP_

#include <cmath>

namespace utils {

	template <typename T>
	constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
		return (v < lo) ? lo : (hi < v) ? hi : v;
	}

	template <typename T>
	bool isValid(const T& value) {
		return !std::isnan(value) && !std::isinf(value);
	}

	template <typename T>
	constexpr T map(T x, T in_min, T in_max, T out_min, T out_max) {
		if (std::abs(in_max - in_min) < 1e-6) return out_min;
		return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
	}

} // namespace utils

#endif /* MATH_UTILS_HPP_ */
