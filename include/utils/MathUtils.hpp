#ifndef MATH_UTILS_HPP_
#define MATH_UTILS_HPP_

namespace utils {

	template <typename T>
	constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
		return (v < lo) ? lo : (hi < v) ? hi : v;
	}

	template <typename T>
	bool isValid(const T& value) {
		return !std::isnan(value) && !std::isinf(value);
	}

} // namespace utils

#endif /* MATH_UTILS_HPP_ */
