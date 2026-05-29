#ifndef TIME_UTILS_HPP_
#define TIME_UTILS_HPP_

#include <chrono>
#include <cstdint>

namespace utils {

	inline uint64_t getCurrentTimeMs() {
		auto now = std::chrono::steady_clock::now();
		auto duration = now.time_since_epoch();
		return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
	}

	inline uint64_t getCurrentTimeUs() {
		auto now = std::chrono::steady_clock::now();
		auto duration = now.time_since_epoch();
		return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
	}

} // namespace utils

#endif /* TIME_UTILS_HPP_ */
