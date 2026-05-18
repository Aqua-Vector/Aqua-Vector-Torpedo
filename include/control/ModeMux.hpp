#ifndef MODE_MUX_HPP_
#define MODE_MUX_HPP_

#include <cstdint>
#include <memory>
#include "ControlTypes.hpp"
#include "IControlDataSource.hpp"

class ModeMux {
private:
	SystemMode current_mode_;
	ControlState last_valid_state_;
	uint64_t last_update_time_ms_;

	std::shared_ptr<IControlDataSource> manual_source_;
	std::shared_ptr<IControlDataSource> auto_source_;

	static constexpr uint64_t WATCHDOG_TIMEOUT_MS = 500;

	void applyFailsafeState();

public:
	ModeMux(std::shared_ptr<IControlDataSource> manual_source, std::shared_ptr<IControlDataSource> auto_source);
	~ModeMux() = default;

	void setMode(SystemMode new_mode);
	SystemMode getMode() const;

	ControlState processControlLoop(uint64_t current_time_ms);
};

#endif /* MODE_MUX_HPP_ */
