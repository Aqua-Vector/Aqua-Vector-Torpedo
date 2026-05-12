#ifndef I_CONTROL_DATA_SOURCE_HPP_
#define I_CONTROL_DATA_SOURCE_HPP_

#include "ControlTypes.hpp"

class IControlDataSource {
public:
	virtual ~IControlDataSource() = default;

	virtual bool fetchLatestState(ControlState& out_state, uint64_t& out_timestamp) = 0;
};

#endif /* I_CONTROL_DATA_SOURCE_HPP_ */
