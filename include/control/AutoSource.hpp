#ifndef AUTO_SOURCE_HPP_
#define AUTO_SOURCE_HPP_

#include "IControlDataSource.hpp"
#include "Mailbox.hpp"

class AutoSource : public IControlDataSource {
private:
	Mailbox<ControlState> mailbox_;

public:
	AutoSource() = default;
	virtual ~AutoSource() = default;

	bool fetchLatestState(ControlState& out_state, uint64_t& out_timestamp) override;

	void updateTargetState(const ControlState& new_state, uint64_t timestamp);
};

#endif /* AUTO_SOURCE_HPP_ */
