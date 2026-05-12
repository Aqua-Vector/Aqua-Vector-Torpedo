#ifndef MANUAL_SOURCE_HPP_
#define MANUAL_SOURCE_HPP_

#include "IControlDataSource.hpp"
#include "Mailbox.hpp"
#include "Payloads.hpp"

class ManualSource : public IControlDataSource {
private:
	Mailbox<ControlState> mailbox_;

public:
	ManualSource() = default;
	virtual ~ManualSource() = default;

	bool fetchLatestState(ControlState& out_state, uint64_t& out_timestamp) override;

	void onControlPacketReceived(const ControlPayload& payload, uint64_t timestamp);
};

#endif /* MANUAL_SOURCE_HPP_ */
