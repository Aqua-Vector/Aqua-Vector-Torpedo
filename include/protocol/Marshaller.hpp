#ifndef MARSHALLER_HPP_
#define MARSHALLER_HPP_

#include "Payloads.hpp"
#include <cstdint>
#include <cstddef>

namespace Marshaller {
	
	// Torpedo 데이터 처리
	bool serialize(const TorpedoTelemetryPayload& data, uint8_t* buffer, size_t size);
	bool deserialize(const uint8_t* buffer, size_t size, TorpedoTelemetryPayload& data);

	// Contorl 데이터 처리
	bool serialize(const ControlPayload& data, uint8_t* buffer, size_t size);
	bool deserialize(const uint8_t* buffer, size_t size, ControlPayload& data);

	// Feedback 데이터 처리
	bool serialize(const FeedbackPayload& data, uint8_t* buffer, size_t size);
	bool deserialize(const uint8_t* buffer, size_t size, FeedbackPayload& data);

	// ControlStation 데이터 처리
	bool serialize(const ControlStationPayload& data, uint8_t* buffer, size_t size);
	bool deserialize(const uint8_t* buffer, size_t size, ControlStationPayload& data);

}

#endif /* MARSHALLER_HPP_ */
