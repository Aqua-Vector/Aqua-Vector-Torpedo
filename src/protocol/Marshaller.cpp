#include "Marshaller.hpp"
#include "BufferView.hpp"

namespace Marshaller {

	// TorpedoTelemetryPayload
	bool serialize(const TorpedoTelemetryPayload& data, uint8_t* buffer, size_t size) {
		BufferView view(buffer, size);
		bool ok = true;

		ok &= view.writeFloat(data.speed);
		ok &= view.writeFloat(data.heading);
		ok &= view.writeFloat(data.acc_x);
		ok &= view.writeFloat(data.acc_y);
		ok &= view.write<int16_t>(data.pos_x);
		ok &= view.write<int16_t>(data.pos_y);

		return ok;
	}

	bool deserialize(const uint8_t* buffer, size_t size, TorpedoTelemetryPayload& data) {
		BufferView view(const_cast<uint8_t*>(buffer), size);
		bool ok = true;

		ok &= view.readFloat(data.speed);
		ok &= view.readFloat(data.heading);
		ok &= view.readFloat(data.acc_x);
		ok &= view.readFloat(data.acc_y);
		ok &= view.read<int16_t>(data.pos_x);
		ok &= view.read<int16_t>(data.pos_y);

		return ok;
	}

	// ControlPayload
	bool serialize(const ControlPayload& data, uint8_t* buffer, size_t size) {
		BufferView view(buffer, size);
		bool ok = true;

		ok &= view.writeFloat(data.velocity);
		ok &= view.writeFloat(data.rudder);
		ok &= view.writeFloat(data.elevator);

		return ok;
	}

	bool deserialize(const uint8_t* buffer, size_t size, ControlPayload& data) {
		BufferView view(const_cast<uint8_t*>(buffer), size);
		bool ok = true;

		ok &= view.readFloat(data.velocity);
		ok &= view.readFloat(data.rudder);
		ok &= view.readFloat(data.elevator);

		return ok;
	}

	// FeedbackPayload
	bool serialize(const FeedbackPayload& data, uint8_t* buffer, size_t size) {
		BufferView view(buffer, size);
		bool ok = true;

		ok &= view.writeFloat(data.m1_rps);
		ok &= view.writeFloat(data.m2_rps);
		ok &= view.write<uint8_t>(data.status);

		return ok;
	}

	bool deserialize(const uint8_t* buffer, size_t size, FeedbackPayload& data) {
		BufferView view(const_cast<uint8_t*>(buffer), size);
		bool ok = true;

		ok &= view.readFloat(data.m1_rps);
		ok &= view.readFloat(data.m2_rps);
		ok &= view.read<uint8_t>(data.status);

		return ok;
	}

}
