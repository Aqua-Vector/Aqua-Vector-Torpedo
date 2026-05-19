#ifndef PAYLOADS_HPP_
#define PAYLOADS_HPP_

#include <cstdint>

#pragma pack(push, 1)

struct NoCrc {};

struct ControlPayload {
	float velocity;
	float rudder;
	float elevator;
};

struct FeedbackPayload {
	float m1_rps;
	float m2_rps;
	uint16_t servo_pos;
	uint8_t status;
};

struct TorpedoTelemetryPayload {
    float   speed;
    float   heading;
    float   acc_x;
    float   acc_y;
    int16_t pos_x;
    int16_t pos_y;
};

#pragma pack(pop)

#endif /* PAYLOADS_HPP_ */
