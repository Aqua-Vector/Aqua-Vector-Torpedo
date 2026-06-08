#ifndef CONTROL_TYPES_HPP_
#define CONTROL_TYPES_HPP_

#include <cstdint>

// 파싱 상태 머신을 위한 공용 상태 정의
enum class ParseState {
    WAIT_SYNC1,
    WAIT_SYNC2,
    READ_MSG_ID,
    READ_LENGTH,
    READ_PAYLOAD,
    READ_CRC
};

// 시스템 제어 상태 데이터 모델
struct ControlState {
    float velocity;
    float rudder;
    float elevator;
    uint64_t last_update_time_ms;
};

// 시스템 제어 모드
enum class SystemMode : uint8_t {
    STANDBY = 0x00,
    MANUAL = 0x01,
    AUTO = 0x02,
    FAILSAFE = 0x03,
    LOCKDOWN = 0x04
};

#endif /* CONTROL_TYPES_HPP_ */
