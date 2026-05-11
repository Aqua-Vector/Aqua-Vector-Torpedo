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

#endif /* CONTROL_TYPES_HPP_ */
