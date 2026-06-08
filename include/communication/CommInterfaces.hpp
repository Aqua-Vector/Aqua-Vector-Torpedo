#ifndef COMM_INTERFACES_HPP_
#define COMM_INTERFACES_HPP_

#include <cstdint>
#include <cstddef>
#include <sys/types.h>

// 통신 링크의 생명주기 관리 인터페이스
class ICommLifeCycle {
public:
    virtual ~ICommLifeCycle() = default;
    virtual bool initialize() = 0;
    virtual void close() = 0;
    virtual bool isConnected() const = 0;
};

// 데이터 송수신 스트림 인터페이스
class ICommDataStream {
public:
    virtual ~ICommDataStream() = default;
    virtual ssize_t send(const uint8_t* data, size_t length) = 0;
    virtual ssize_t receive(uint8_t* buffer, size_t max_length) = 0;
};

// 특정 메시지 ID에 대한 처리 핸들러 인터페이스
class IMessageHandler {
public:
    virtual ~IMessageHandler() = default;
    virtual bool handle(const uint8_t* payload, size_t payload_length, uint64_t timestamp_ms) = 0;
};

// 바이트 단위 파싱 인터페이스
class IPacketParser {
public:
    virtual ~IPacketParser() = default;
    virtual void parseByte(uint8_t byte, uint64_t timestamp_ms) = 0;
};

#endif /* COMM_INTERFACES_HPP_ */
