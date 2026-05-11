#ifndef UART_LINK_HPP_
#define UART_LINK_HPP_

#include "CommInterfaces.hpp"
#include <string>

class UartLink : public ICommLifeCycle, public ICommDataStream {
private:
    int fd_;
    std::string device_path_;
    int baud_rate_;

    int get_baud_constant(int baudrate);

public:
    UartLink(const std::string& device_path, int baud_rate);
    ~UartLink() override;

    // ICommLifeCycle 구현
    bool initialize() override;
    void close() override;
    bool isConnected() const override;

    // ICommDataStream 구현
    size_t send(const uint8_t* data, size_t length) override;
    size_t receive(uint8_t* buffer, size_t max_length) override;
};

#endif /* UART_LINK_HPP_ */
