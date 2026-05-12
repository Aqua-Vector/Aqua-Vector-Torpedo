#ifndef UDP_LINK_HPP_
#define UDP_LINK_HPP_

#include "CommInterfaces.hpp"
#include <string>
#include <netinet/in.h>

class UdpLink : public ICommLifeCycle, public ICommDataStream {
private:
    int sockfd_;
    std::string remote_ip_;
    int port_;
    struct sockaddr_in remote_addr_;

public:
    UdpLink(const std::string& remote_ip, int port);
    ~UdpLink() override;

    bool initialize() override;
    void close() override;
    bool isConnected() const override;

    size_t send(const uint8_t* data, size_t length) override;
    size_t receive(uint8_t* buffer, size_t max_length) override;
};

#endif /* UDP_LINK_HPP_ */
