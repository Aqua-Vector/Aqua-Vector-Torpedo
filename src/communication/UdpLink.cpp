#include "UdpLink.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

UdpLink::UdpLink(const std::string& remote_ip, int port)
    : sockfd_(-1), remote_ip_(remote_ip), port_(port) {
    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
}

UdpLink::~UdpLink() {
    close();
}

bool UdpLink::initialize() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) return false;

    // 수신 타임아웃 설정 (100ms)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(port_);
    remote_addr_.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

    return true;
}

void UdpLink::close() {
    if (sockfd_ != -1) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

bool UdpLink::isConnected() const {
    return sockfd_ != -1;
}

ssize_t UdpLink::send(const uint8_t* data, size_t length) {
    if (sockfd_ == -1) return -1;
    return sendto(sockfd_, data, length, 0, 
                  (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
}

ssize_t UdpLink::receive(uint8_t* buffer, size_t max_length) {
    if (sockfd_ == -1) return -1;
    return recvfrom(sockfd_, buffer, max_length, 0, nullptr, nullptr);
}
