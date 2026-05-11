#include "UartLink.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

UartLink::UartLink(const std::string& device_path, int baud_rate)
	: fd_(-1), device_path_(device_path), baud_rate_(baud_rate) {}

	UartLink::~UartLink() {
		close();
	}

bool UartLink::initialize() {

	// fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
	if (fd_ == -1) return false;

	struct termios options;
	tcgetattr(fd_, &options);

	int baud = get_baud_constant(baud_rate_);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 1;

	tcsetattr(fd_, TCSANOW, &options);
	return true;
}

void UartLink::close() {
	if (fd_ != -1) {
		::close(fd_);
		fd_ = -1;
	}
}

bool UartLink::isConnected() const {
	return fd_ != -1;
}

size_t UartLink::send(const uint8_t* data, size_t length) {
	if (fd_ == -1) return 0;
	ssize_t written = write(fd_, data, length);
	return (written > 0) ? static_cast<size_t>(written) : 0;
}

size_t UartLink::receive(uint8_t* buffer, size_t max_length) {
	if (fd_ == -1) return 0;
	ssize_t bytes_read = read(fd_, buffer, max_length);
	return (bytes_read > 0) ? static_cast<size_t>(bytes_read) : 0;
}

int UartLink::get_baud_constant(int baudrate) {
	switch (baudrate) {
		case 9600:   return B9600;
		case 19200:  return B19200;
		case 38400:  return B38400;
		case 57600:  return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		default:     return B115200;
	}
}
