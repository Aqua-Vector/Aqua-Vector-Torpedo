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
	fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_ == -1) return false;

	struct termios options;
	std::memset(&options, 0, sizeof(options));
	if (tcgetattr(fd_, &options) != 0) return false;

	int baud = get_baud_constant(baud_rate_);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	// 8N1, Raw Mode, No flow control
	options.c_cflag |= (CLOCAL | CREAD | CS8);
	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	
	// Disable software flow control and translations
	options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	
	// Raw output
	options.c_oflag &= ~OPOST;
	
	// Raw input (non-canonical)
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Low latency: Return as soon as 1 byte is available
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;

	tcflush(fd_, TCIOFLUSH);
	if (tcsetattr(fd_, TCSANOW, &options) != 0) return false;

	// After setting termios, clear O_NONBLOCK to allow VMIN/VTIME to work as intended
	int flags = fcntl(fd_, F_GETFL, 0);
	fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

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
