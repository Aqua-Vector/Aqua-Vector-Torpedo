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

/*
bool UartLink::initialize() {
	fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_ == -1) return false;

	struct termios options;
	// Use memset to clear all flags, avoiding inheritance of OS defaults via tcgetattr
	std::memset(&options, 0, sizeof(options));

	int baud = get_baud_constant(baud_rate_);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	// Verified "Clean" configuration for Raw Binary Mode
	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	options.c_cflag |= (CS8 | CLOCAL | CREAD); 
	options.c_iflag = IGNPAR;
	options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 0;

	tcflush(fd_, TCIFLUSH);
	if (tcsetattr(fd_, TCSANOW, &options) != 0) {
		::close(fd_);
		fd_ = -1;
		return false;
	}

	return true;
}
*/
bool UartLink::initialize() {
	fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_ == -1) return false;

	struct termios options;
	// Use memset to clear all flags, avoiding inheritance of OS defaults via tcgetattr
	std::memset(&options, 0, sizeof(options));
	if(tcgetattr(fd_, &options) !=  0) return false;

	int baud = get_baud_constant(baud_rate_);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);

	// Verified "Clean" configuration for Raw Binary Mode
	options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
	options.c_cflag |= (CS8 | CLOCAL | CREAD); 
	//options.c_iflag = IGNPAR;
	options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 0;

	tcflush(fd_, TCIOFLUSH);
	if (tcsetattr(fd_, TCSANOW, &options) != 0) {
		::close(fd_);
		fd_ = -1;
		return false;
	}

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

ssize_t UartLink::send(const uint8_t* data, size_t length) {
	if (fd_ == -1) return -1;
	return write(fd_, data, length);
}

ssize_t UartLink::receive(uint8_t* buffer, size_t max_length) {
	if (fd_ == -1) return -1;
	return read(fd_, buffer, max_length);
}

int UartLink::get_baud_constant(int baudrate) {
	switch (baudrate) {
		case 9600:   return B9600;
		case 19200:  return B19200;
		case 38400:  return B38400;
		case 57600:  return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 921600: return B921600;
		default:     return B115200;
	}
}

