#include "torpedo/sensor/MiniImuUart.hpp"
#include "utils/TimeUtils.hpp" 
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cmath>
#include <cstring>
#include <iostream>

namespace torpedo::sensor {

MiniImuUart::MiniImuUart(const std::string& dev, int baud)
	: device_path_(dev), baud_rate_(baud) {
		std::memset(&latest_sample_, 0, sizeof(ImuSample));
	}

MiniImuUart::~MiniImuUart() {
	shutdown();
}

int MiniImuUart::openSerial(const char* dev, int baud) {
	int fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0) return -1;

	termios tio{};
	if (tcgetattr(fd, &tio) < 0) {
		close(fd);
		return -1;
	}

	cfmakeraw(&tio);

	speed_t speed = B115200;
	if (baud == 9600) speed = B9600;
	else if (baud == 19200) speed = B19200;
	else if (baud == 38400) speed = B38400;
	else if (baud == 57600) speed = B57600;
	else if (baud == 115200) speed = B115200;
	else if (baud == 230400) speed = B230400;

	cfsetispeed(&tio, speed);
	cfsetospeed(&tio, speed);

	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 5;

	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
		close(fd);
		return -1;
	}
	tcflush(fd, TCIOFLUSH);
	return fd;
}

bool MiniImuUart::init() {
	fd_ = openSerial(device_path_.c_str(), baud_rate_);
	if (fd_ < 0) return false;

	// init 성공 시 자동으로 수신 스레드 시작
	if (!is_running_) {
		is_running_ = true;
		rx_thread_ = std::thread(&MiniImuUart::rxLoop, this);
	}
	return true;
}

void MiniImuUart::shutdown() {
	if (is_running_) {
		is_running_ = false;
		if (rx_thread_.joinable()) {
			rx_thread_.join();
		}
	}
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}
}

bool MiniImuUart::read(ImuSample& out_sample) {
	uint32_t seq_before = 0;
	uint32_t seq_after = 0;
	const int MAX_RETRY = 3; 

	for (int retry = 0; retry < MAX_RETRY; ++retry) {
		seq_before = seq_counter_.load(std::memory_order_acquire);

		if (seq_before % 2 != 0) {
			continue; 
		}

		out_sample = latest_sample_;

		seq_after = seq_counter_.load(std::memory_order_acquire);

		if (seq_before == seq_after) {
			return (out_sample.t_us != 0); 
		}
	}

	return false;
}

bool MiniImuUart::readByte(uint8_t& out, int timeout_ms) {
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(fd_, &rfds);
	timeval tv{};
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
	if (ret <= 0) return false;
	return ::read(fd_, &out, 1) == 1;
}

bool MiniImuUart::readFrame(uint8_t frame[11]) {
	uint8_t b = 0;
	while (is_running_) {
		if (!readByte(b, 1000)) return false;
		if (b == 0x55) break;
	}
	frame[0] = b;
	for (int i = 1; i < 11; ++i) {
		if (!readByte(frame[i], 200)) return false;
	}
	return true;
}

bool MiniImuUart::checksumOk(const uint8_t frame[11]) {
	uint8_t sum = 0;
	for (int i = 0; i < 10; ++i) sum = static_cast<uint8_t>(sum + frame[i]);
	return sum == frame[10];
}

void MiniImuUart::rxLoop() {
	uint8_t frame[11] = {};

	float t_ax = 0.0f, t_ay = 0.0f, t_az = 0.0f;
	float t_gx = 0.0f, t_gy = 0.0f, t_gz = 0.0f;
	bool accel_ready = false;
	bool gyro_ready = false;

	while (is_running_) {
		if (!readFrame(frame)) continue;

		if (!checksumOk(frame)) {
			bad_checksum_count_++;
			continue;
		}

		int16_t x = static_cast<int16_t>(frame[2] | (frame[3] << 8));
		int16_t y = static_cast<int16_t>(frame[4] | (frame[5] << 8));
		int16_t z = static_cast<int16_t>(frame[6] | (frame[7] << 8));

		if (frame[1] == 0x51) {
			t_ax = static_cast<float>(x) / 32768.0f * 16.0f * 9.80665f;
			t_ay = static_cast<float>(y) / 32768.0f * 16.0f * 9.80665f;
			t_az = static_cast<float>(z) / 32768.0f * 16.0f * 9.80665f;
			accel_ready = true;
		} 
		else if (frame[1] == 0x52) {
			t_gx = static_cast<float>(x) / 32768.0f * 2000.0f * (M_PI / 180.0f);
			t_gy = static_cast<float>(y) / 32768.0f * 2000.0f * (M_PI / 180.0f);
			t_gz = static_cast<float>(z) / 32768.0f * 2000.0f * (M_PI / 180.0f);
			gyro_ready = true;
		}

		if (accel_ready && gyro_ready) {
			uint32_t current_seq = seq_counter_.load(std::memory_order_relaxed);
			seq_counter_.store(current_seq + 1, std::memory_order_release);

			latest_sample_.t_us = utils::getCurrentTimeUs(); 
			latest_sample_.ax = t_ax;
			latest_sample_.ay = t_ay;
			latest_sample_.az = t_az;
			latest_sample_.gx = t_gx;
			latest_sample_.gy = t_gy;
			latest_sample_.gz = t_gz;
			latest_sample_.valid = true;

			seq_counter_.store(current_seq + 2, std::memory_order_release);

			accel_ready = false;
			gyro_ready = false;
		}
	}
}

} // namespace torpedo::sensor
