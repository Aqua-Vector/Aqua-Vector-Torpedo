#include "LinuxPwmChannel.hpp"
#include <iostream>
#include <fstream>
#include <unistd.h>

LinuxPwmChannel::LinuxPwmChannel(uint32_t pwm_index, const std::string& sysfs_path) : pwm_index_str_(std::to_string(pwm_index)) {
	std::string normalized_path = sysfs_path;
	if (normalized_path.empty() || normalized_path.back() != '/') {
		normalized_path += "/";
	}

	base_path_ = normalized_path + "pwm" + pwm_index_str_ + "/";
	export_path_ = normalized_path + "export";
	unexport_path_ = normalized_path + "unexport";
}

LinuxPwmChannel::~LinuxPwmChannel() {
	enable(false);
	writeToFile(unexport_path_, pwm_index_str_);
}

ErrorCode LinuxPwmChannel::init(uint32_t period_ns) {
	writeToFile(unexport_path_, pwm_index_str_);
	usleep(10000);

	if (!writeToFile(export_path_, pwm_index_str_)) {
		std::cerr << "[PWM Error] Failed to export PWM" << pwm_index_str_ << std::endl;
		return ErrorCode::ERR_HW_INIT_FAILED;
	}
	usleep(50000);

	if (!writeToFile(base_path_ + "period", std::to_string(period_ns))) {
		std::cerr << "[PWM Error] Failed to write period: " << period_ns << " ns" << std::endl;
		return ErrorCode::ERR_HW_INIT_FAILED;
	}

	return ErrorCode::OK;
}

ErrorCode LinuxPwmChannel::setDutyCycle(uint32_t duty_ns) {
	if (!writeToFile(base_path_ + "duty_cycle", std::to_string(duty_ns))) {
		std::cerr << "[PWM Error] Failed to write duty_cycle: " << duty_ns << " ns" << std::endl;
		return ErrorCode::ERR_IO_WRITE_FAILED;
	}

	return ErrorCode::OK;
}

ErrorCode LinuxPwmChannel::enable(bool state) {
	std::string val = state ? "1" : "0";
	if (!writeToFile(base_path_ + "enable", val)) {
	std::cerr << "[PWM Error] Failed to set enable state to " << val << std::endl;
	return ErrorCode::ERR_IO_WRITE_FAILED;
	}

	return ErrorCode::OK;
}

bool LinuxPwmChannel::writeToFile(const std::string& filepath, const std::string& value) {
	std::ofstream file(filepath);
	if (!file.is_open()) {
		return false;
	}
	file << value << std::endl;
	file.close();
	return true;
}

