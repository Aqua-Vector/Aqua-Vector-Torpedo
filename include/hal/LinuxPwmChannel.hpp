#ifndef LINUX_PWM_CHANNEL_HPP_
#define LINUX_PWM_CHANNEL_HPP_

#include "IPwmChannel.hpp"
#include <string>

class LinuxPwmChannel : public IPwmChannel {
private:
	std::string base_path_;
	std::string export_path_;
	std::string unexport_path_;
	std::string pwm_index_str_;

	bool writeToFile(const std::string& filepath, const std::string& value);

public:
	LinuxPwmChannel(uint32_t pwm_index, const std::string& sysfs_path = "/sys/class/pwm/pwmchip0/");
	virtual ~LinuxPwmChannel();

	ErrorCode init(uint32_t period_ns) override;
	ErrorCode setDutyCycle(uint32_t duty_ns) override;
	ErrorCode enable(bool state) override;
};

#endif /* LINUX_PWM_CHANNEL_HPP_ */
