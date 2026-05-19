#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

#include "actuator/ActuatorManager.hpp"
#include "actuator/ServoMotor.hpp"
#include "hal/LinuxPwmChannel.hpp"
#include "common/ControlTypes.hpp"
#include "common/ActuatorTypes.hpp"

/**
 * @brief Mock environment for Linux PWM Sysfs
 * Creates temporary directories and files to simulate /sys/class/pwm/
 */
class PwmMockEnv {
public:
    std::string root_path;
    std::string pwm0_path;
    std::string pwm1_path;

    PwmMockEnv() {
        root_path = "/tmp/gemini_pwm_test/";
        std::string chip0_path = root_path + "pwmchip0/";
        std::string chip1_path = root_path + "pwmchip1/";
        pwm0_path = chip0_path + "pwm0/";
        pwm1_path = chip1_path + "pwm0/";

        // Create directories
        mkdir(root_path.c_str(), 0777);
        mkdir(chip0_path.c_str(), 0777);
        mkdir(chip1_path.c_str(), 0777);
        mkdir(pwm0_path.c_str(), 0777);
        mkdir(pwm1_path.c_str(), 0777);

        // Create dummy files
        createFile(chip0_path + "export", "");
        createFile(chip0_path + "unexport", "");
        createFile(chip1_path + "export", "");
        createFile(chip1_path + "unexport", "");
        
        createFile(pwm0_path + "period", "0");
        createFile(pwm0_path + "duty_cycle", "0");
        createFile(pwm0_path + "enable", "0");

        createFile(pwm1_path + "period", "0");
        createFile(pwm1_path + "duty_cycle", "0");
        createFile(pwm1_path + "enable", "0");
    }

    ~PwmMockEnv() {
        // Cleanup would go here, but keep for inspection if needed
    }

    void createFile(const std::string& path, const std::string& content) {
        std::ofstream ofs(path);
        ofs << content;
        ofs.close();
    }

    std::string readFile(const std::string& path) {
        std::ifstream ifs(path);
        std::string content;
        ifs >> content;
        return content;
    }
};

void test_slew_rate_limiting() {
    std::cout << "[Test] Slew Rate Limiting Check..." << std::endl;
    PwmMockEnv env;
    
    LinuxPwmChannel pwm0(0, 0, env.root_path);
    ServoConfig config;
    config.period_ns = 20000000;    // 50Hz
    config.min_pulse_ns = 1000000;  // 1ms (-90 deg)
    config.max_pulse_ns = 2000000;  // 2ms (+90 deg)
    config.max_angle_deg = 90.0f;
    config.max_deg_per_sec = 180.0f; // 1 second for full range

    ServoMotor servo(pwm0, config);
    servo.init();

    // 1. Move to 0.5 (45 degree) with dt=0.1s
    // 180 deg/sec * 0.1s = 18.0 deg max step
    servo.setAngle(0.5f, 0.1f);
    
    // We expect current_angle to be 18.0f since target was 45.0f
    // Duty cycle at 18 deg:
    // map(18, -90, 90, 1000000, 2000000)
    // ratio = (18 - (-90)) / (90 - (-90)) = 108 / 180 = 0.6
    // pulse = 1000000 + 0.6 * 1000000 = 1,600,000 ns
    
    std::string duty = env.readFile(env.pwm0_path + "duty_cycle");
    assert(duty == "1600000");
    
    std::cout << "  - Slew rate step 1 OK (18 deg)" << std::endl;

    // 2. Large dt move
    servo.setAngle(1.0f, 1.0f); // Should reach 90 deg immediately
    duty = env.readFile(env.pwm0_path + "duty_cycle");
    assert(duty == "2000000");
    std::cout << "  - Reach target with large dt OK" << std::endl;
}

void test_failsafe_protection() {
    std::cout << "[Test] Failsafe & Invalid Input Check..." << std::endl;
    PwmMockEnv env;
    
    LinuxPwmChannel pwm0(0, 0, env.root_path);
    LinuxPwmChannel pwm1(1, 0, env.root_path);
    
    ServoConfig config;
    config.period_ns = 20000000;
    config.min_pulse_ns = 1000000;
    config.max_pulse_ns = 2000000;
    config.max_angle_deg = 90.0f;
    config.max_deg_per_sec = 360.0f;

    ServoMotor rudder(pwm0, config);
    ServoMotor elevator(pwm1, config);
    ActuatorManager manager(rudder, elevator);

    manager.initAll();
    
    // Test NaN protection
    ControlState bad_state;
    bad_state.rudder = std::nanf("");
    bad_state.elevator = 0.0f;
    
    ErrorCode err = manager.applyControl(bad_state, 0.01f);
    assert(err == ErrorCode::ERR_INVALID_PARAMETER);
    std::cout << "  - NaN protection OK" << std::endl;

    // Test Failsafe (Disable)
    manager.triggerFailSafe();
    assert(env.readFile(env.pwm0_path + "enable") == "0");
    assert(env.readFile(env.pwm1_path + "enable") == "0");
    std::cout << "  - Failsafe trigger (PWM Disable) OK" << std::endl;
}

int main() {
    try {
        test_slew_rate_limiting();
        test_failsafe_protection();
        std::cout << "\nALL ACTUATOR TESTS PASSED!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "\nTEST FAILED: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
