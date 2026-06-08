#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "torpedo/sensor/MiniImuUart.hpp"
#include "utils/LowPassFilter.hpp"

int main(int argc, char** argv) {
    std::string port = "/dev/ttyS3";
    if (argc > 1) port = argv[1];

    std::cout << "========== MiniIMU Heading Logger ==========" << std::endl;
    std::cout << "Port: " << port << " | Interval: 1.0s" << std::endl;
    std::cout << "Applying LPF (alpha=0.3)" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    torpedo::sensor::MiniImuUart imu(port, 115200);
    utils::LowPassFilter yaw_lpf(0.3f);

    if (!imu.init()) {
        std::cerr << "Failed to initialize IMU on " << port << std::endl;
        return -1;
    }

    auto last_log_time = std::chrono::steady_clock::now();
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Timestamp [s] |  Raw Yaw [deg] | Filtered Yaw [deg]" << std::endl;

    while (true) {
        torpedo::ImuSample sample;
        if (imu.read(sample)) {
            float raw_yaw_deg = sample.yaw * (180.0f / M_PI);
            float filtered_yaw_rad = yaw_lpf.updateAngle(sample.yaw);
            float filtered_yaw_deg = filtered_yaw_rad * (180.0f / M_PI);

            auto now = std::chrono::steady_clock::now();
            if (now - last_log_time >= std::chrono::seconds(1)) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() % 10000;
                
                std::cout << std::setw(12) << elapsed << " | "
                          << std::setw(13) << raw_yaw_deg << " | "
                          << std::setw(18) << filtered_yaw_deg << std::endl;
                
                last_log_time = now;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    imu.shutdown();
    return 0;
}
