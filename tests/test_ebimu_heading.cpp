#include "guidance/EbImuUart.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>

using namespace torpedo;
using namespace torpedo::sensor;

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    if (argc > 1) port = argv[1];

    // EBIMU24GV52는 사용자가 921600으로 설정한 것으로 보임
    EbImuUart imu(port, 921600);

    std::cout << "Initializing EBIMU on " << port << "..." << std::endl;
    if (!imu.init()) {
        std::cerr << "Failed to initialize IMU" << std::endl;
        return -1;
    }

    std::cout << "Reading Heading (Yaw) from EBIMU. Press Ctrl+C to stop." << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(12) << "Time[us]" 
              << std::setw(10) << "Roll[deg]" 
              << std::setw(10) << "Pitch[deg]" 
              << std::setw(10) << "Yaw[deg]" 
              << std::setw(10) << "Gz[dps]" 
              << std::endl;

    ImuSample sample;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        if (imu.read(sample)) {
            float roll_deg = sample.roll * (180.0f / M_PI);
            float pitch_deg = sample.pitch * (180.0f / M_PI);
            float yaw_deg = sample.yaw * (180.0f / M_PI);
            float gz_dps = sample.gz * (180.0f / M_PI);

            std::cout << "\r" 
                      << std::setw(12) << sample.t_us
                      << std::setw(10) << roll_deg
                      << std::setw(10) << pitch_deg
                      << std::setw(10) << yaw_deg
                      << std::setw(10) << gz_dps
                      << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() > 30) {
            // 30초 후 자동 종료 (테스트용)
            // break; 
        }
    }

    imu.shutdown();
    return 0;
}
