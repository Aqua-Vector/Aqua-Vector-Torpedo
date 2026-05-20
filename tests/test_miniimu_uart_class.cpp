#include <iostream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <chrono>
#include "torpedo/sensor/MiniImuUart.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
    std::cout << "\n[Test] Signal (" << signum << ") received. Stopping..." << std::endl;
    g_running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    std::string port = "/dev/ttyS3";
    int baud = 115200;

    if (argc > 1) port = argv[1];
    if (argc > 2) baud = std::stoi(argv[2]);

    std::cout << "================================================" << std::endl;
    std::cout << "      MiniImuUart Class Hardware Test           " << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "Port: " << port << ", Baud: " << baud << std::endl;

    torpedo::sensor::MiniImuUart imu(port, baud);

    // 1. 초기화 (이제 내부에서 스레드까지 자동으로 시작됨)
    if (!imu.init()) {
        std::cerr << "[Error] Failed to open serial port: " << port << std::endl;
        return 1;
    }

    std::cout << "[Test] IMU Initialized. Reading data..." << std::endl;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(12) << "Time(us)" 
              << std::setw(10) << "Ax" << std::setw(10) << "Ay" << std::setw(10) << "Az" 
              << std::setw(10) << "Gx" << std::setw(10) << "Gy" << std::setw(10) << "Gz" << std::endl;

    torpedo::ImuSample sample;
    while (g_running) {
        // 2. 데이터 읽기 (Lock-free Read)
        if (imu.read(sample)) {
            std::cout << std::setw(12) << sample.t_us 
                      << std::setw(10) << sample.ax 
                      << std::setw(10) << sample.ay 
                      << std::setw(10) << sample.az 
                      << std::setw(10) << sample.gx 
                      << std::setw(10) << sample.gy 
                      << std::setw(10) << sample.gz << "\r" << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
    }

    // 3. 종료
    imu.shutdown();
    std::cout << "\n[Test] Finished. Bad Checksums: " << imu.getErrorCount() << std::endl;

    return 0;
}
