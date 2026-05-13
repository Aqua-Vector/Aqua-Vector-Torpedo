// main.cpp — 어뢰 ZYBO 진입점
//
// 빌드:
//   g++ -std=c++17 -O2 -Iinclude -I/usr/include/eigen3
//       src/app/main.cpp src/app/main_loop.cpp
//       src/comm/spidev.cpp src/comm/rs485_port.cpp src/comm/packet_io.cpp
//       src/sensor/ism330dhcx_imu.cpp
//       src/hal/system_clock.cpp
//       src/domain/estimator/eskf_estimator.cpp
//       -o torpedo_main

#include "torpedo/app/main_loop.hpp"

#include <csignal>
#include <cstdio>

static torpedo::MainLoop* g_loop = nullptr;

static void signal_handler(int sig) {
    (void)sig;
    if (g_loop) g_loop->request_stop();
}

int main(int argc, char** argv) {
    torpedo::MainLoopConfig cfg;
    
    // CLI 인자 (옵션)
    if (argc >= 2) cfg.rs485_device = argv[1];
    if (argc >= 3) cfg.spi_device   = argv[2];
    
    torpedo::MainLoop loop;
    g_loop = &loop;
    
    // Ctrl+C 처리
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    if (!loop.init(cfg)) {
        std::fprintf(stderr, "[main] 초기화 실패\n");
        return 1;
    }

    // 정지 캘리브레이션 (5초)
    if (!loop.calibrate()) {
        std::fprintf(stderr, "[main] 캘리브 실패 (계속 진행)\n");
    }
    
    loop.run();
    loop.shutdown();
    
    return 0;
}