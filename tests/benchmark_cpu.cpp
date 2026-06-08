#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include "core/NetworkManager.hpp"
#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "core/TorpedoControlSystem.hpp" // For STMPacket alias

/**
 * @brief CPU 사용률(Duty Cycle) 측정용 벤치마크
 * 100Hz 루프에서 실제 작업이 차지하는 시간 비율을 계산합니다.
 */
int main() {
    std::cout << "=== Aqua-Vector CPU Usage Benchmark (100Hz Loop) ===" << std::endl;

    // 가상의 통신 환경 설정
    UartLink link("/dev/ttyPS1", 230400); 
    STMControlParser parser;
    StaticRingBuffer<STMPacket, 64> tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> nm(link, parser, tx_q);

    nm.start();

    const int TOTAL_SAMPLES = 500; // 5초간 측정
    std::vector<double> duty_cycles;
    
    auto next_loop = std::chrono::steady_clock::now();
    
    std::cout << "Measuring CPU Load..." << std::endl;

    for (int i = 0; i < TOTAL_SAMPLES; ++i) {
        auto start = std::chrono::steady_clock::now();
        
        // --- 실제 부하 시뮬레이션 (TCS 메인 루프 작업량) ---
        STMPacket pkt;
        pkt.msg_id = 0x01;
        pkt.payload.velocity = -60.0f;
        pkt.payload.rudder = 10.0f;
        nm.send(pkt); 
        
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 연산 부하 0.5ms 가정
        // ----------------------------------------------

        auto end = std::chrono::steady_clock::now();
        
        double elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        duty_cycles.push_back(elapsed_us / 10000.0 * 100.0); // 10ms(100Hz) 대비 점유율 %

        next_loop += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_loop);

        if (i % 100 == 0) std::cout << "." << std::flush;
    }

    double avg_duty = std::accumulate(duty_cycles.begin(), duty_cycles.end(), 0.0) / duty_cycles.size();
    
    std::cout << "\n\nBenchmark Result:" << std::endl;
    std::cout << " - Average Task Duty Cycle: " << std::fixed << std::setprecision(2) << avg_duty << "%" << std::endl;
    std::cout << " - Peak Task Duty Cycle:    " << *std::max_element(duty_cycles.begin(), duty_cycles.end()) << "%" << std::endl;
    std::cout << "\n(Note: If this value is near 100%, the system is unstable.)" << std::endl;

    nm.stop();
    return 0;
}
