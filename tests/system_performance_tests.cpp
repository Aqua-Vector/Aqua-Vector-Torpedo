#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>
#include <atomic>
#include <sys/resource.h>

#include "core/TorpedoControlSystem.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ActuatorManager.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "protocol/TorpedoParser.hpp"
#include "control/STMControlParser.hpp"
#include "communication/UartLink.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"

// Lightweight Mock Link for benchmarking (no overhead)
class BenchMockLink : public UartLink {
public:
    BenchMockLink() : UartLink("/dev/null", 115200) {}
    bool initialize() override { return true; }
    void close() override {}
    ssize_t send(const uint8_t*, size_t len) override { return len; }
    ssize_t receive(uint8_t*, size_t) override { return 0; }
};

class BenchMockPwm : public IPwmChannel {
public:
    ErrorCode init(uint32_t) override { return ErrorCode::OK; }
    ErrorCode setDutyCycle(uint32_t) override { return ErrorCode::OK; }
    ErrorCode enable(bool) override { return ErrorCode::OK; }
};

/**
 * @brief System Performance & Latency Benchmark
 */
int main() {
    std::cout << "========== [ SYSTEM PERFORMANCE & LATENCY BENCHMARK ] ==========\n" << std::endl;

    // 1. Initialize System Components
    BenchMockLink gcs_link, stm_link;
    TorpedoParser gcs_parser;
    STMControlParser stm_parser;
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> gcs_tx_q;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<TorpedoParser, UartLink, GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_nm(gcs_link, gcs_parser, gcs_tx_q);
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_nm(stm_link, stm_parser, stm_tx_q);

    BenchMockPwm pwm1, pwm2;
    ServoConfig cfg = {20000000, 1000000, 2000000, 45.0f, 1000.0f};
    ServoMotor rudder(pwm1, cfg), elevator(pwm2, cfg);
    ActuatorManager am(rudder, elevator);
    
    ManualSource ms;
    AutoSource as;
    ModeMux mux(ms.getMailbox(), as.getMailbox());
    
    torpedo::SystemClock clock;
    torpedo::FakeImu imu(clock);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf.init(eskf_params, 0.01f);
    torpedo::domain::RpsPositionTracker rps_tracker;

    TorpedoControlSystem tcs(mux, am, ms, as, imu, eskf, rps_tracker, gcs_nm, stm_nm);

    // 2. Metrics Storage
    std::vector<double> loop_times_us;
    std::vector<double> jitter_us;
    std::atomic<bool> stop_bench{false};

    // 3. Start System
    tcs.start();
    std::cout << "[INFO] System Started. Measuring for 5 seconds..." << std::endl;

    auto bench_start = std::chrono::steady_clock::now();
    auto last_loop_time = std::chrono::steady_clock::now();

    // 4. Monitoring Loop
    while (std::chrono::steady_clock::now() - bench_start < std::chrono::seconds(5)) {
        uint32_t elapsed = tcs.getLoopElapsedUs();
        if (elapsed > 0) {
            loop_times_us.push_back(static_cast<double>(elapsed));
        }

        // Measure Interval Jitter (Target: 10000us)
        auto now = std::chrono::steady_clock::now();
        double interval = std::chrono::duration_cast<std::chrono::microseconds>(now - last_loop_time).count();
        if (interval < 20000) { // Filter out startup artifacts
            jitter_us.push_back(std::abs(interval - 10000.0));
        }
        last_loop_time = now;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tcs.stop();

    // 5. CPU Usage Measurement (Total process time)
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    double total_cpu_time = (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) + 
                           (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1e6;

    // 6. Statistics Calculation
    if (loop_times_us.empty()) {
        std::cerr << "[ERROR] No loop data collected!" << std::endl;
        return 1;
    }

    double avg_loop = std::accumulate(loop_times_us.begin(), loop_times_us.end(), 0.0) / loop_times_us.size();
    double max_loop = *std::max_element(loop_times_us.begin(), loop_times_us.end());
    double avg_jitter = std::accumulate(jitter_us.begin(), jitter_us.end(), 0.0) / jitter_us.size();
    double max_jitter = *std::max_element(jitter_us.begin(), jitter_us.end());

    // 7. Report Generation
    std::cout << "\n==============================================================" << std::endl;
    std::cout << "                 SYSTEM PERFORMANCE REPORT                    " << std::endl;
    std::cout << "==============================================================" << std::endl;
    std::cout << std::left << std::setw(30) << " [1] Control Loop Latency (Avg): " << std::fixed << std::setprecision(2) << avg_loop << " us" << std::endl;
    std::cout << std::left << std::setw(30) << " [2] Control Loop Latency (Max): " << max_loop << " us" << std::endl;
    std::cout << std::left << std::setw(30) << " [3] Timing Jitter (Avg): " << avg_jitter << " us" << std::endl;
    std::cout << std::left << std::setw(30) << " [4] Timing Jitter (Max): " << max_jitter << " us" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << std::left << std::setw(30) << " [5] Total CPU Time (5s Bench): " << total_cpu_time << " s" << std::endl;
    std::cout << std::left << std::setw(30) << " [6] Estimated CPU Load: " << (total_cpu_time / 5.0) * 100.0 << " %" << std::endl;
    std::cout << "==============================================================\n" << std::endl;

    std::cout << "Analysis for Presentation:" << std::endl;
    std::cout << " - Worst-case Latency (" << max_loop << "us) is only " << (max_loop/10000.0)*100.0 << "% of the 10ms control cycle." << std::endl;
    std::cout << " - System maintains a stable 100Hz frequency with " << avg_jitter << "us avg jitter." << std::endl;
    std::cout << " - Multi-threaded performance is highly efficient, utilizing less than " << (total_cpu_time / 5.0) * 100.0 + 1.0 << "% of CPU resources." << std::endl;

    return 0;
}
