#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Dense>

#include "torpedo/sensor/MiniImuUart.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "utils/LowPassFilter.hpp"

int main(int argc, char** argv) {
    std::string port = "/dev/ttyS3";
    if (argc > 1) port = argv[1];

    std::cout << "========== Stationary Position Drift Test ==========" << std::endl;
    std::cout << "Assumption: RPS = 0 (Fixed Speed = 0.0 m/s)" << std::endl;
    std::cout << "Step 1: 5s Static Calibration..." << std::endl;

    torpedo::sensor::MiniImuUart imu(port, 115200);
    torpedo::domain::RpsPositionTracker tracker;
    utils::LowPassFilter yaw_lpf(0.3f);
    torpedo::domain::BiasCalibrator cal;

    if (!imu.init()) {
        std::cerr << "IMU Init Failed" << std::endl;
        return -1;
    }

    // 1. 5초 정지 캘리브레이션
    cal.start(5.0f, 100);
    uint64_t last_t_us = 0;
    while (!cal.is_done()) {
        torpedo::ImuSample s;
        if (imu.read(s)) {
            if (s.t_us != last_t_us) {
                cal.add_sample(s);
                last_t_us = s.t_us;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    auto cal_res = cal.finalize();
    std::cout << "[Step 1 Done] Calibration Success: " << (cal_res.success ? "YES" : "NO") << std::endl;
    std::cout << " - Initial Roll/Pitch: " << cal_res.roll_rad*180/M_PI << ", " << cal_res.pitch_rad*180/M_PI << " deg" << std::endl;

    // 2. 위치 추정 루프
    std::cout << "\nStep 2: Tracking Position (RPS=0)..." << std::endl;
    std::cout << "  Time [s] |    Yaw [deg] | Pos X [m] | Pos Y [m] | Dist Error [m]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    float dt = 0.01f; // 100Hz

    while (true) {
        torpedo::ImuSample sample;
        if (imu.read(sample)) {
            // 헤딩 필터링
            float filtered_yaw = yaw_lpf.updateAngle(sample.yaw);
            
            // RPS = 0 가정 (정지 상태)
            float speed = 0.0f; 
            
            // 위치 업데이트 (Stable Yaw 기반)
            Eigen::Quaternionf q_stable = Eigen::Quaternionf(Eigen::AngleAxisf(filtered_yaw, Eigen::Vector3f::UnitZ()));
            tracker.update(speed, q_stable, dt);

            auto now = std::chrono::steady_clock::now();
            if (now - last_log_time >= std::chrono::seconds(1)) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
                const auto& pos = tracker.getPosition();
                float dist_error = std::sqrt(pos.x()*pos.x() + pos.y()*pos.y());

                std::cout << std::fixed << std::setprecision(3);
                std::cout << std::setw(10) << elapsed << " | "
                          << std::setw(12) << filtered_yaw * 180.0f / M_PI << " | "
                          << std::setw(9) << pos.x() << " | "
                          << std::setw(9) << pos.y() << " | "
                          << std::setw(14) << dist_error << std::endl;

                last_log_time = now;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    imu.shutdown();
    return 0;
}
