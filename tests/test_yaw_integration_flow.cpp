#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

#include "core/TorpedoControlSystem.hpp"
#include "guidance/EbImuUart.hpp"
#include "control/ModeMux.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/GenericPacket.hpp"

/**
 * @brief GCS 송신 패킷을 가로채는 Spy 클래스
 */
class SpyGcsManager : public ITxNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>> {
public:
    TorpedoUplinkPayload last_payload;
    bool start() override { return true; }
    void stop() override {}
    bool send(const GenericPacket<TorpedoUplinkPayload, uint16_t>& packet) override {
        last_payload = packet.payload; // 송신 직전의 데이터를 복사
        return true;
    }
};

/**
 * @brief STM32 송신 패킷을 가로채는 Spy 클래스
 */
class SpyStm32Manager : public ITxNetworkManager<STMPacket> {
public:
    ControlPayload last_payload;
    bool start() override { return true; }
    void stop() override {}
    bool send(const STMPacket& packet) override {
        last_payload = packet.payload; // 송신 직전의 데이터를 복사
        return true;
    }
};

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "   Full-Path Yaw Sign Trace Test (Pure Flow)    " << std::endl;
    std::cout << "================================================" << std::endl;

    // 1. 의존성 객체 준비
    torpedo::sensor::EbImuUart imu("/dev/ttyUSB0", 921600);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    ManualSource ms; AutoSource as;
    ModeMux mux(ms.getMailbox(), as.getMailbox());
    
    // Spy Managers (패킷 캡처용)
    auto gcs_spy = new SpyGcsManager();
    auto stm_spy = new SpyStm32Manager();

    // 2. TCS 생성
    TorpedoControlSystem tcs(mux, ms, as, imu, eskf, rps_tracker, *gcs_spy, *stm_spy);

    // 3. 초기화 (5초 정지 캘리브레이션 실행)
    std::cout << "[Test] Step 1: Calibration (Don't touch for 5s)..." << std::endl;
    if (!tcs.init()) return -1;

    // 4. 시스템 가동
    tcs.start();
    std::cout << "[Test] Step 2: Main Loop Running." << std::endl;
    std::cout << ">>> ACTION: Rotate RIGHT (CW) and see the signs <<<" << std::endl;
    std::cout << "----------------------------------------------------------------------" << std::endl;

    auto start_t = std::chrono::steady_clock::now();
    uint64_t last_imu_t = 0;

    while (tcs.isRunning()) {
        auto now = std::chrono::steady_clock::now();
        
        // 100Hz 주기로 가상 속도 주입 (1.0m/s 전진)
        FeedbackPayload fb; 
        fb.m1_rps = 15.0f; fb.m2_rps = 15.0f; // 전진 (abs 합산 로직 대응)
        fb.servo_pos = 1500; // 중립 가동
        tcs.onStm32FeedbackReceived(fb, std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());

        torpedo::ImuSample raw_s;
        if (imu.read(raw_s) && raw_s.t_us != last_imu_t) {
            last_imu_t = raw_s.t_us;

            // 0.5초 주기로 모든 단계의 Yaw값 관찰
            static auto last_log = std::chrono::steady_clock::now();
            if (now - last_log >= std::chrono::milliseconds(500)) {
                last_log = now;

                // [지점 1] Raw IMU (센서 원시값)
                float raw_deg = raw_s.yaw * 180.0f / M_PI;

                // [지점 2] Internal (캘리브레이션 이후 TCS 내부 각도)
                float internal_deg = tcs.getLatestCumulativeYaw() * 180.0f / M_PI;

                // [지점 3] Telemetry (통제소 송신 직전 패킷 데이터)
                float gcs_pkt_deg = gcs_spy->last_payload.yaw * 180.0f / M_PI;

                // [지점 4] Control (STM32 송신 직전 조향 명령 - Yaw의 결과물)
                float stm_steer = stm_spy->last_payload.rudder;

                // [지점 5] Estimation (실제 위치 좌표 누적 결과)
                const auto& pos = rps_tracker.getPosition();

                std::cout << std::fixed << std::setprecision(1)
                          << "[Trace] RAW: " << std::setw(6) << raw_deg << " | "
                          << "INT: " << std::setw(6) << internal_deg << " | "
                          << "GCS: " << std::setw(6) << gcs_pkt_deg << " | "
                          << "STEER: " << std::setw(5) << stm_steer << " | "
                          << "POS_X: " << std::setprecision(2) << std::setw(5) << pos.x() 
                          << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (now - start_t > std::chrono::seconds(60)) break;
    }

    tcs.stop();
    return 0;
}
