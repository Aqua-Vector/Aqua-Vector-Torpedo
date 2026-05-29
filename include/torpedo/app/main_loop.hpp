// main_loop.hpp — 어뢰 메인 루프
//
// 100Hz 주기로:
//   - 사이클 안 8 sub-samples IMU read (Trimmed Mean)
//   - bias 보정 + ESKF predict
//   - RS-485 downlink 수신 → ESKF update_lidar (NaN 아니면)
//   - NHC 조건부
//   - Uplink 송신
//
// 시작 시 5초 정지 캘리브 → bias + 초기 자세 추정.
//
// 참조: ADR-001, ADR-005, ADR-006, ADR-007

#pragma once

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/sensor/ism330dhcx_imu.hpp"
#include "torpedo/sensor/imu_downsampler.hpp"
#include "torpedo/comm/rs485_port.hpp"
#include "torpedo/comm/packet.hpp"
#include "torpedo/comm/packet_io.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_estimate.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace torpedo {

struct MainLoopConfig {
    // RS-485
    std::string rs485_device = "/dev/ttyPS1";
    int         rs485_baud   = 460800;
    
    // IMU
    std::string spi_device   = "/dev/spidev0.0";
    
    // 루프 주기
    float       dt           = 0.01f;       // 10ms = 100Hz
    int         period_us    = 10000;
    
    // 캘리브
    float       calib_duration_sec = 5.0f;
    
    // 통계 출력 주기 (초)
    int         print_interval_sec = 1;
    
    // IImu 주입용 (테스트에서 FakeImu 가능)
    IImu*       imu_override = nullptr;
};

class MainLoop {
public:
    MainLoop() = default;
    ~MainLoop() = default;
    
    /// 초기화 (IMU, RS-485, ESKF)
    bool init(const MainLoopConfig& cfg);
    
    /// 정지 캘리브레이션 실행 (5초)
    bool calibrate();
    
    /// 메인 루프 실행 (blocking)
    void run();
    
    /// 종료 요청 (signal handler에서)
    void request_stop() { stop_requested_ = true; }
    
    /// 정리
    void shutdown();
    
    // ── 테스트용 ──
    const domain::EskfState& state() const;
    
private:
    MainLoopConfig    cfg_;
    
    // IMU (실제 또는 fake, override 가능)
    Ism330dhcxImu     real_imu_;
    IImu*             imu_ = nullptr;
    
    Rs485Port             rs485_;
    domain::EskfEstimator eskf_;
    domain::BiasEstimate  bias_;
    ImuDownsampler        downsampler_;
    
    std::atomic<bool> stop_requested_{false};
    bool              initialized_ = false;
    bool              calibrated_  = false;
    
    uint16_t          tx_seq_ = 0;
    uint32_t          loop_count_ = 0;
    uint32_t          imu_ok_count_ = 0;
    uint32_t          downlink_count_ = 0;
    uint32_t          update_lidar_count_ = 0;
    
    bool process_downlink();
    void send_uplink();
    bool gather_sub_samples(ImuSample& out);  // 8 sub-samples → Trimmed Mean

    // IMU 비동기 버퍼
    static constexpr int IMU_RING_SIZE = 32;  // 2사이클치 여유
    ImuSample            imu_ring_[IMU_RING_SIZE];
    std::atomic<int>     imu_write_idx_{0};
    std::atomic<int>     imu_read_idx_{0};
    
    std::thread          imu_thread_;
    std::atomic<bool>    imu_thread_running_{false};
    
    void imu_thread_func();
    bool pop_imu_samples(ImuSample* out, int n);  // n개 꺼내기
};

} // namespace torpedo