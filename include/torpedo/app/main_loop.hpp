// main_loop.hpp — 어뢰 메인 루프
//
// IMU → 신호처리(스킵) → ESKF predict → RS-485 수신/송신 → ESKF update
// 100Hz 주기, usleep 기반 단순 타이밍.
//
// 참조: ADR-005 (ESKF), ADR-006 (외부 측정), ADR-007 (메인 루프 100Hz)

#pragma once

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/sensor/ism330dhcx_imu.hpp"
#include "torpedo/comm/rs485_port.hpp"
#include "torpedo/comm/packet.hpp"
#include "torpedo/comm/packet_io.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_estimate.hpp"

#include <atomic>
#include <cstdint>

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
    
    // 통계 출력 주기 (초)
    int         print_interval_sec = 1;
};

/**
 * 어뢰 메인 루프.
 *
 * 사용:
 *   MainLoop loop;
 *   if (!loop.init(cfg)) return -1;
 *   loop.run();      // blocking, Ctrl+C로 중단
 *   loop.shutdown();
 */
class MainLoop {
public:
    MainLoop() = default;
    ~MainLoop() = default;
    
    /// 초기화 (IMU, RS-485, ESKF 다)
    bool init(const MainLoopConfig& cfg);
    
    /// 메인 루프 실행 (blocking)
    void run();
    
    /// 종료 요청 (signal handler에서)
    void request_stop() { stop_requested_ = true; }
    
    /// 정리
    void shutdown();
    
private:
    MainLoopConfig    cfg_;
    Ism330dhcxImu     imu_;
    Rs485Port         rs485_;
    domain::EskfEstimator     eskf_;
    domain::BiasEstimate      bias_;  // 일단 0
    
    std::atomic<bool> stop_requested_{false};
    bool              initialized_ = false;
    
    uint16_t          tx_seq_ = 0;
    uint32_t          loop_count_ = 0;
    uint32_t          imu_ok_count_ = 0;
    uint32_t          downlink_count_ = 0;
    uint32_t          update_lidar_count_ = 0;
    
    // 내부 메서드
    bool process_downlink();
    void send_uplink();
};

} // namespace torpedo