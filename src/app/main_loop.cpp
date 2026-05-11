#include "torpedo/app/main_loop.hpp"
#include "torpedo/hal/system_clock.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <Eigen/Dense>

namespace torpedo {

bool MainLoop::init(const MainLoopConfig& cfg) {
    cfg_ = cfg;
    
    // -------- 1. IMU 초기화 --------
    Ism330dhcxConfig imu_cfg;
    imu_cfg.spi_device = cfg_.spi_device;
    imu_.set_config(imu_cfg);
    
    if (!imu_.init()) {
        std::fprintf(stderr, "[main] IMU init 실패\n");
        return false;
    }
    
    // -------- 2. RS-485 초기화 --------
    Rs485Config rs_cfg;
    rs_cfg.device          = cfg_.rs485_device;
    rs_cfg.baud            = cfg_.rs485_baud;
    rs_cfg.read_timeout_ms = 1;  // non-blocking 수준
    
    if (!rs485_.open(rs_cfg)) {
        std::fprintf(stderr, "[main] RS-485 open 실패\n");
        imu_.shutdown();
        return false;
    }
    
    // -------- 3. ESKF 초기화 --------
    domain::EskfInitParams eskf_cfg;
    eskf_cfg.p0 = Eigen::Vector3f::Zero();
    eskf_cfg.q0 = Eigen::Quaternionf::Identity();
    eskf_.init(eskf_cfg, cfg_.dt);
    
    // -------- 4. Bias 0으로 시작 (캘리브는 Day 6) --------
    bias_.b_a.setZero();
    bias_.b_g.setZero();
    
    initialized_ = true;
    std::printf("[main] 초기화 완료\n");
    std::printf("  IMU: %s\n", cfg_.spi_device.c_str());
    std::printf("  RS-485: %s @ %d bps\n", cfg_.rs485_device.c_str(), cfg_.rs485_baud);
    std::printf("  주기: %d us (%.0f Hz)\n", cfg_.period_us, 1.0f / cfg_.dt);
    return true;
}

void MainLoop::run() {
    if (!initialized_) {
        std::fprintf(stderr, "[main] 초기화 안 됨\n");
        return;
    }
    
    std::printf("[main] 루프 시작 (Ctrl+C로 중단)\n");
    
    SystemClock clock;
    uint64_t last_print_us = clock.now_us();
    const uint64_t print_interval_us = cfg_.print_interval_sec * 1000000ULL;
    
    while (!stop_requested_) {
        uint64_t loop_start = clock.now_us();
        
        // -------- 1. IMU read --------
        ImuSample s;
        bool imu_ok = imu_.read(s);
        if (imu_ok) {
            imu_ok_count_++;
            // -------- 2. ESKF predict --------
            eskf_.predict(s, bias_, cfg_.dt);
        }
        
        // -------- 3. RS-485 downlink 수신 --------
        if (process_downlink()) {
            downlink_count_++;
        }
        
        // -------- 4. NHC (속도 > 0.1 m/s일 때만) --------
        if (eskf_.state().v.norm() > 0.1f) {
            eskf_.update_nhc();
        }
        
        // -------- 5. Uplink 송신 --------
        send_uplink();
        
        loop_count_++;
        
        // -------- 6. 통계 출력 (1초마다) --------
        uint64_t now = clock.now_us();
        if (now - last_print_us >= print_interval_us) {
            float dt_sec = (now - last_print_us) / 1e6f;
            const auto& x = eskf_.state();
            std::printf("[%.1fs] loops=%u imu=%u downlink=%u update=%u | "
                       "p=(%.3f,%.3f) v=(%.3f,%.3f) P=%.4f\n",
                       now / 1e6f,
                       loop_count_, imu_ok_count_, downlink_count_, update_lidar_count_,
                       x.p.x(), x.p.y(), x.v.x(), x.v.y(),
                       x.P.trace());
            last_print_us = now;
        }
        
        // -------- 7. 다음 사이클까지 대기 --------
        uint64_t elapsed = clock.now_us() - loop_start;
        if (elapsed < static_cast<uint64_t>(cfg_.period_us)) {
            usleep(cfg_.period_us - elapsed);
        }
        // 만약 elapsed > period면 그냥 다음 사이클 (overrun, 통계로 잡힘)
    }
    
    std::printf("[main] 루프 종료\n");
    std::printf("  총 사이클: %u\n", loop_count_);
    std::printf("  IMU OK: %u\n", imu_ok_count_);
    std::printf("  Downlink 수신: %u\n", downlink_count_);
    std::printf("  LiDAR update: %u\n", update_lidar_count_);
}

bool MainLoop::process_downlink() {
    // RS-485에서 한 패킷 시도
    uint8_t buf[64];
    int n = rs485_.read_bytes(buf, sizeof(buf));
    if (n < static_cast<int>(sizeof(DownlinkPacket))) {
        return false;  // timeout 또는 짧음
    }
    
    // sync byte 찾기 (간단 sync 동기화)
    int sync_idx = -1;
    for (int i = 0; i <= n - static_cast<int>(sizeof(DownlinkPacket)); ++i) {
        if (buf[i] == SYNC_DOWNLINK) {
            sync_idx = i;
            break;
        }
    }
    if (sync_idx < 0) return false;
    
    // 파싱
    DownlinkPacket pkt;
    if (!parse_downlink(buf + sync_idx, n - sync_idx, pkt)) {
        return false;
    }
    
    // LiDAR 측정 유효 → ESKF update_lidar
    if (!std::isnan(pkt.torpedo_x) && !std::isnan(pkt.torpedo_y)) {
        Eigen::Vector2f z(pkt.torpedo_x, pkt.torpedo_y);
        eskf_.update_lidar(z);
        update_lidar_count_++;
    }
    
    return true;
}

void MainLoop::send_uplink() {
    const auto& x = eskf_.state();
    
    UplinkPacket up;
    up.sync          = SYNC_UPLINK;
    up.timestamp_us  = static_cast<uint32_t>(SystemClock().now_us() & 0xFFFFFFFF);
    up.seq           = tx_seq_++;
    up.p_x           = x.p.x();
    up.p_y           = x.p.y();
    up.v_x           = x.v.x();
    up.v_y           = x.v.y();
    
    // yaw 추출 (quaternion → euler)
    auto euler = x.q.toRotationMatrix().eulerAngles(2, 1, 0);
    up.yaw = euler[0];
    
    up.status_flags = static_cast<uint8_t>(StatusFlag::EskfOk);
    up.reserved     = 0;
    up.crc16        = 0;  // P1
    
    uint8_t buf[64];
    std::size_t n = serialize_uplink(up, buf);
    rs485_.write_bytes(buf, n);
}

void MainLoop::shutdown() {
    if (initialized_) {
        rs485_.close();
        imu_.shutdown();
        initialized_ = false;
        std::printf("[main] 정리 완료\n");
    }
}

} // namespace torpedo