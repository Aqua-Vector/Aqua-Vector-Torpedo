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
    
    // ─── 1. IMU 선택 (테스트용 override 또는 실제) ───
    if (cfg_.imu_override) {
        imu_ = cfg_.imu_override;
        // 외부 주입 IMU는 호출자가 init/shutdown 책임
    } else {
        Ism330dhcxConfig imu_cfg;
        imu_cfg.spi_device = cfg_.spi_device;
        real_imu_.set_config(imu_cfg);
        if (!real_imu_.init()) {
            std::fprintf(stderr, "[main] IMU init 실패\n");
            return false;
        }
        imu_ = &real_imu_;
    }
    
    // ─── 2. RS-485 초기화 ───
    Rs485Config rs_cfg;
    rs_cfg.device          = cfg_.rs485_device;
    rs_cfg.baud            = cfg_.rs485_baud;
    rs_cfg.read_timeout_ms = 1;  // non-blocking 수준
    
    if (!rs485_.open(rs_cfg)) {
        std::fprintf(stderr, "[main] RS-485 open 실패\n");
        if (!cfg_.imu_override) real_imu_.shutdown();
        return false;
    }
    
    // ─── 3. ESKF 초기화 (기본값) ───
    domain::EskfInitParams eskf_cfg;
    eskf_cfg.p0 = Eigen::Vector3f::Zero();
    eskf_cfg.q0 = Eigen::Quaternionf::Identity();
    eskf_.init(eskf_cfg, cfg_.dt);
    
    bias_.b_a.setZero();
    bias_.b_g.setZero();
    
    initialized_ = true;
    std::printf("[main] 초기화 완료\n");
    std::printf("  IMU: %s\n",
        cfg_.imu_override ? "external (test)" : cfg_.spi_device.c_str());
    std::printf("  RS-485: %s @ %d bps\n",
        cfg_.rs485_device.c_str(), cfg_.rs485_baud);
    std::printf("  주기: %d us (%.0f Hz)\n", cfg_.period_us, 1.0f / cfg_.dt);
    return true;
}

bool MainLoop::calibrate() {
    if (!initialized_) {
        std::fprintf(stderr, "[main] init 안 됨\n");
        return false;
    }
    
    std::printf("[calib] 정지 캘리브레이션 시작 (%.1fs)...\n",
        cfg_.calib_duration_sec);
    
    domain::BiasCalibrator cal;
    int rate_hz = static_cast<int>(1.0f / cfg_.dt);  // 100
    cal.start(cfg_.calib_duration_sec, rate_hz);
    
    SystemClock clock;
    while (!cal.is_done() && !stop_requested_) {
        ImuSample s;
        if (imu_->read(s)) {
            cal.add_sample(s);
        }
        usleep(cfg_.period_us);
    }
    
    if (!cal.is_done()) {
        std::fprintf(stderr, "[calib] 중단됨\n");
        return false;
    }
    
    auto result = cal.finalize();
    if (!result.success) {
        std::fprintf(stderr, "[calib] 실패\n");
        return false;
    }
    
    // bias 설정
    bias_ = result.bias;
    
    // 초기 자세 적용 (ESKF state q에 직접 박기)
    // EskfEstimator에 set_attitude 같은 게 없으면 reinit
    domain::EskfInitParams eskf_cfg;
    eskf_cfg.p0 = Eigen::Vector3f::Zero();
    eskf_cfg.q0 = result.q0;
    eskf_.init(eskf_cfg, cfg_.dt);
    
    std::printf("[calib] 완료 (%d 샘플)\n", result.samples_used);
    std::printf("  roll  = %+.3f rad (%+.2f°)\n",
        result.roll_rad, result.roll_rad * 57.2958f);
    std::printf("  pitch = %+.3f rad (%+.2f°)\n",
        result.pitch_rad, result.pitch_rad * 57.2958f);
    std::printf("  b_a   = (%+.4f, %+.4f, %+.4f) m/s²\n",
        bias_.b_a.x(), bias_.b_a.y(), bias_.b_a.z());
    std::printf("  b_g   = (%+.5f, %+.5f, %+.5f) rad/s\n",
        bias_.b_g.x(), bias_.b_g.y(), bias_.b_g.z());
    
    calibrated_ = true;
    return true;
}

bool MainLoop::gather_sub_samples(ImuSample& out) {
    // 100Hz 사이클 안 8 sub-samples 수집 + Trimmed Mean
    downsampler_.reset();
    
    // 8 sub-samples 시도 — 사이클 시간 안에서
    for (int i = 0; i < IMU_SUB_SAMPLES; i++) {
        ImuSample s;
        // IMU read (STATUS poll 내부)
        // 안 ready면 잠시 대기 후 재시도 (최대 몇 번)
        bool ok = false;
        for (int retry = 0; retry < 4; retry++) {
            if (imu_->read(s)) { ok = true; break; }
            usleep(200);  // 200μs 대기
        }
        if (!ok) {
            // 이번 sub-sample 실패
            continue;
        }
        downsampler_.add(s);
    }
    
    if (!downsampler_.is_full()) {
        return false;  // 충분한 sub-samples 못 모음
    }
    
    out = downsampler_.finalize();
    return out.valid;
}

void MainLoop::run() {
    if (!initialized_) {
        std::fprintf(stderr, "[main] 초기화 안 됨\n");
        return;
    }
    
    if (!calibrated_) {
        std::printf("[main] 경고: 캘리브 안 됨, bias=0으로 진행\n");
    }
    
    std::printf("[main] 루프 시작 (Ctrl+C로 중단)\n");
    
    SystemClock clock;
    uint64_t last_print_us = clock.now_us();
    const uint64_t print_interval_us = cfg_.print_interval_sec * 1000000ULL;
    
    while (!stop_requested_) {
        uint64_t loop_start = clock.now_us();
        
        // ─── 1. 8 sub-samples + Trimmed Mean ───
        ImuSample filtered;
        bool imu_ok = gather_sub_samples(filtered);
        if (imu_ok) {
            imu_ok_count_++;
            // ─── 2. ESKF predict (bias 보정은 ESKF 내부) ───
            eskf_.predict(filtered, bias_, cfg_.dt);
        }
        
        // ─── 3. RS-485 downlink 수신 ───
        if (process_downlink()) {
            downlink_count_++;
        }
        
        // ─── 4. NHC ───
        if (eskf_.state().v.norm() > 0.1f) {
            eskf_.update_nhc();
        }
        
        // ─── 5. Uplink 송신 ───
        send_uplink();
        
        loop_count_++;
        
        // ─── 6. 통계 출력 ───
        uint64_t now = clock.now_us();
        if (now - last_print_us >= print_interval_us) {
            const auto& x = eskf_.state();
            std::printf("[%.1fs] loops=%u imu=%u downlink=%u update=%u | "
                       "p=(%.3f,%.3f) v=(%.3f,%.3f) P_tr=%.4f\n",
                       now / 1e6f,
                       loop_count_, imu_ok_count_, downlink_count_,
                       update_lidar_count_,
                       x.p.x(), x.p.y(), x.v.x(), x.v.y(),
                       x.P.trace());
            last_print_us = now;
        }
        
        // ─── 7. 다음 사이클 대기 ───
        uint64_t elapsed = clock.now_us() - loop_start;
        if (elapsed < static_cast<uint64_t>(cfg_.period_us)) {
            usleep(cfg_.period_us - elapsed);
        }
    }
    
    std::printf("[main] 루프 종료\n");
    std::printf("  총 사이클: %u\n", loop_count_);
    std::printf("  IMU OK: %u\n", imu_ok_count_);
    std::printf("  Downlink 수신: %u\n", downlink_count_);
    std::printf("  LiDAR update: %u\n", update_lidar_count_);
}

bool MainLoop::process_downlink() {
    uint8_t buf[64];
    int n = rs485_.read_bytes(buf, sizeof(buf));
    if (n < static_cast<int>(sizeof(DownlinkPacket))) {
        return false;
    }
    
    // sync byte 찾기
    int sync_idx = -1;
    for (int i = 0; i <= n - static_cast<int>(sizeof(DownlinkPacket)); ++i) {
        if (buf[i] == SYNC_DOWNLINK) {
            sync_idx = i;
            break;
        }
    }
    if (sync_idx < 0) return false;
    
    // 파싱 (CRC 검증 포함)
    DownlinkPacket pkt;
    if (!parse_downlink(buf + sync_idx, n - sync_idx, pkt)) {
        return false;
    }
    
    // LiDAR 측정 유효 → update_lidar
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
    
    auto euler = x.q.toRotationMatrix().eulerAngles(2, 1, 0);
    up.yaw = euler[0];
    
    up.status_flags = static_cast<uint8_t>(StatusFlag::EskfOk);
    if (calibrated_) {
        up.status_flags |= static_cast<uint8_t>(StatusFlag::BiasReady);
    }
    up.reserved     = 0;
    up.crc16        = 0;  // serialize 함수가 자동 계산
    
    uint8_t buf[64];
    std::size_t n = serialize_uplink(up, buf);
    rs485_.write_bytes(buf, n);
}

const domain::EskfState& MainLoop::state() const {
    return eskf_.state();
}

void MainLoop::shutdown() {
    if (initialized_) {
        rs485_.close();
        if (!cfg_.imu_override) {
            real_imu_.shutdown();
        }
        initialized_ = false;
        std::printf("[main] 정리 완료\n");
    }
}

} // namespace torpedo