// 통합 시뮬 테스트 — 메인 루프 핵심 흐름 검증 (VM에서)
//
// 시나리오:
//   1. BiasCalibrator (FakeImu 500 sample)
//   2. ESKF predict 100회 (캘리브된 bias 적용)
//   3. 가짜 Downlink (CRC 박힘) → parse_downlink 통과
//   4. update_lidar → ESKF 좌표 수렴
//   5. 시퀀스 + status_flags 검증
//
// VM에서 IMU/RS-485 없이도 메인 흐름 검증.

#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/sensor/imu_downsampler.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "torpedo/comm/packet.hpp"
#include "torpedo/comm/packet_io.hpp"
#include "torpedo/comm/crc16.hpp"
#include "torpedo/hal/system_clock.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>

using namespace torpedo;
using namespace torpedo::domain;

static bool approx(float a, float b, float tol) {
    return std::abs(a - b) < tol;
}

// 가짜 Downlink 패킷 만들기 (CRC 박힘)
static void build_downlink(uint8_t* buf, uint16_t seq,
                           float tx, float ty,
                           float mx, float my) {
    DownlinkPacket pkt;
    pkt.sync         = SYNC_DOWNLINK;
    pkt.timestamp_us = 1000;
    pkt.seq          = seq;
    pkt.target_x     = tx;
    pkt.target_y     = ty;
    pkt.torpedo_x    = mx;
    pkt.torpedo_y    = my;
    pkt.crc16        = 0;
    
    std::memcpy(buf, &pkt, sizeof(pkt));
    // CRC 계산 (sync ~ torpedo_y, 23 byte)
    uint16_t crc = crc16_ccitt(buf, 23);
    std::memcpy(buf + 23, &crc, 2);
}

int main() {
    printf("================================================\n");
    printf(" 통합 시뮬 — 메인 루프 핵심 흐름\n");
    printf("================================================\n\n");
    
    // ─── Phase 1: 캘리브레이션 ───
    printf("[Phase 1] BiasCalibrator (FakeImu 정지 5초)\n");
    
    SystemClock clock;
    FakeImu imu(clock);
    imu.init();
    
    BiasCalibrator cal;
    cal.start(5.0f, 100);  // 5초 × 100Hz = 500 samples
    
    int valid_samples = 0;
    while (!cal.is_done() && valid_samples < 600) {
        ImuSample s;
        if (imu.read(s)) {
            cal.add_sample(s);
            valid_samples++;
        }
    }
    
    if (!cal.is_done()) {
        printf("[FAIL] 캘리브 완료 안 됨 (%d/500)\n", valid_samples);
        return 1;
    }
    
    auto cal_result = cal.finalize();
    if (!cal_result.success) {
        printf("[FAIL] cal.finalize 실패\n");
        return 1;
    }
    
    printf("  Samples: %d\n", cal_result.samples_used);
    printf("  roll  = %+.4f rad\n", cal_result.roll_rad);
    printf("  pitch = %+.4f rad\n", cal_result.pitch_rad);
    printf("  b_a   = (%+.4f, %+.4f, %+.4f)\n",
        cal_result.bias.b_a.x(), cal_result.bias.b_a.y(), cal_result.bias.b_a.z());
    printf("  b_g   = (%+.5f, %+.5f, %+.5f)\n",
        cal_result.bias.b_g.x(), cal_result.bias.b_g.y(), cal_result.bias.b_g.z());
    
    // FakeImu가 정지+중력만 출력 → bias 거의 0이어야
    if (cal_result.bias.b_g.norm() > 1e-3f) {
        printf("[FAIL] b_g 너무 큼\n");
        return 1;
    }
    printf("  OK\n\n");
    
    // ─── Phase 2: ESKF init + predict ───
    printf("[Phase 2] ESKF predict 100회 (캘리브된 bias 적용)\n");
    
    EskfInitParams eskf_cfg;
    eskf_cfg.p0 = Eigen::Vector3f::Zero();
    eskf_cfg.q0 = cal_result.q0;
    
    EskfEstimator eskf;
    eskf.init(eskf_cfg, 0.01f);
    
    BiasEstimate bias = cal_result.bias;
    
    for (int i = 0; i < 100; i++) {
        ImuSample s;
        if (imu.read(s)) {
            eskf.predict(s, bias, 0.01f);
        }
    }
    
    const auto& x_before = eskf.state();
    printf("  100회 후 p = (%.4f, %.4f, %.4f) m\n",
        x_before.p.x(), x_before.p.y(), x_before.p.z());
    printf("  v = (%.4f, %.4f, %.4f) m/s\n",
        x_before.v.x(), x_before.v.y(), x_before.v.z());
    printf("  P trace = %.4f\n", x_before.P.trace());
    
    // 정지이면 p, v 작아야 (drift 발생 가능하지만 1m 이하)
    if (x_before.p.norm() > 1.0f) {
        printf("[FAIL] 1초 동안 p drift 너무 큼\n");
        return 1;
    }
    printf("  OK (정지 + 캘리브 → drift 제어됨)\n\n");
    
    // ─── Phase 3: 가짜 Downlink (CRC 정상) → parse ───
    printf("[Phase 3] 가짜 Downlink 파싱 (CRC 검증)\n");
    
    uint8_t buf[64];
    build_downlink(buf, 42, 2.0f, 3.0f, 1.5f, 2.5f);
    
    DownlinkPacket parsed;
    bool ok = parse_downlink(buf, 25, parsed);
    if (!ok) {
        printf("[FAIL] parse_downlink 실패 (CRC?)\n");
        return 1;
    }
    
    if (parsed.seq != 42) { printf("[FAIL] seq\n"); return 1; }
    if (!approx(parsed.target_x, 2.0f, 1e-5f)) { printf("[FAIL] target_x\n"); return 1; }
    if (!approx(parsed.torpedo_x, 1.5f, 1e-5f)) { printf("[FAIL] torpedo_x\n"); return 1; }
    printf("  Downlink 정상 파싱 (seq=%u, target=(%.1f,%.1f), torpedo=(%.1f,%.1f))\n",
        parsed.seq, parsed.target_x, parsed.target_y,
        parsed.torpedo_x, parsed.torpedo_y);
    printf("  OK\n\n");
    
    // ─── Phase 4: 손상된 CRC → parse 거부 ───
    printf("[Phase 4] 손상된 CRC → parse 거부\n");
    
    build_downlink(buf, 100, 1.0f, 1.0f, 1.0f, 1.0f);
    // CRC byte 1개 손상
    buf[23] ^= 0xFF;
    
    DownlinkPacket bad;
    if (parse_downlink(buf, 25, bad)) {
        printf("[FAIL] 손상된 CRC 통과시킴\n");
        return 1;
    }
    printf("  OK (CRC 검증 동작)\n\n");
    
    // ─── Phase 5: update_lidar → 좌표 수렴 ───
    printf("[Phase 5] update_lidar 반복 → ESKF p가 LiDAR 좌표로 수렴\n");
    
    // 가짜 LiDAR가 (4, 5) 일관되게 측정
    const Eigen::Vector2f z_target(4.0f, 5.0f);
    float P_before = eskf.state().P.trace();
    
    for (int i = 0; i < 50; i++) {
        eskf.update_lidar(z_target);
    }
    
    const auto& x_after = eskf.state();
    float P_after = x_after.P.trace();
    
    printf("  P trace: %.4f → %.4f (감소: %.1f%%)\n",
        P_before, P_after, 100.0f * (P_before - P_after) / P_before);
    printf("  p = (%.4f, %.4f) (target (4.0, 5.0))\n",
        x_after.p.x(), x_after.p.y());
    
    // P 단조 감소
    if (P_after >= P_before) {
        printf("[FAIL] P 감소 안 함\n");
        return 1;
    }
    // p가 (4, 5)에 가까워져야
    if (std::abs(x_after.p.x() - 4.0f) > 0.5f) {
        printf("[FAIL] p_x 수렴 X\n");
        return 1;
    }
    if (std::abs(x_after.p.y() - 5.0f) > 0.5f) {
        printf("[FAIL] p_y 수렴 X\n");
        return 1;
    }
    printf("  OK (P 감소 + p 수렴)\n\n");
    
    // ─── Phase 6: Uplink 직렬화 round-trip ───
    printf("[Phase 6] Uplink 직렬화 + 역파싱\n");
    
    UplinkPacket up;
    up.sync = SYNC_UPLINK;
    up.timestamp_us = 7777;
    up.seq = 999;
    up.p_x = x_after.p.x();
    up.p_y = x_after.p.y();
    up.yaw = 0.5f;
    up.status_flags = static_cast<uint8_t>(StatusFlag::EskfOk) 
                    | static_cast<uint8_t>(StatusFlag::BiasReady);
    up.reserved = 0;
    up.crc16 = 0;
    
    uint8_t up_buf[64];
    std::size_t n = serialize_uplink(up, up_buf);
    if (n != sizeof(UplinkPacket)) {
        printf("[FAIL] serialize 길이\n");
        return 1;
    }
    
    // CRC 박혀있는지 확인 (마지막 2 byte != 0)
    uint16_t crc_check;
    std::memcpy(&crc_check, up_buf + n - 2, 2);
    if (crc_check == 0) {
        printf("[FAIL] CRC 안 박힘\n");
        return 1;
    }
    
    // CRC 다시 계산해서 검증
    uint16_t expected_crc = crc16_ccitt(up_buf, n - 2);
    if (crc_check != expected_crc) {
        printf("[FAIL] CRC 잘못 (%04X vs %04X)\n", crc_check, expected_crc);
        return 1;
    }
    
    printf("  Uplink %zu byte 직렬화 OK, CRC=0x%04X\n", n, crc_check);
    printf("  OK\n\n");
    
    imu.shutdown();
    
    printf("================================================\n");
    printf(" [OK] 통합 시뮬 — 메인 흐름 6/6 통과\n");
    printf("================================================\n");
    return 0;
}