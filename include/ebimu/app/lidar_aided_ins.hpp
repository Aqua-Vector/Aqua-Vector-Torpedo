// lidar_aided_ins.hpp — EBIMU INS + LiDAR 보정 통합 (버전 1)
//
// 동작:
//   - EBIMU IMU로 6-state ESKF predict (항상)
//   - ZUPT 정지 감지 시 속도 0 update
//   - LiDAR 좌표가 들어오면(NaN 아니면) update_lidar로 위치 보정
//   - LiDAR 없으면 순수 INS dead-reckoning
//
// LiDAR 입력은 외부(통제소 통신 스레드/콜백)에서 feed_lidar()로 주입.
// step()이 돌 때 보관된 LiDAR 값이 유효하면 소비.
//
// 사용:
//   EbimuImu imu(cfg);
//   LidarAidedIns ins(imu);
//   ins.init();
//   ins.calibrate();              // 5초 정지 (blocking)
//   while (running) {
//       // (다른 곳에서) ins.feed_lidar(lx, ly);
//       ins.step();               // IMU read + predict + (LiDAR) update
//       auto st = ins.get_state();
//   }
//
// 참조: test_ins_live.cpp (동일 흐름 + LiDAR 분기 추가)

#pragma once

#include "ebimu/sensor/iimu.hpp"
#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/domain/eskf.hpp"
#include "ebimu/domain/zupt_detector.hpp"

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>

namespace ebimu {

struct LidarAidedInsConfig {
    uint64_t calib_duration_us = 5000000ULL;  // 5초 정지 캘리브
    uint64_t warmup_us         = 2000000ULL;  // 시작 전 dummy read (노이즈 비우기)
    float    expected_g        = 9.80665f;

    // ZUPT 파라미터 (ZuptDetector 기본값과 동일)
    float    zupt_accel_tol    = 0.3f;
    float    zupt_gyro_tol     = 0.05f;
    int      zupt_min_consec   = 50;
};

class LidarAidedIns {
public:
    explicit LidarAidedIns(IImu& imu, const LidarAidedInsConfig& cfg = {})
        : imu_(imu),
          cfg_(cfg),
          calib_(cfg.calib_duration_us, cfg.expected_g),
          zupt_(cfg.zupt_accel_tol, cfg.zupt_gyro_tol, cfg.zupt_min_consec) {}

    /// IMU 초기화 + 워밍업(dummy read). 캘리브 전에 1회.
    /// @return 성공 여부
    bool init();

    /// 5초 정지 캘리브 (blocking). bias + 시작 자세 추정 후 ESKF init.
    /// @return 성공 여부
    bool calibrate();

    /// 한 사이클 진행:
    ///   IMU read → predict → ZUPT → (LiDAR 유효 시) update_lidar
    /// @return IMU 샘플을 읽어 predict 했으면 true (LiDAR는 별개)
    bool step();

    /// LiDAR 측정 좌표 주입 (통제소 통신부에서 호출).
    /// NaN을 넣으면 "측정 없음"으로 간주되어 step()에서 무시됨.
    /// thread-safe (atomic).
    void feed_lidar(float px, float py);

    // ── 상태 조회 ──
    EskfState get_state() const { return eskf_.get_state(); }

    /// 시작 자세 기준 상대 yaw (deg). 시연 컨벤션: 오른쪽 +, 왼쪽 -.
    float get_yaw_rel_deg() const { return last_yaw_rel_deg_; }

    /// 최신 IMU 샘플의 원본 yaw (deg).
    float get_raw_yaw_deg() const { return last_raw_yaw_deg_; }

    /// 가장 최근 사이클에서 LiDAR update가 적용됐는지.
    bool last_lidar_used() const { return last_lidar_used_; }

    /// 가장 최근 사이클이 ZUPT였는지.
    bool last_zupt_active() const { return last_zupt_active_; }

    bool is_calibrated() const { return calibrated_; }
    const BiasCalibration& bias() const { return bias_; }

    // ── 누적 통계 (디버그) ──
    uint32_t predict_count()      const { return predict_count_; }
    uint32_t lidar_update_count() const { return lidar_update_count_; }
    uint32_t zupt_count()         const { return zupt_count_; }

private:
    IImu&               imu_;
    LidarAidedInsConfig cfg_;

    BiasCalibrator      calib_;
    Eskf                eskf_;
    ZuptDetector        zupt_;
    BiasCalibration     bias_;

    // 시작 자세 (yaw 정렬용)
    Eigen::Quaternionf  q_start_{1.0f, 0.0f, 0.0f, 0.0f};

    // 외부 주입 LiDAR (NaN = 측정 없음). atomic으로 통신 스레드 안전.
    std::atomic<float>  lidar_x_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float>  lidar_y_{std::numeric_limits<float>::quiet_NaN()};

    // 타이밍
    uint64_t            t_prev_us_ = 0;
    bool                first_step_ = true;

    // 상태 플래그
    bool                calibrated_       = false;
    bool                last_lidar_used_  = false;
    bool                last_zupt_active_ = false;
    float               last_yaw_rel_deg_ = 0.0f;
    float               last_raw_yaw_deg_ = 0.0f;

    // 통계
    uint32_t            predict_count_      = 0;
    uint32_t            lidar_update_count_ = 0;
    uint32_t            zupt_count_         = 0;

    void warmup();
    void update_yaw(const ImuSample& s);
};

}  // namespace ebimu