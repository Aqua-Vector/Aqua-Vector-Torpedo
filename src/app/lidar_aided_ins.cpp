#include "ebimu/app/lidar_aided_ins.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cmath>
#include <limits>
#include <unistd.h>

namespace ebimu {

namespace {
// quaternion → yaw(deg). 시연 컨벤션(오른쪽 +, 왼쪽 -) = test_ins_live와 동일.
float quat_to_yaw_deg(float w, float x, float y, float z) {
    float yaw_rad = std::atan2(2.0f * (w * z + x * y),
                               1.0f - 2.0f * (y * y + z * z));
    return -yaw_rad * 180.0f / 3.14159265f;
}
}  // namespace

bool LidarAidedIns::init() {
    if (!imu_.init()) {
        return false;
    }
    warmup();
    return true;
}

void LidarAidedIns::warmup() {
    // 시작 직후 노이즈 sample 비우기 (test_ins_live와 동일 패턴)
    uint64_t t_end = monotonic_us() + cfg_.warmup_us;
    while (monotonic_us() < t_end) {
        ImuSample s;
        if (!imu_.read_sample(s)) {
            usleep(1000);
        }
    }
}

bool LidarAidedIns::calibrate() {
    imu_.flush(); // 캘리브레이션 시작 전 버퍼 비우기 (최신 데이터 확보)
    calib_.start();
    uint64_t start_t = monotonic_us();
    int last_sec = -1;

    std::printf("[LidarAidedIns] Starting 5s calibration...\n");

    while (calib_.is_collecting()) {
        ImuSample s;
        if (!imu_.read_sample(s)) {
            usleep(1000);
            
            // 데이터가 아예 안 들어올 경우를 대비한 타임아웃 (10초)
            if (monotonic_us() - start_t > 10000000ULL) {
                std::printf("[LidarAidedIns] ERROR: Calibration Timeout! No data from IMU.\n");
                return false;
            }
            continue;
        }
        calib_.add_sample(s);

        // 1초마다 진행 상황 출력
        int current_sec = static_cast<int>((monotonic_us() - start_t) / 1000000ULL);
        if (current_sec != last_sec) {
            std::printf("[LidarAidedIns] Calibrating... %d/5s (Progress: %.0f%%)\n", 
                        current_sec + 1, calib_.progress() * 100.0f);
            last_sec = current_sec;
        }
    }

    bias_ = calib_.finalize();
    if (!bias_.valid) {
        std::printf("[LidarAidedIns] ERROR: Calibration Finalize Failed (Samples: %d)\n", bias_.n_samples);
        calibrated_ = false;
        return false;
    }

    // 시작 자세 보관 (상대 yaw 출력용)
    q_start_ = Eigen::Quaternionf(bias_.q_start_w, bias_.q_start_x,
                                  bias_.q_start_y, bias_.q_start_z);
    q_start_.normalize();

    // 시작 Yaw 확인 (디버깅)
    float yaw0 = quat_to_yaw_deg(q_start_.w(), q_start_.x(), q_start_.y(), q_start_.z());
    std::printf("[LidarAidedIns] Calibration Successful! Samples: %d, Start Yaw: %.2f deg\n", 
                bias_.n_samples, yaw0);

    // ESKF 초기화 — 시작 위치 (0,0,0), 정지
    eskf_.init();

    calibrated_ = true;
    first_step_ = true;
    return true;
}

void LidarAidedIns::feed_lidar(float px, float py) {
    // NaN을 넣으면 step()에서 자동으로 무시됨 (Eskf::update_lidar도 NaN 거부)
    lidar_x_.store(px, std::memory_order_relaxed);
    lidar_y_.store(py, std::memory_order_relaxed);
}

void LidarAidedIns::update_yaw(const ImuSample& s) {
    Eigen::Quaternionf q_abs(s.qw, s.qx, s.qy, s.qz);
    q_abs.normalize();
    Eigen::Quaternionf q_rel = q_start_.conjugate() * q_abs;
    last_yaw_rel_deg_ = quat_to_yaw_deg(q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z());
}

bool LidarAidedIns::step() {
    last_lidar_used_  = false;
    last_zupt_active_ = false;

    if (!calibrated_) {
        return false;  // 캘리브 안 됐으면 진행 X
    }

    ImuSample s;
    if (!imu_.read_sample(s)) {
        return false;  // 이번엔 새 IMU 샘플 없음
    }

    // ── dt 계산 ──
    uint64_t t_now = monotonic_us();
    float dt;
    if (first_step_) {
        dt = 0.01f;  // 첫 사이클은 명목 dt (100Hz 가정)
        first_step_ = false;
    } else {
        dt = static_cast<float>(t_now - t_prev_us_) / 1000000.0f;
    }
    t_prev_us_ = t_now;

    // ── 1. INS predict (항상) ──
    eskf_.predict(s, bias_, dt);
    predict_count_++;

    // ── 2. ZUPT 정지 보정 ──
    last_zupt_active_ = zupt_.update(s);
    if (last_zupt_active_) {
        eskf_.update_zupt();
        zupt_count_++;
    }

    // ── 3. LiDAR 위치 보정 (값 있고 NaN 아니면) ──
    float lx = lidar_x_.load(std::memory_order_relaxed);
    float ly = lidar_y_.load(std::memory_order_relaxed);
    if (!std::isnan(lx) && !std::isnan(ly)) {
        if (eskf_.update_lidar(lx, ly)) {
            last_lidar_used_ = true;
            lidar_update_count_++;
        }
        // 소비 처리: 같은 측정을 다음 사이클에 또 쓰지 않도록 NaN으로 리셋.
        // (통제소가 새 값 주면 다시 채워짐)
        lidar_x_.store(std::numeric_limits<float>::quiet_NaN(), std::memory_order_relaxed);
        lidar_y_.store(std::numeric_limits<float>::quiet_NaN(), std::memory_order_relaxed);
    }

    // ── 4. 자세(yaw) 갱신 ──
    update_yaw(s);

    return true;
}

}  // namespace ebimu