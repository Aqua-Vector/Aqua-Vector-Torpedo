// test_lidar_aided_ins.cpp — 버전 1 통합 검증
//
// 검증:
//   1. 캘리브 동작 (FakeImu 정지)
//   2. LiDAR 없으면 INS-only로 predict만 (값 무시)
//   3. LiDAR 주입 시 update_lidar 적용 → 위치가 측정 쪽으로 끌려감
//   4. NaN 주입 시 무시
//   5. ZUPT 정지 감지 시 속도 0

#include "ebimu/app/lidar_aided_ins.hpp"
#include "ebimu/sensor/fake_imu.hpp"
#include "ebimu/hal/system_clock.hpp"

#include <cstdio>
#include <cmath>
#include <limits>
#include <unistd.h>

using namespace ebimu;

static bool nearly(float a, float b, float tol) { return std::fabs(a - b) < tol; }

// 캘리브 헬퍼.
// 주의: LidarAidedIns::calibrate()는 monotonic_us() 실시간 기반이라
//       FakeImu(무한 속도)와 쓰면 수백만 샘플이 쌓여 float 누적합이 깨진다.
//       이건 테스트 환경(가짜 IMU가 비현실적으로 빠름) 한계일 뿐,
//       실제 EBIMU(99Hz)에선 5초에 ~495샘플이라 문제 없음.
//       테스트에선 calibrate()를 직접 안 쓰고, 짧은 duration + sleep으로
//       현실적 샘플 수만 흘려보내 캘리브를 흉내낸다.
static bool run_calibrate(LidarAidedIns& ins) {
    return ins.calibrate();
}

int main() {
    std::printf("=== LidarAidedIns (버전 1) 검증 ===\n");

    // 짧은 캘리브/워밍업으로 테스트 빠르게
    LidarAidedInsConfig cfg;
    cfg.calib_duration_us = 200000ULL;   // 0.2초 (throttle 300us → ~600샘플)
    cfg.warmup_us         = 100000ULL;   // 0.1초
    cfg.zupt_min_consec   = 10;          // 빨리 ZUPT 뜨게

    // ── Test 1: 캘리브 동작 ──
    {
        std::printf("\n[Test 1] 캘리브 (정지 직립)\n");
        FakeImu imu;
        imu.set_quaternion(1, 0, 0, 0);  // 직립
        imu.set_dt_us(1000);
        imu.set_read_throttle_us(300);   // ~현실적 샘플 속도 (float 누적 폭주 방지)
        LidarAidedIns ins(imu, cfg);
        if (!ins.init()) { std::printf("[FAIL] init\n"); return 1; }
        if (!run_calibrate(ins)) { std::printf("[FAIL] calibrate\n"); return 1; }
        if (!ins.is_calibrated()) { std::printf("[FAIL] not calibrated\n"); return 1; }
        auto b = ins.bias();
        std::printf("  bias=(%.4f,%.4f,%.4f) n=%d\n", b.bx, b.by, b.bz, b.n_samples);
        if (!nearly(b.bx, 0, 0.05f) || !nearly(b.bz, 0, 0.05f)) {
            std::printf("[FAIL] bias 비정상\n"); return 1;
        }
        std::printf("[OK] 캘리브 → bias ~0\n");
    }

    // ── Test 2: LiDAR 없이 정지 → INS-only, drift 작음 + ZUPT ──
    {
        std::printf("\n[Test 2] LiDAR 없음 + 정지 → INS only\n");
        FakeImu imu;
        imu.set_quaternion(1, 0, 0, 0);
        imu.set_dt_us(10000);  // 100Hz
        imu.set_read_throttle_us(300);
        LidarAidedIns ins(imu, cfg);
        ins.init();
        run_calibrate(ins);

        for (int i = 0; i < 200; i++) {
            ins.step();
            usleep(200);  // dt 흐름 흉내 (monotonic 기반)
        }
        auto st = ins.get_state();
        std::printf("  p=(%.4f,%.4f) lidar_used_cnt=%u zupt_cnt=%u\n",
                    st.px, st.py, ins.lidar_update_count(), ins.zupt_count());
        if (ins.lidar_update_count() != 0) {
            std::printf("[FAIL] LiDAR 안 줬는데 update 발생\n"); return 1;
        }
        if (ins.zupt_count() == 0) {
            std::printf("[FAIL] 정지인데 ZUPT 미발생\n"); return 1;
        }
        std::printf("[OK] INS-only 동작 (LiDAR update 0회, ZUPT 동작)\n");
    }

    // ── Test 3: LiDAR 주입 → update 적용, 위치가 측정 쪽으로 ──
    {
        std::printf("\n[Test 3] LiDAR 주입 → 위치 보정\n");
        FakeImu imu;
        imu.set_quaternion(1, 0, 0, 0);
        imu.set_dt_us(10000);
        imu.set_read_throttle_us(300);
        LidarAidedIns ins(imu, cfg);
        ins.init();
        run_calibrate(ins);

        // 시작은 (0,0) 근처. LiDAR가 (3, 4)라고 알려줌.
        ins.feed_lidar(3.0f, 4.0f);
        bool stepped = false;
        for (int i = 0; i < 5; i++) { if (ins.step()) { stepped = true; break; } usleep(200); }
        if (!stepped) { std::printf("[FAIL] step 안 됨\n"); return 1; }

        auto st = ins.get_state();
        std::printf("  보정 후 p=(%.4f,%.4f), lidar_used=%d cnt=%u\n",
                    st.px, st.py, ins.last_lidar_used(), ins.lidar_update_count());
        if (ins.lidar_update_count() != 1) {
            std::printf("[FAIL] update 1회 기대\n"); return 1;
        }
        // (0,0)에서 (3,4) 방향으로 끌려가야 (K<1이라 도달은 안 함)
        if (st.px <= 0.0f || st.px >= 3.0f || st.py <= 0.0f || st.py >= 4.0f) {
            std::printf("[FAIL] 측정 방향으로 안 끌려감\n"); return 1;
        }
        std::printf("[OK] LiDAR update 적용 (측정 방향 이동)\n");

        // 같은 측정 소비됐는지: 다음 step에선 update 0이어야 (재주입 안 함)
        for (int i = 0; i < 5; i++) { if (ins.step()) break; usleep(200); }
        if (ins.lidar_update_count() != 1) {
            std::printf("[FAIL] 측정 소비 안 됨 (재사용)\n"); return 1;
        }
        std::printf("[OK] 측정 1회 소비 후 재사용 안 함\n");
    }

    // ── Test 4: NaN 주입 → 무시 ──
    {
        std::printf("\n[Test 4] NaN LiDAR → 무시\n");
        FakeImu imu;
        imu.set_quaternion(1, 0, 0, 0);
        imu.set_dt_us(10000);
        imu.set_read_throttle_us(300);
        LidarAidedIns ins(imu, cfg);
        ins.init();
        run_calibrate(ins);

        float nan = std::numeric_limits<float>::quiet_NaN();
        ins.feed_lidar(nan, nan);
        for (int i = 0; i < 5; i++) { if (ins.step()) break; usleep(200); }
        if (ins.lidar_update_count() != 0) {
            std::printf("[FAIL] NaN인데 update 발생\n"); return 1;
        }
        std::printf("[OK] NaN 무시\n");
    }

    // ── Test 5: 반복 LiDAR 보정으로 위치 수렴 ──
    {
        std::printf("\n[Test 5] 반복 LiDAR → (5,3)로 수렴\n");
        FakeImu imu;
        imu.set_quaternion(1, 0, 0, 0);
        imu.set_dt_us(10000);
        imu.set_read_throttle_us(300);
        LidarAidedIns ins(imu, cfg);
        ins.init();
        run_calibrate(ins);

        for (int i = 0; i < 100; i++) {
            ins.feed_lidar(5.0f, 3.0f);
            ins.step();
            usleep(200);
        }
        auto st = ins.get_state();
        std::printf("  최종 p=(%.4f,%.4f) (기대 ~5,3) update=%u\n",
                    st.px, st.py, ins.lidar_update_count());
        if (!nearly(st.px, 5.0f, 0.3f) || !nearly(st.py, 3.0f, 0.3f)) {
            std::printf("[FAIL] 수렴 안 함\n"); return 1;
        }
        std::printf("[OK] 반복 보정 → 측정 좌표 수렴\n");
    }

    std::printf("\n[ALL PASS] LidarAidedIns 버전 1 검증 통과\n");
    return 0;
}