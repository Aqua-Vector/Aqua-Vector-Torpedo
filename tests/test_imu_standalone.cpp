// test_imu_standalone.cpp
#include "torpedo/sensor/ism330dhcx_imu.hpp"
#include "torpedo/sensor/imu_downsampler.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/bias_calibrator.hpp"
#include "torpedo/hal/system_clock.hpp"

#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>

static volatile bool g_running = true;
static void on_signal(int) { g_running = false; }

int main(int argc, char** argv) {
    const char* spi_dev = argc > 1 ? argv[1] : "/dev/spidev0.0";

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ------- 1. IMU 초기화 -------
    torpedo::Ism330dhcxImu imu;
    torpedo::Ism330dhcxConfig imu_cfg;
    imu_cfg.spi_device   = spi_dev;
    imu_cfg.spi_speed_hz = 1000000;
    imu.set_config(imu_cfg);

    if (!imu.init()) {
        std::fprintf(stderr, "[ERROR] IMU init 실패: %s\n", spi_dev);
        return 1;
    }
    std::printf("[OK] IMU 초기화: %s\n\n", spi_dev);

    // ------- 2. 캘리브레이션 (5초) -------
    std::printf("[CALIB] 보드를 평평하게 놓고 대기 (5초)...\n");

    torpedo::domain::BiasCalibrator cal;
    cal.start(5.0f, 833);

    while (!cal.is_done() && g_running) {
        torpedo::ImuSample s;
        if (imu.read(s)) cal.add_sample(s);
        usleep(300);
    }

    torpedo::domain::CalibrationResult cal_result = cal.finalize();
    if (!cal_result.success) {
        std::fprintf(stderr, "[WARN] 캘리브 실패, bias=0 진행\n");
    } else {
        std::printf("[CALIB] 완료 (%d 샘플)\n", cal_result.samples_used);
        std::printf("  roll  = %+.2f deg\n", cal_result.roll_rad  * 57.2958f);
        std::printf("  pitch = %+.2f deg\n", cal_result.pitch_rad * 57.2958f);
        std::printf("  b_a   = (%+.4f, %+.4f, %+.4f) m/s²\n",
            cal_result.bias.b_a.x(),
            cal_result.bias.b_a.y(),
            cal_result.bias.b_a.z());
        std::printf("  b_g   = (%+.5f, %+.5f, %+.5f) rad/s\n\n",
            cal_result.bias.b_g.x(),
            cal_result.bias.b_g.y(),
            cal_result.bias.b_g.z());
    }

    // 캘리브 완료 후 추가
    std::printf("\n[RAW ACCEL 평균]\n");
    std::printf("  ax=%.4f ay=%.4f az=%.4f m/s²\n",
        cal_result.bias.b_a.x() + 0.0f,  // 임시
        cal_result.bias.b_a.y() + 0.0f,
        cal_result.bias.b_a.z() + 9.81f  // bias + 중력
    );

    // ------- 3. ESKF 초기화 -------
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams eskf_params;
    eskf_params.p0 = Eigen::Vector3f::Zero();
    eskf_params.q0 = cal_result.success
                   ? cal_result.q0
                   : Eigen::Quaternionf::Identity();
    eskf.init(eskf_params, 0.01f);

    torpedo::domain::BiasEstimate bias = cal_result.success
                                       ? cal_result.bias
                                       : torpedo::domain::BiasEstimate{};

    // ------- 4. 메인 루프 -------
    torpedo::SystemClock    clock;
    torpedo::ImuDownsampler downsampler;

    constexpr int   PERIOD_US   = 10000;
    constexpr int   PRINT_EVERY = 10;     // 10Hz 출력
    constexpr float DT          = 0.01f;

    uint64_t t0         = clock.now_us();
    int      loop_count = 0;
    int      imu_ok     = 0;

    std::printf("Ctrl+C로 종료\n");
    std::printf("-----------------------------------\n");
    std::printf("%-7s  %-8s %-8s  %-9s\n",
        "t(s)", "px(m)", "py(m)", "yaw(deg)");
    std::printf("-----------------------------------\n");

    while (g_running) {
        uint64_t cycle_start = clock.now_us();

        // 8 sub-samples 수집
        downsampler.reset();
        for (int i = 0; i < 8; i++) {
            torpedo::ImuSample s;
            for (int retry = 0; retry < 5; retry++) {
                if (imu.read(s)) break;
                usleep(200);
            }
            if (s.valid) downsampler.add(s);
        }

        if (downsampler.is_full()) {
            torpedo::ImuSample filtered = downsampler.finalize();
            if (filtered.valid) {
                eskf.predict(filtered, bias, DT);
                imu_ok++;

                float ax = filtered.ax - bias.b_a.x();
                float ay = filtered.ay - bias.b_a.y();
                float az = filtered.az - bias.b_a.z();
                float a_mag = std::sqrt(ax*ax + ay*ay + az*az);

                float w_mag = std::sqrt(
                    filtered.gx * filtered.gx +
                    filtered.gy * filtered.gy +
                    filtered.gz * filtered.gz
                );

                // if (std::abs(a_mag - 9.9974f) < 0.15f && w_mag < 0.05f) {
                //     eskf.update_attitude_accel(ax, ay, az, 0.02f);
                // }
            }
        }

        loop_count++;

        if (loop_count % PRINT_EVERY == 0) {
            const auto& x = eskf.state();

            float t_sec   = (float)(clock.now_us() - t0) / 1e6f;
            Eigen::Quaternionf q = x.q;
            float yaw_rad = std::atan2(
                2.0f * (q.w() * q.z() + q.x() * q.y()),
                1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())
            );
            float yaw_deg = -yaw_rad * 57.2958f;  // 부호 반전

            std::printf("%-7.2f  %-8.3f %-8.3f  %+.2f\n",
                t_sec,
                x.p.x(),
                x.p.y(),
                yaw_deg);
        }

        uint64_t elapsed = clock.now_us() - cycle_start;
        if (elapsed < (uint64_t)PERIOD_US) {
            usleep((unsigned)(PERIOD_US - elapsed));
        }
    }

    std::printf("-----------------------------------\n");
    std::printf("[DONE] loops=%d  imu_ok=%d\n", loop_count, imu_ok);
    imu.shutdown();
    return 0;
}