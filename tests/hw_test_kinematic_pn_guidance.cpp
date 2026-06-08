#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <csignal>
#include <atomic>

#include <Eigen/Dense>

#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "core/NetworkManager.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/GenericPacket.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/Mailbox.hpp"
#include "utils/TimeUtils.hpp"
#include "utils/LowPassFilter.hpp"

// Ebimu & Estimator
#include "guidance/EbImuUart.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "torpedo/domain/estimator/eskf_state.hpp"

// Guidance
#include "guidance/IGuidanceController.hpp"
#include "guidance/HybridGuidanceController.hpp"
#include "guidance/TargetStateEstimator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

using STMPacket = GenericPacket<ControlPayload, uint8_t>;
using ImuSample = torpedo::ImuSample;

std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief STM32 피드백 패킷 핸들러
 */
class FeedbackHandler : public IMessageHandler {
public:
    Mailbox<FeedbackPayload>& mb;
    explicit FeedbackHandler(Mailbox<FeedbackPayload>& mailbox) : mb(mailbox) {}

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(FeedbackPayload)) return false;
        FeedbackPayload data;
        Marshaller::deserialize(payload, length, data);
        mb.update(data, timestamp_ms);
        return true;
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    // 기본 파라미터 설정
    float target_x = 0.0f;
    float target_y = 10.0f;
    float test_velocity = 120.0f;
    std::string imu_port = "/dev/ttyUSB0";
    std::string stm_port = "/dev/ttyPS1";

    if (argc > 1) target_x = std::stof(argv[1]);
    if (argc > 2) target_y = std::stof(argv[2]);
    if (argc > 3) test_velocity = std::stof(argv[3]);
    if (argc > 4) imu_port = argv[4];
    if (argc > 5) stm_port = argv[5];

    std::cout << "================================================================" << std::endl;
    std::cout << "   [ADVANCED] Kinematic PN Guidance Test (DR Target)            " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << " Target Start: (" << target_x << ", " << target_y << ") m" << std::endl;
    std::cout << " Velocity: " << test_velocity << std::endl;
    std::cout << " IMU: " << imu_port << " | STM: " << stm_port << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. 센서 및 하드웨어 초기화
    torpedo::sensor::EbImuUart imu(imu_port, 921600);
    if (!imu.init()) { std::cerr << "IMU Init Fail" << std::endl; return -1; }

    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    FeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &fb_handler);
    if (!stm_manager.start()) { std::cerr << "STM32 Link Fail" << std::endl; return -1; }

    // 2. 초기 캘리브레이션 (5초)
    std::cout << "[CALIB] 어뢰를 정렬하세요..." << std::endl;
    float yaw_offset = 0.0f;
    float sum_yaw = 0.0f;
    int samples = 0;
    uint64_t last_cal_t = 0;
    auto start_cal = std::chrono::steady_clock::now();
    while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_cal).count() < 5) {
        ImuSample s;
        if (imu.read(s) && s.t_us != last_cal_t) { 
            sum_yaw += s.yaw; 
            samples++; 
            last_cal_t = s.t_us;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    yaw_offset = (samples > 0) ? (sum_yaw / (float)samples) : 0.0f;
    std::cout << "[OK] Offset: " << yaw_offset * 180.0f / M_PI << " deg" << std::endl;

    // 3. 로직 컴포넌트 초기화
    torpedo::domain::RpsPositionTracker rps_tracker;
    guidance::TargetStateEstimator target_estimator;
    std::unique_ptr<guidance::IGuidanceController> guidance_controller = std::make_unique<guidance::HybridGuidanceController>();
    utils::LowPassFilter speed_lpf(0.2f);

    const float RPS_TO_MPS = 2.0f * M_PI * 0.0332f; // [CALIB] 0.0339 -> 0.0332 (2% error correction)

    // 타겟 초기 좌표 주입 (속도는 0으로 초기화됨)
    target_estimator.updateFromLidar(Eigen::Vector2f(target_x, target_y), 0.1f);

    std::cout << "\n>>> Press ENTER to START Guidance <<<" << std::endl;
    std::string dummy; std::getline(std::cin, dummy);

    auto last_time = std::chrono::steady_clock::now();
    uint64_t last_fb_ts = 0;
    uint64_t last_imu_t = 0;
    float current_speed = 0.0f;
    float current_yaw = 0.0f;

    while (g_keep_running) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();

        // 10ms (100Hz) 정밀 루프
        if (dt >= 0.01f) {
            last_time = now;

            // A. 나의 상태 업데이트
            ImuSample imu_s;
            if (imu.read(imu_s) && imu_s.t_us != last_imu_t) {
                // [Fix] 마이너스(-) 추가: 오른쪽 회전 시 (+)가 되도록 하여 내부 표준(CW+) 확립
                current_yaw = -(imu_s.yaw - yaw_offset);
                last_imu_t = imu_s.t_us;
                
                // Normalize
                while (current_yaw > M_PI) current_yaw -= 2.0f * M_PI;
                while (current_yaw < -M_PI) current_yaw += 2.0f * M_PI;
            }

            FeedbackPayload fb; uint64_t ts;
            if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
                last_fb_ts = ts;
                // [Fix] 속도 계산: 두 바퀴 회전량 합산 (0.1m/s 방지)
                current_speed = speed_lpf.update((std::abs(fb.m1_rps) + std::abs(fb.m2_rps)) * 0.5f * RPS_TO_MPS);
            }

            // [Fix] 마이너스(-) 추가: CW+ 각도를 Eigen(CCW+)용으로 번역하여 X(+) 좌표 보장
            Eigen::Quaternionf q_nav(Eigen::AngleAxisf(-current_yaw, Eigen::Vector3f::UnitZ()));
            rps_tracker.update(current_speed, q_nav, dt);

            torpedo::domain::EskfState my_state;
            my_state.p = rps_tracker.getPosition();
            my_state.v = q_nav * Eigen::Vector3f(0.0f, current_speed, 0.0f);
            my_state.q = q_nav;

            // B. 타겟 상태 업데이트 (Blind Phase 예측)
            // Lidar 데이터가 없는 테스트이므로 속도는 0으로 유지되며, 
            // 만약 타겟 속도가 있었다면 이 단계에서 가상 좌표가 생성됨
            target_estimator.predictVirtualState(dt);
            Eigen::Vector2f virtual_target_pos = target_estimator.getState().pos;

            // C. PN 유도 알고리즘 실행
            float dist = (virtual_target_pos - my_state.p.head<2>()).norm();
            float rudder_cmd_cw = 0.0f; // 내부 표준 (오른쪽이 양수)
            float target_velocity = test_velocity;

            // 요격 판정 (0.1m)
            if (dist < 0.1f) {
                target_velocity = 0.0f;
                std::cout << "\n[SUCCESS] Target Intercepted!" << std::endl;
                g_keep_running = false;
            } else {
                // Hybrid 유도 알고리즘 출력은 CW+ (오른쪽이 양수)
                rudder_cmd_cw = guidance_controller->calculateSteering(my_state, virtual_target_pos, dt);
            }

            // D. STM32 송신
            // [중요] 내부 표준(CW+) -> STM32 표준(CCW+)으로 변환하기 위해 단 한 번만 반전 (-)
            ControlPayload cmd = {target_velocity, -rudder_cmd_cw, 0.0f};
            STMPacket pkt;
            pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
            pkt.length = sizeof(ControlPayload); pkt.payload = cmd;
            stm_tx_q.push(pkt);

            // E. 로깅 (10Hz)
            static int log_cnt = 0;
            if (++log_cnt >= 10) {
                std::cout << std::fixed << std::setprecision(2)
                          << "\r[HYBRID PN] MyPos: (" << std::setw(5) << my_state.p.x() << ", " << std::setw(5) << my_state.p.y() << ")"
                          << " | TgtPos: (" << std::setw(5) << virtual_target_pos.x() << ", " << std::setw(5) << virtual_target_pos.y() << ")"
                          << " | R_CW: " << std::setw(6) << rudder_cmd_cw
                          << " | Dist: " << std::setw(5) << dist << "m" << std::flush;
                log_cnt = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 종료 시 정지
    for(int i=0; i<5; ++i) {
        ControlPayload stop = {0,0,0};
        STMPacket pkt; pkt.header[0]=0xAA; pkt.header[1]=0x55; pkt.msg_id=PACKET_FUNC_CHASSIS_CTRL;
        pkt.length=sizeof(ControlPayload); pkt.payload=stop;
        stm_tx_q.push(pkt); std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stm_manager.stop();
    return 0;
}
