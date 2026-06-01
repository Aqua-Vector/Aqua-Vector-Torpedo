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
#include "guidance/PNGuidanceController.hpp"

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

    // 기본 타겟 및 파라미터 설정
    // 출발 시 전방을 Y축(0도) 방향으로 가정하므로,
    // (X=10, Y=20) 이면 전방으로 20m, 우측으로 10m 떨어진 곳이 타겟입니다.
    float target_x = 0.0f;
    float target_y = 10.0f;
    float test_velocity = 120.0f;
    std::string imu_port = "/dev/ttyUSB0";
    std::string stm_port = "/dev/ttyPS1";
    int imu_baud = 921600; // EBIMU 기본 보드레이트 

    if (argc > 1) target_x = std::stof(argv[1]);
    if (argc > 2) target_y = std::stof(argv[2]);
    if (argc > 3) test_velocity = std::stof(argv[3]);
    if (argc > 4) imu_port = argv[4];
    if (argc > 5) stm_port = argv[5];
    if (argc > 6) imu_baud = std::stoi(argv[6]);

    std::cout << "================================================================" << std::endl;
    std::cout << "   [NEW] PN Guidance Test (Ebimu Yaw + RPS Odometry)            " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << " Target: (" << target_x << ", " << target_y << ") m" << std::endl;
    std::cout << " Velocity: " << test_velocity << std::endl;
    std::cout << " IMU Port: " << imu_port << " (@" << imu_baud << ") | STM Port: " << stm_port << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Ebimu 초기화 및 연결 체크
    torpedo::sensor::EbImuUart imu(imu_port, imu_baud);
    
    std::cout << "[CHECK] EBIMU 센서 연결 확인 중..." << std::endl;
    if (!imu.init()) {
        std::cerr << "[ERROR] EBIMU 초기화 실패! 장치 연결 및 포트 설정을 확인하세요." << std::endl;
        return -1;
    }

    // 초기 데이터 수신 확인 (연결성 검증)
    bool imu_alive = false;
    for (int i = 0; i < 50; ++i) { // 5초간 시도
        ImuSample sample;
        if (imu.read(sample)) {
            imu_alive = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!imu_alive) {
        std::cerr << "[ERROR] EBIMU 장치는 열렸으나 데이터가 들어오지 않습니다! 보드레이트를 확인하세요." << std::endl;
        return -1;
    }
    std::cout << "[OK] EBIMU 센서가 정상적으로 작동 중입니다." << std::endl;

    // 2. 초기 Yaw Offset 및 정지 상태 캘리브레이션 (5초)
    float yaw_offset = 0.0f;
    std::cout << "\n[CALIB] 정지 상태 캘리브레이션을 시작합니다 (5초). 어뢰를 움직이지 마세요..." << std::endl;
    int init_samples = 0;
    float sum_yaw = 0.0f;
    auto calib_start = std::chrono::steady_clock::now();
    int last_reported_sec = -1;
    
    while(true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - calib_start).count();
        if (elapsed >= 5.0) break;

        // 1초 단위 진행 상황 출력
        int current_sec = static_cast<int>(elapsed);
        if (current_sec != last_reported_sec) {
            std::cout << "[CALIB] 진행 중: " << (current_sec + 1) << " / 5s" << std::endl;
            last_reported_sec = current_sec;
        }

        ImuSample sample;
        if (imu.read(sample)) {
            sum_yaw += sample.yaw;
            init_samples++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (init_samples > 50) { // 최소 샘플 수 확인
        yaw_offset = sum_yaw / init_samples;
        std::cout << "[OK] 캘리브레이션 완료. 초기 Yaw Offset: " << yaw_offset * 180.0f / M_PI << " deg (" << init_samples << " 샘플)" << std::endl;
    } else {
        std::cerr << "[ERROR] 캘리브레이션 실패! 데이터 샘플이 부족합니다." << std::endl;
        return -1;
    }

    // 3. STM32 하드웨어 링크 설정 및 연결 체크
    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    FeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(PACKET_FUNC_CHASSIS_FEEDBACK, &fb_handler);

    std::cout << "[CHECK] STM32 링크(/dev/ttyPS1) 연결 확인 중..." << std::endl;
    if (!stm_manager.start()) {
        std::cerr << "[ERROR] STM32 매니저 시작 실패! 포트 설정을 확인하세요." << std::endl;
        return -1;
    }

    // STM32 피드백 수신 확인
    bool stm_alive = false;
    for (int i = 0; i < 20; ++i) { // 2초간 시도
        FeedbackPayload fb;
        uint64_t ts;
        if (stm_fb_mb.fetch(fb, ts)) {
            stm_alive = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!stm_alive) {
        std::cerr << "[ERROR] STM32 장치는 연결되었으나 피드백 데이터가 없습니다! 전원을 확인하세요." << std::endl;
        return -1;
    }
    std::cout << "[OK] STM32 피드백 수신 확인됨." << std::endl;

    // 4. 로직 컴포넌트 (트래커, 유도기, 필터) 초기화
    torpedo::domain::RpsPositionTracker rps_tracker;
    utils::LowPassFilter speed_lpf(0.2f);
    PNGuidanceController pn_controller;

    // 바퀴 반지름: 최신 캘리브레이션 반영 (33.9mm)
    const float WHEEL_RADIUS = 0.0339f; 
    const float RPS_TO_MPS = 2.0f * M_PI * WHEEL_RADIUS;

    std::cout << "\n>>> 어뢰를 타겟 방향(또는 기준 전방)으로 정렬하고 ENTER를 누르면 3초 후 시작합니다. <<<" << std::endl;
    std::string dummy;
    std::getline(std::cin, dummy);

    for (int i = 3; i > 0; --i) {
        std::cout << "[COUNTDOWN] " << i << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\n[RUNNING] Starting PN Guidance Loop..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_update_time = start_time;
    auto last_cmd_time = start_time;
    auto last_log_time = start_time;
    uint64_t last_fb_ts = 0;

    float current_speed = 0.0f;
    float current_yaw_rad = 0.0f;
    float last_valid_rudder = 0.0f;

    while (g_keep_running) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_update_time).count();
        // A. Ebimu 센서 데이터 갱신
        ImuSample sample;
        if (imu.read(sample)) {
            // [최종 수정] 사용자 정의: 오른쪽이 + (CW+)
            // EbImuUart의 yaw는 이미 parseLine에서 계산되어 나옴.
            current_yaw_rad = -(sample.yaw - yaw_offset); 

            // 각도를 -PI ~ PI 사이로 정규화
            while (current_yaw_rad > M_PI) current_yaw_rad -= 2.0f * M_PI;
            while (current_yaw_rad < -M_PI) current_yaw_rad += 2.0f * M_PI;
        }

        // B. STM32 RPS 피드백 데이터 갱신
        FeedbackPayload fb;
        uint64_t ts;
        if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
            last_fb_ts = ts;
            // V=120 전진 시 RPS가 양수(M1)와 음수(M2)로 나옴
            float raw_speed = (fb.m1_rps - fb.m2_rps) * 0.5f * RPS_TO_MPS; 
            current_speed = speed_lpf.update(raw_speed);
        }

        // [핵심] C. 위치 추정 및 PN 알고리즘 연산 (무조건 10ms 마다 독립적으로 실행)
        // state_updated 변수에 의존하지 않고 고정 주기로 실행 (Zero-Order Hold)
        if (dt >= 0.01f) {
            last_update_time = now;
            
            // 위치 추산 엔진(RpsPositionTracker)은 표준 CCW+를 사용하므로 부호 반전하여 전달
            Eigen::Quaternionf q_nav(Eigen::AngleAxisf(-current_yaw_rad, Eigen::Vector3f::UnitZ()));
            
            // 위치 업데이트 (새로운 데이터가 없어도 마지막 값을 유지하여 업데이트)
            rps_tracker.update(current_speed, q_nav, dt);

            // PN 컨트롤러를 위한 ESKF State 구성 (가짜 ESKF State)
            torpedo::domain::EskfState my_state;
            my_state.p = rps_tracker.getPosition();
            my_state.v = q_nav * Eigen::Vector3f(0.0f, current_speed, 0.0f); 
            my_state.q = q_nav;

            Eigen::Vector2f target_pos(target_x, target_y);
            Eigen::Vector2f relative_pos = target_pos - my_state.p.head<2>();
            float dist_to_target = relative_pos.norm();
            
            // [추가] 지나침 감지 로직: 현재 속도 벡터와 타겟 방향 벡터의 내적 확인
            // 내적이 음수이면 타겟을 등지고 멀어지고 있다는 뜻입니다.
            float dot_product = my_state.v.head<2>().dot(relative_pos);
            bool passed_target = (dist_to_target < 0.5f && dot_product < 0);

            float target_velocity = test_velocity;
            float rudder_cmd = 0.0f;

            // [수정] 도착 판정 로직 개선: 지나침 감지 포함
            
            // 1. 도착 판정 (10cm 이내 혹은 50cm 이내에서 멀어지기 시작할 때)
            if (dist_to_target < 0.1f || passed_target) { 
                target_velocity = 0.0f;
                rudder_cmd = 0.0f;
                if (passed_target) std::cout << "\n[INFO] Target passed (CPA reached). Stopping..." << std::endl;
            } 
            // 2. 블라인드 레인지 진입 (1.0m 이내) -> 급조향 방지
            else if (dist_to_target < 1.0f) {
                target_velocity = test_velocity;
                // 마지막 유효 조향각 유지 혹은 직진(0.0f)
                rudder_cmd = last_valid_rudder; 
            }
            // 3. 정상 PN 유도 구간
            else {
                // PN 조향각 계산 (출력은 표준 CCW+ 각도)
                float raw_pn_out = pn_controller.calculateSteering(my_state, target_pos, dt);
                // CCW+ 출력 -> 시스템 표준(CW+, 오른쪽+)으로 변환
                rudder_cmd = -raw_pn_out;
                last_valid_rudder = rudder_cmd;
            }

            // D. STM32로 제어 명령 송신 (50Hz = 20ms)
            if (std::chrono::duration<float>(now - last_cmd_time).count() >= 0.02f) {
                // 시스템 표준(CW+, 오른쪽+) -> STM32 기준(CCW+, 오른쪽-) 변환을 위해 부호 반전
                ControlPayload cmd = {target_velocity, -rudder_cmd, 0.0f};
                STMPacket pkt;
                pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
                pkt.length = sizeof(ControlPayload); pkt.payload = cmd; pkt.crc = 0;
                stm_tx_q.push(pkt);
                last_cmd_time = now;
            }

            // E. 로깅 (10Hz)
            if (std::chrono::duration<float>(now - last_log_time).count() >= 0.1f) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "\r[PN] Pos: (" << std::setw(5) << my_state.p.x() << ", " << std::setw(5) << my_state.p.y() << ")"
                          << " | Yaw: " << std::setw(6) << current_yaw_rad * 180.0f / M_PI << " deg"
                          << " | Spd: " << std::setw(5) << current_speed << " m/s"
                          << " | Cmd_R: " << std::setw(6) << rudder_cmd
                          << " | Dist: " << std::setw(5) << dist_to_target << "m " << std::flush;
                last_log_time = now;
            }
            
            // 종료 조건 처리
            if (dist_to_target < 0.1f && target_velocity == 0.0f) {
                std::cout << "\n\n[Target Reached] Test completed successfully!" << std::endl;
                g_keep_running = false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 종료 시 모터 정지
    std::cout << "\n[STOP] Stopping motors..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        ControlPayload stop_cmd = {0.0f, 0.0f, 0.0f};
        STMPacket pkt;
        pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
        pkt.length = sizeof(ControlPayload); pkt.payload = stop_cmd; pkt.crc = 0;
        stm_tx_q.push(pkt);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    stm_manager.stop();
    return 0;
}
