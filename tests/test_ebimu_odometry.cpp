#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Dense>

#include "guidance/EbImuUart.hpp"
#include "utils/LowPassFilter.hpp"
#include "utils/Mailbox.hpp"
#include "utils/StaticRingBuffer.hpp"

// 통신 관련
#include "UartLink.hpp"
#include "STMControlParser.hpp"
#include "NetworkManager.hpp"
#include "Payloads.hpp"
#include "GenericPacket.hpp"
#include "Marshaller.hpp"

using namespace torpedo;
using namespace torpedo::sensor;

using STMPacket = GenericPacket<ControlPayload, uint8_t>;

// STM32 피드백 핸들러 (RPS 수신용)
class TestFeedbackHandler : public IMessageHandler {
public:
    Mailbox<FeedbackPayload>& mb;
    explicit TestFeedbackHandler(Mailbox<FeedbackPayload>& mailbox) : mb(mailbox) {}

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(FeedbackPayload)) return false;
        FeedbackPayload data;
        Marshaller::deserialize(payload, length, data);
        mb.update(data, timestamp_ms);
        return true;
    }
};

int main(int argc, char** argv) {
    std::string imu_port = "/dev/ttyUSB0";
    std::string stm_port = "/dev/ttyPS1"; // Zynq UART PS1 (상황에 따라 /dev/ttyS1 등 확인 필요)
    
    if (argc > 1) imu_port = argv[1];
    if (argc > 2) stm_port = argv[2];

    std::cout << "========== EBIMU + STM32 Odometry Tracking Test ==========" << std::endl;
    
    // 1. 하드웨어 초기화
    EbImuUart imu(imu_port, 921600); // 사용자가 설정한 보드레이트 921600
    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    TestFeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(0x0B, &fb_handler); // Feedback ID 0x0B

    if (!imu.init()) {
        std::cerr << "EBIMU Init Failed on " << imu_port << std::endl;
        return -1;
    }
    if (!stm_manager.start()) {
        std::cerr << "STM32 Link Init Failed on " << stm_port << std::endl;
        return -1;
    }

    // 2. 파라미터 및 변수 설정
    const float WHEEL_RADIUS = 0.0325f; // Measured diameter 65mm -> radius 32.5mm
    const float RPS_TO_MPS = 2.0f * M_PI * WHEEL_RADIUS;
    
    float x = 0.0f, y = 0.0f;     // 위치 (m)
    float current_yaw = 0.0f;    // 현재 헤딩 (rad)
    float current_speed = 0.0f;  // 현재 속도 (m/s)
    
    utils::LowPassFilter speed_lpf(0.2f);
    
    auto start_time = std::chrono::steady_clock::now();
    auto last_update_time = start_time;
    auto last_log_time = start_time;
    auto last_cmd_time = start_time;
    float dt_accumulator = 0.0f; // Accumulate dt until next sensor update

    std::cout << "\nStarting Tracking Loop..." << std::endl;
    std::cout << "  Time [s] | RPS (M1/M2) | Speed [m/s] | Yaw [deg] | Pos X [m] | Pos Y [m]" << std::endl;
    std::cout << "----------------------------------------------------------------------------" << std::endl;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_update_time).count();
        last_update_time = now;
        dt_accumulator += dt;

        // [Directive] STM32 제어 명령 송신 (100Hz) - 속도 50 전송
        if (now - last_cmd_time >= std::chrono::milliseconds(10)) {
            ControlPayload cmd = {50.0f, 0.0f, 0.0f}; // 속도 50, 조향 0
            STMPacket pkt;
            pkt.header[0] = 0xAA;
            pkt.header[1] = 0x55;
            pkt.msg_id = 0x0A; // Control ID
            pkt.length = sizeof(ControlPayload);
            pkt.payload = cmd;
            pkt.crc = 0; 
            stm_tx_q.push(pkt);
            last_cmd_time = now;
        }

        // EBIMU에서 최신 헤딩(Yaw) 읽기
        torpedo::ImuSample imu_sample;
        if (imu.read(imu_sample)) {
            current_yaw = imu_sample.yaw;
        }

        // STM32에서 RPS 피드백 읽기 및 위치 업데이트
        FeedbackPayload fb;
        uint64_t ts;
        static uint64_t last_fb_ts = 0;
        if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
            last_fb_ts = ts;
            
            // 속도 계산 (M1, M2 평균 RPS 기반)
            // [수정] 커맨드 -60f가 전진이므로, RPS 결과에 -를 붙여 양수 속도로 변환
            float avg_rps = -(fb.m1_rps - fb.m2_rps) * 0.5f; 
            float raw_speed = avg_rps * RPS_TO_MPS;
            current_speed = speed_lpf.update(raw_speed);
            
            // [Odometry Core] 위치 업데이트 (지점 항법)
            // 모아둔 dt_accumulator를 사용하여 실제 이동 거리를 정확히 계산
            float distance = current_speed * dt_accumulator;
            x += distance * std::cos(current_yaw);
            y += distance * std::sin(current_yaw);
            
            dt_accumulator = 0.0f; // Reset after integration
        }

        // 1초마다 로그 출력
        if (now - last_log_time >= std::chrono::seconds(1)) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(10) << (double)elapsed << " | "
                      << std::setw(5) << fb.m1_rps << "/" << std::setw(5) << fb.m2_rps << " | "
                      << std::setw(11) << current_speed << " | "
                      << std::setw(9) << current_yaw * 180.0f / M_PI << " | "
                      << std::setw(9) << x << " | "
                      << std::setw(9) << y << std::endl;
            
            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    stm_manager.stop();
    imu.shutdown();
    return 0;
}
