#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>
#include <cmath>
#include <csignal>
#include <atomic>

#include "communication/UartLink.hpp"
#include "control/STMControlParser.hpp"
#include "core/NetworkManager.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/GenericPacket.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/Mailbox.hpp"
#include "utils/TimeUtils.hpp"

using STMPacket = GenericPacket<ControlPayload, uint8_t>;

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

/**
 * @brief 특정 속도로 일정 시간 동안 이동하며 RPS 데이터를 수집하는 함수
 */
void run_test(float target_velocity, float duration_sec, Mailbox<FeedbackPayload>& stm_fb_mb, StaticRingBuffer<STMPacket, 64>& stm_tx_q) {
    if (!g_keep_running) return;

    std::cout << "\n====================================================" << std::endl;
    std::cout << " [TEST] 속도 명령: " << target_velocity << " | 측정 시간: " << duration_sec << "초" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << ">>> 어뢰를 출발선에 정렬하고 ENTER를 누르면 3초 카운트다운을 시작합니다. <<<" << std::endl;
    
    std::string dummy;
    std::getline(std::cin, dummy);

    for (int i = 3; i > 0; --i) {
        std::cout << "[COUNTDOWN] " << i << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::vector<float> m1_rps_history;
    std::vector<float> m2_rps_history;
    std::vector<float> combined_rps_history;

    // 물리 상수 (TorpedoControlSystem 설정값 유지)
    const float WHEEL_RADIUS = 0.0325f; 
    const float RPS_TO_MPS = 2.0f * 3.14159265f * WHEEL_RADIUS;

    float estimated_distance = 0.0f;
    uint64_t last_fb_ts = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_cmd_time = start_time;
    auto last_update_time = start_time;

    std::cout << "[RUNNING] 이동 중... (V=" << target_velocity << ")" << std::endl;

    while (g_keep_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - start_time).count();

        if (elapsed_sec >= duration_sec) break;

        // 제어 명령 송신 @ 50Hz
        if (now - last_cmd_time >= std::chrono::milliseconds(20)) {
            ControlPayload cmd = {target_velocity, 0.0f, 0.0f};
            STMPacket pkt;
            pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = 0x0A;
            pkt.length = sizeof(ControlPayload); pkt.payload = cmd; pkt.crc = 0;
            stm_tx_q.push(pkt);
            last_cmd_time = now;
        }

        // 피드백 수신 및 데이터 저장
        FeedbackPayload fb;
        uint64_t ts;
        if (stm_fb_mb.fetch(fb, ts) && ts != last_fb_ts) {
            float dt = std::chrono::duration<float>(now - last_update_time).count();
            last_update_time = now;
            last_fb_ts = ts;

            // 평균 RPS 계산: (M1 - M2) / 2 (모터 대칭 장착 가정)
            float combined_rps = (fb.m1_rps - fb.m2_rps) * 0.5f;
            float speed = combined_rps * RPS_TO_MPS;
            
            m1_rps_history.push_back(fb.m1_rps);
            m2_rps_history.push_back(fb.m2_rps);
            combined_rps_history.push_back(combined_rps);
            
            estimated_distance += std::abs(speed) * dt;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 모터 정지 명령
    std::cout << "[STOP] 모터 정지 중..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        ControlPayload stop_cmd = {0.0f, 0.0f, 0.0f};
        STMPacket pkt;
        pkt.header[0] = 0xAA; pkt.header[1] = 0x55; pkt.msg_id = 0x0A;
        pkt.length = sizeof(ControlPayload); pkt.payload = stop_cmd; pkt.crc = 0;
        stm_tx_q.push(pkt);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 통계 계산
    float avg_m1 = 0, avg_m2 = 0, avg_combined = 0;
    if (!combined_rps_history.empty()) {
        avg_m1 = std::accumulate(m1_rps_history.begin(), m1_rps_history.end(), 0.0f) / m1_rps_history.size();
        avg_m2 = std::accumulate(m2_rps_history.begin(), m2_rps_history.end(), 0.0f) / m2_rps_history.size();
        avg_combined = std::accumulate(combined_rps_history.begin(), combined_rps_history.end(), 0.0f) / combined_rps_history.size();
    }

    std::cout << "\n---------------- 측정 결과 ----------------" << std::endl;
    std::cout << " 속도 명령 (V):          " << target_velocity << std::endl;
    std::cout << " 평균 M1 RPS:           " << std::fixed << std::setprecision(4) << avg_m1 << std::endl;
    std::cout << " 평균 M2 RPS:           " << std::fixed << std::setprecision(4) << avg_m2 << std::endl;
    std::cout << " 평균 결합 RPS:         " << std::fixed << std::setprecision(4) << avg_combined << std::endl;
    std::cout << " 평균 속도 (추정):       " << std::fixed << std::setprecision(4) << avg_combined * RPS_TO_MPS << " m/s" << std::endl;
    std::cout << " 주행 거리 (추정):       " << std::fixed << std::setprecision(4) << estimated_distance << " m" << std::endl;
    std::cout << "---------------------------------------------\n" << std::endl;
}

int main() {
    std::signal(SIGINT, signalHandler);

    std::cout << "====================================================" << std::endl;
    std::cout << "   RPS Multi-Speed Accuracy Test Tool (5s Each)     " << std::endl;
    std::cout << "====================================================" << std::endl;
    
    UartLink stm_link("/dev/ttyPS1", 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    Mailbox<FeedbackPayload> stm_fb_mb;
    FeedbackHandler fb_handler(stm_fb_mb);
    stm_parser.registerHandler(0x0B, &fb_handler); // Chassis Feedback ID

    if (!stm_manager.start()) {
        std::cerr << "[ERROR] /dev/ttyPS1 장치를 열거나 STM32 매니저를 시작할 수 없습니다." << std::endl;
        return -1;
    }

    std::vector<float> speeds = {60.0f, 80.0f, 100.0f, 120.0f};
    for (float v : speeds) {
        if (!g_keep_running) break;
        run_test(v, 5.0f, stm_fb_mb, stm_tx_q);
    }

    stm_manager.stop();
    std::cout << "모든 테스트가 종료되었습니다." << std::endl;
    return 0;
}
