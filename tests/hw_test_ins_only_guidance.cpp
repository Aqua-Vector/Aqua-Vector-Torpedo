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
#include "protocol/ControlStationParser.hpp"
#include "protocol/ProtocolPolicies.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/Mailbox.hpp"
#include "utils/TimeUtils.hpp"
#include "utils/LowPassFilter.hpp"

// Team's INS Module
#include "ebimu/app/lidar_aided_ins.hpp"
#include "ebimu/sensor/ebimu_imu.hpp"

// Guidance
#include "guidance/GuidanceManager.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

using STMPacket = GenericPacket<ControlPayload, uint8_t>;
using GCSPacket = GenericPacket<TorpedoUplinkPayload, uint16_t>;

std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief GCS(통제소) 명령 패킷 핸들러
 */
class GcsHandler : public IMessageHandler {
public:
    Mailbox<ControlStationPayload>& mb;
    explicit GcsHandler(Mailbox<ControlStationPayload>& mailbox) : mb(mailbox) {}

    bool handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) override {
        if (length != sizeof(ControlStationPayload)) {
            return false;
        }
        ControlStationPayload data;
        Marshaller::deserialize(payload, length, data);
        mb.update(data, timestamp_ms);
        return true;
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    // [Modify] 포트 고정 및 인자 단순화
    const std::string imu_port = "/dev/ttyUSB0";
    const std::string stm_port = "/dev/ttyPS1";
    const std::string gcs_port = "/dev/ttyS2";

    float test_velocity = 120.0f;
    if (argc > 1) {
        test_velocity = std::stof(argv[1]);
    }

    std::cout << "================================================================" << std::endl;
    std::cout << "   [STRAIGHT] 8-Seconds Straight Running Test                   " << std::endl;
    std::cout << "================================================================" << std::endl;
    std::cout << " Velocity : " << static_cast<int>(test_velocity) << std::endl;
    std::cout << " Duration : 8.0 seconds" << std::endl;
    std::cout << " Ports    : IMU=" << imu_port << ", STM=" << stm_port << ", GCS=" << gcs_port << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Ebimu 및 INS 초기화
    ebimu::EbimuConfig imu_cfg;
    imu_cfg.device = imu_port;
    imu_cfg.baud = 921600;
    ebimu::EbimuImu imu(imu_cfg);
    
    ebimu::LidarAidedInsConfig ins_cfg;
    ebimu::LidarAidedIns ins(imu, ins_cfg);

    if (!ins.init()) {
        std::cerr << "[ERROR] INS/EBIMU 초기화 실패!" << std::endl;
        return -1;
    }

    // 2. 5초 정지 상태 캘리브레이션
    if (!ins.calibrate()) {
        std::cerr << "[ERROR] 캘리브레이션 실패! (데이터 수신 여부를 확인하세요)" << std::endl;
        return -1;
    }
    std::cout << "[OK] 캘리브레이션 완료." << std::endl;

    // 3. 통신 설정 (STM32 & GCS)
    UartLink stm_link(stm_port, 230400);
    STMControlParser stm_parser;
    StaticRingBuffer<STMPacket, 64> stm_tx_q;
    NetworkManager<STMControlParser, UartLink, STMPacket> stm_manager(stm_link, stm_parser, stm_tx_q);
    
    UartLink gcs_link(gcs_port, 115200);
    ControlStationParser gcs_parser;
    StaticRingBuffer<GCSPacket, 64> gcs_tx_q;
    NetworkManager<ControlStationParser, UartLink, GCSPacket> gcs_manager(gcs_link, gcs_parser, gcs_tx_q);

    Mailbox<ControlStationPayload> gcs_mb;
    GcsHandler gcs_handler(gcs_mb);
    gcs_parser.registerHandler(0x00, &gcs_handler); 

    if (!stm_manager.start() || !gcs_manager.start()) {
        std::cerr << "[ERROR] 통신 매니저 시작 실패!" << std::endl;
        return -1;
    }

    // 4. 유도 매니저 초기화 (PN 포함)
    guidance::GuidanceManager guidance_manager;
    uint16_t uplink_seq = 0;

    std::cout << "\n>>> ENTER를 누르면 3초 후 시작합니다. <<<" << std::endl;
    std::string dummy;
    std::getline(std::cin, dummy);

    for (int i = 3; i > 0; --i) {
        std::cout << "[COUNTDOWN] " << i << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    imu.flush(); // 루프 시작 직전 버퍼 비우기 (지연 방지)
    std::cout << "\n[RUNNING] Straight Running Started (8 seconds)..." << std::endl;

    auto loop_start_time = std::chrono::steady_clock::now();
    auto last_update_time = loop_start_time;
    auto last_cmd_time = loop_start_time;
    auto last_log_time = loop_start_time;
    auto last_uplink_time = loop_start_time;

    while (g_keep_running) {
        auto now = std::chrono::steady_clock::now();
        float elapsed_total = std::chrono::duration<float>(now - loop_start_time).count();
        float dt = std::chrono::duration<float>(now - last_update_time).count();

        if (elapsed_total >= 8.0f) {
            std::cout << "\n[INFO] 8 seconds reached. Stopping..." << std::endl;
            break;
        }

        // A. GCS 데이터 확인 및 LiDAR 주입
        ControlStationPayload gcs_data;
        uint64_t gcs_ts;
        std::optional<Eigen::Vector2f> gcs_target = std::nullopt;
        bool terminal_trigger = false;

        if (gcs_mb.fetch(gcs_data, gcs_ts)) {
            ins.feed_lidar(gcs_data.torpedo_x, gcs_data.torpedo_y);
            gcs_target = Eigen::Vector2f(gcs_data.target_x, gcs_data.target_y);
            terminal_trigger = (gcs_data.flags & 0x01);
        } else {
            // 직진 테스트이므로 타겟 불필요
            gcs_target = std::nullopt;
        }

        // B. INS 업데이트
        if (ins.step()) {
            last_update_time = now;
            
            auto ins_state = ins.get_state();
            const auto& bias = ins.bias();
            Eigen::Quaternionf q_start(bias.q_start_w, bias.q_start_x, bias.q_start_y, bias.q_start_z);
            q_start.normalize();

            // GuidanceManager용 상태 구성 (Relative Frame)
            torpedo::domain::EskfState my_state;
            my_state.p = q_start.conjugate() * Eigen::Vector3f(ins_state.px, ins_state.py, ins_state.pz);
            my_state.v = q_start.conjugate() * Eigen::Vector3f(ins_state.vx, ins_state.vy, ins_state.vz);
            
            float yaw_rad_ccw = -(ins.get_yaw_rel_deg() * (M_PI / 180.0f));
            my_state.q = Eigen::Quaternionf(Eigen::AngleAxisf(yaw_rad_ccw, Eigen::Vector3f::UnitZ()));

            // [Modified] PN 유도 대신 8초간 직진 명령
            float target_velocity = test_velocity;

            // D. STM32 송신 (50Hz)
            if (std::chrono::duration<float>(now - last_cmd_time).count() >= 0.02f) {
                ControlPayload cmd = {target_velocity, 0.0f, 0.0f}; // 직진
                STMPacket pkt;
                pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL;
                pkt.payload = cmd;
                stm_manager.send(pkt);
                last_cmd_time = now;
            }

            // E. GCS 업링크 송신 (10Hz)
            if (std::chrono::duration<float>(now - last_uplink_time).count() >= 0.1f) {
                GCSPacket pkt;
                pkt.header[0] = TelemetryPolicy::SYNC1;
                pkt.header[1] = TelemetryPolicy::SYNC2;
                pkt.msg_id = TelemetryPolicy::MSG_ID;
                pkt.length = sizeof(TorpedoUplinkPayload);
                
                pkt.payload.seq = uplink_seq++;
                pkt.payload.p_x = my_state.p.x();
                pkt.payload.p_y = my_state.p.y();
                pkt.payload.yaw = -yaw_rad_ccw; // CCW+ -> CW+
                pkt.payload.status_flags = 0xAA; // 직진 모드 표시용 더미
                
                // [특수 상황] Uplink CRC는 싱크 바이트 제외 계산
                pkt.crc = TelemetryPolicy::calculateCrc(&pkt.msg_id, 2 + pkt.length);
                gcs_manager.send(pkt);
                last_uplink_time = now;
            }

            // F. 로깅 (10Hz)
            if (std::chrono::duration<float>(now - last_log_time).count() >= 0.1f) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "\r[STRAIGHT] Time: " << std::setw(4) << elapsed_total << "s | Pos: (" << std::setw(6) << my_state.p.x() << ", " << std::setw(6) << my_state.p.y() << ")"
                          << " | Yaw: " << std::setw(6) << ins.get_yaw_rel_deg() << " deg"
                          << (ins.last_lidar_used() ? " [LiDAR]" : "        ") << std::flush;
                last_log_time = now;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    // 종료 시 정지
    std::cout << "\n[STOP] Stopping motors..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        ControlPayload stop_cmd = {0.0f, 0.0f, 0.0f};
        STMPacket pkt; pkt.msg_id = PACKET_FUNC_CHASSIS_CTRL; pkt.payload = stop_cmd;
        stm_manager.send(pkt);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    stm_manager.stop();
    gcs_manager.stop();
    return 0;
}

