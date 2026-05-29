#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <iomanip>
#include <cmath>

// Team member's new INS headers
#include "ebimu/app/lidar_aided_ins.hpp"
#include "ebimu/sensor/ebimu_imu.hpp"

// Project communication headers
#include "communication/UartLink.hpp"
#include "protocol/TorpedoParser.hpp"
#include "protocol/Payloads.hpp"
#include "protocol/ProtocolIds.hpp"
#include "protocol/Marshaller.hpp"
#include "utils/StaticRingBuffer.hpp"
#include "utils/TimeUtils.hpp"
#include "utils/CrcCalculator.hpp"

using namespace ebimu;

// Global flag for graceful shutdown
std::atomic<bool> g_keep_running(true);
void signalHandler(int) { g_keep_running = false; }

/**
 * @brief GCS Data Handler
 * Receives Lidar position from GCS and feeds it to the LidarAidedIns module.
 */
class LidarAidedInsParser : public IPacketParser {
private:
    LidarAidedIns& ins_;
    uint8_t buffer_[128];
    size_t rx_idx_ = 0;
    int state_ = 0;
    uint8_t expected_length_ = 0;
    uint64_t last_rx_time_ms_ = 0;

public:
    explicit LidarAidedInsParser(LidarAidedIns& ins) : ins_(ins) {}

    void parseByte(uint8_t byte, uint64_t timestamp_ms) override {
        if (state_ != 0 && (timestamp_ms - last_rx_time_ms_) > 50) {
            state_ = 0;
            rx_idx_ = 0;
        }
        last_rx_time_ms_ = timestamp_ms;

        if (state_ == 0) {
            if (byte == 0xAA) { buffer_[0] = byte; state_ = 1; }
        } else if (state_ == 1) {
            if (byte == 0x55) { buffer_[1] = byte; state_ = 2; }
            else if (byte == 0xAA) { state_ = 1; }
            else { state_ = 0; }
        } else if (state_ == 2) {
            buffer_[2] = byte; // msg_id
            state_ = 3;
        } else if (state_ == 3) {
            buffer_[3] = byte; // length
            expected_length_ = byte;
            if (expected_length_ > 100) { state_ = 0; return; }
            state_ = 4;
            rx_idx_ = 4;
        } else if (state_ == 4) {
            buffer_[rx_idx_++] = byte;
            size_t total_expected = expected_length_ + 6;
            if (rx_idx_ >= total_expected) {
                if (expected_length_ == sizeof(ControlStationPayload)) {
                    uint16_t calc_crc = utils::CrcCalculator::CalculateCrc16Ccitt(buffer_, total_expected - 2);
                    uint16_t received_crc = buffer_[total_expected - 2] | (buffer_[total_expected - 1] << 8);
                    
                    if (calc_crc == received_crc) {
                        ControlStationPayload payload;
                        std::memcpy(&payload, &buffer_[4], sizeof(ControlStationPayload));
                        
                        // Feed Lidar data to INS
                        // Note: GCS provides torpedo_x/y as Lidar measurements
                        ins_.feed_lidar(payload.torpedo_x, payload.torpedo_y);
                    }
                }
                state_ = 0; rx_idx_ = 0;
            }
        }
    }

    struct ProtocolPolicy { static constexpr size_t MAX_PAYLOAD_SIZE = 128; };
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>& pkt, uint8_t* buf, size_t max_len) {
        size_t total_size = 4 + sizeof(TorpedoUplinkPayload) + 2;
        if (max_len < total_size) return 0;
        std::memcpy(buf, &pkt, total_size);
        return total_size;
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    std::string gcs_port = "/dev/ttyS2";
    std::string imu_port = "/dev/ttyUSB0"; // Default EBIMU port
    if (argc > 1) gcs_port = argv[1];
    if (argc > 2) imu_port = argv[2];

    std::cout << "================================================================" << std::endl;
    std::cout << "   Integrated Lidar-Aided INS Test (Team Member's Module)       " << std::endl;
    std::cout << "================================================================" << std::endl;

    // 1. Initialize Sensors & INS
    EbimuConfig imu_cfg;
    imu_cfg.device = imu_port;
    imu_cfg.baud = 921600;
    EbimuImu imu(imu_cfg);

    LidarAidedInsConfig ins_cfg;
    ins_cfg.calib_duration_us = 3000000ULL; // 3 seconds calibration
    LidarAidedIns ins(imu, ins_cfg);

    if (!ins.init()) {
        std::cerr << "[INS] Failed to initialize INS/IMU!" << std::endl;
        return -1;
    }

    std::cout << "[INS] Starting Calibration (Keep Torpedo Still)..." << std::endl;
    if (!ins.calibrate()) {
        std::cerr << "[INS] Calibration Failed!" << std::endl;
        return -1;
    }
    std::cout << "[INS] Calibration Complete." << std::endl;

    // 2. Initialize Communication
    UartLink gcs_link(gcs_port, 115200);
    if (!gcs_link.initialize()) {
        std::cerr << "[GCS] Failed to open " << gcs_port << std::endl;
        return -1;
    }

    LidarAidedInsParser parser(ins);
    StaticRingBuffer<GenericPacket<TorpedoUplinkPayload, uint16_t>, 64> tx_queue;

    // 3. Main Loop
    std::cout << "[SYS] Starting Main Loop..." << std::endl;
    std::cout << "\n" << std::left << std::setw(12) << "Time[s]" 
              << std::setw(15) << "Pos(X, Y)" 
              << std::setw(10) << "Yaw[deg]" 
              << std::setw(10) << "LidarCnt" 
              << std::setw(10) << "ZUPT" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_log_time = start_time;
    auto last_uplink_time = start_time;
    uint16_t seq = 0;

    uint8_t rx_buf[1024];

    while (g_keep_running) {
        // A. Process incoming GCS data
        ssize_t n = gcs_link.receive(rx_buf, sizeof(rx_buf));
        if (n > 0) {
            uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            for (ssize_t i = 0; i < n; ++i) {
                parser.parseByte(rx_buf[i], now_ms);
            }
        }

        // B. Update INS state
        if (ins.step()) {
            // C. Periodic Uplink (10Hz)
            auto now = std::chrono::steady_clock::now();
            if (now - last_uplink_time >= std::chrono::milliseconds(100)) {
                last_uplink_time = now;
                auto state = ins.get_state();
                
                GenericPacket<TorpedoUplinkPayload, uint16_t> pkt;
                pkt.header[0] = 0xAA;
                pkt.header[1] = 0x55;
                pkt.msg_id = 0x02; // MSG_ID_UPLINK
                pkt.length = sizeof(TorpedoUplinkPayload);
                
                pkt.payload.seq = seq++;
                pkt.payload.p_x = state.px;
                pkt.payload.p_y = state.py;
                pkt.payload.yaw = ins.get_yaw_rel_deg() * (M_PI / 180.0f); // Convert to rad for GCS
                pkt.payload.status_flags = ins.last_lidar_used() ? 0x01 : 0x00;
                pkt.payload.status_flags |= ins.last_zupt_active() ? 0x02 : 0x00;
                
                pkt.crc = utils::CrcCalculator::CalculateCrc16Ccitt(reinterpret_cast<uint8_t*>(&pkt.msg_id), 2 + pkt.length);
                
                uint8_t tx_buf[128];
                size_t tx_len = parser.serialize(pkt, tx_buf, sizeof(tx_buf));
                if (tx_len > 0) {
                    gcs_link.send(tx_buf, tx_len);
                }
            }

            // D. Log Status (1Hz)
            if (now - last_log_time >= std::chrono::seconds(1)) {
                last_log_time = now;
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
                auto state = ins.get_state();
                
                std::cout << std::left << std::fixed << std::setprecision(2)
                          << std::setw(12) << (double)elapsed
                          << "(" << std::setw(6) << state.px << "," << std::setw(6) << state.py << ")  "
                          << std::setw(10) << ins.get_yaw_rel_deg()
                          << std::setw(10) << ins.lidar_update_count()
                          << std::setw(10) << (ins.last_zupt_active() ? "ACTIVE" : "OFF") << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::cout << "[SYS] Stopping..." << std::endl;
    gcs_link.close();
    return 0;
}
