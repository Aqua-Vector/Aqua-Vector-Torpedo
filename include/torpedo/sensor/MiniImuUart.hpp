#ifndef MINI_IMU_UART_HPP_
#define MINI_IMU_UART_HPP_

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/sensor/imu_sample.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <cstdint>

namespace torpedo::sensor {

/**
 * @brief MiniIMU-UART / MPU6050 UART 모듈 드라이버
 * 
 * 0x55 헤더 기반의 11바이트 패킷을 파싱하여 가속도 및 각속도 데이터를 추출합니다.
 * IImu 인터페이스를 구현하며, 내부적으로 백그라운드 스레드를 통해 비차단(Non-blocking) 수신을 수행합니다.
 */
class MiniImuUart : public IImu {
private:
        int fd_ = -1;
        std::string device_path_;
        int baud_rate_;

        std::thread rx_thread_;
        std::atomic<bool> is_running_{false};

        ImuSample latest_sample_;
        std::atomic<uint32_t> seq_counter_{0};

        uint32_t bad_checksum_count_ = 0;

        // 내부 통신 및 파싱 함수
        int openSerial(const char* dev, int baud);
        bool readByte(uint8_t& out, int timeout_ms);
        bool readFrame(uint8_t frame[11]);
        bool checksumOk(const uint8_t frame[11]);
        void rxLoop();

public:
        MiniImuUart(const std::string& dev = "/dev/ttyS3", int baud = 115200);
        ~MiniImuUart() override;

        // IImu 인터페이스 구현
        bool init() override;
        void shutdown() override;
        bool read(ImuSample& out_sample) override;

        uint32_t getErrorCount() const { return bad_checksum_count_; }
};

} // namespace torpedo::sensor

#endif /* MINI_IMU_UART_HPP_ */
