#ifndef EB_IMU_UART_HPP_
#define EB_IMU_UART_HPP_

#include "torpedo/sensor/iimu.hpp"
#include "torpedo/sensor/imu_sample.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <cstdint>
#include <vector>

namespace torpedo::sensor {

/**
 * @brief EBIMU24GV52 UART 모듈 드라이버 (ASCII 모드)
 * 
 * "ID,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,bat\r\n" 형식의 ASCII 데이터를 파싱합니다.
 * IImu 인터페이스를 구현하며, 내부적으로 백그라운드 스레드를 통해 비차단(Non-blocking) 수신을 수행합니다.
 */
class EbImuUart : public IImu {
private:
    int fd_ = -1;
    std::string device_path_;
    int baud_rate_;

    std::thread rx_thread_;
    std::atomic<bool> is_running_{false};

    ImuSample latest_sample_;
    std::atomic<uint32_t> seq_counter_{0};

    // 내부 통신 및 파싱 함수
    int openSerial(const char* dev, int baud);
    void rxLoop();
    void parseLine(const std::string& line);
    std::vector<std::string> split(const std::string& s, char delimiter);

public:
    EbImuUart(const std::string& dev = "/dev/ttyUSB0", int baud = 115200);
    ~EbImuUart() override;

    // IImu 인터페이스 구현
    bool init() override;
    void shutdown() override;
    bool read(ImuSample& out_sample) override;
};

} // namespace torpedo::sensor

#endif /* EB_IMU_UART_HPP_ */
