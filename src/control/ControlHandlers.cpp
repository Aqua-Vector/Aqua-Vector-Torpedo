#include "control/ControlHandlers.hpp"
#include "core/TorpedoControlSystem.hpp"
#include "protocol/Marshaller.hpp"
#include <cstring>

bool ControlStationHandler::handle(const uint8_t* payload, size_t len, uint64_t ts) {
    if (len != sizeof(ControlStationPayload)) return false;
    
    ControlStationPayload data;
    Marshaller::deserialize(payload, len, data);

    // TCS에 GCS 데이터 전달 (Lidar 좌표, 타겟 좌표 등)
    tcs_.onGcsDataReceived(data, ts);

    // [Add] GCS RX 로그 (약 1초 주기)
    static uint32_t gcs_rx_cnt = 0;
    if (++gcs_rx_cnt % 100 == 1) {
        std::cout << "[COMM] RX <- GCS | Seq: " << data.seq 
                  << " | Target: (" << data.target_x << ", " << data.target_y << ")"
                  << " | Steer: " << data.steer << std::endl;
    }

    // GCS의 조향 명령을 ManualSource에 전달 (중기 유도 테스트 용)
    ControlPayload cmd;
    cmd.velocity = target_velocity_; 
    cmd.rudder = static_cast<float>(data.steer); // [Fix] 반전 없이 그대로 전달 (내부 표준 CW+ 일치)
    cmd.elevator = 0.0f;
    ms_.onControlPacketReceived(cmd, ts);

    return true;
}
