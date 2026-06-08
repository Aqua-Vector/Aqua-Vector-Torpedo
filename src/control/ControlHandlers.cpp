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
        // [사용자 요구사항] 중기 유도 시에는 GCS 실측값, 종말 유도 시에는 추측항법(DR) 값 출력
        float log_x, log_y;
        if (tcs_.getGuidanceManager().getPhase() == GuidancePhase::MIDCOURSE) {
            log_x = data.target_x;
            log_y = data.target_y;
        } else {
            const auto& dr_target = tcs_.getGuidanceManager().getTargetState().pos;
            log_x = dr_target.x();
            log_y = dr_target.y();
        }

        std::cout << "[COMM] RX <- GCS | Seq: " << data.seq 
                  << " | Target: (" << std::fixed << std::setprecision(1) << log_x << ", " << log_y << ")"
                  << " | Phase: " << static_cast<int>(tcs_.getGuidanceManager().getPhase())
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

bool Stm32FeedbackHandler::handle(const uint8_t* payload, size_t length, uint64_t timestamp_ms) {
    if (length != sizeof(FeedbackPayload)) return false;
    
    FeedbackPayload data;
    Marshaller::deserialize(payload, length, data);
    
    // Update TCS with feedback
    tcs_.onStm32FeedbackReceived(data, timestamp_ms);
    return true;
}
