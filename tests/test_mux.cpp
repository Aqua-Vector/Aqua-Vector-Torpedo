#include <iostream>
#include <memory>
#include <iomanip>
#include "control/ManualSource.hpp"
#include "control/ModeMux.hpp"

// 테스트 결과를 보기 좋게 출력하는 헬퍼 함수
void printState(uint64_t time_ms, SystemMode mode, const ControlState& state) {
    std::string mode_str = (mode == SystemMode::MANUAL) ? "MANUAL  " : "FAILSAFE";
    std::cout << "[Time: " << std::setw(4) << time_ms << "ms] "
              << "Mode: " << mode_str
              << " | V: " << std::fixed << std::setprecision(1) << state.velocity 
              << " | R: " << state.rudder 
              << " | E: " << state.elevator << std::endl;
}

int main() {
    std::cout << "========== [ ModeMux Core Test ] ==========\n\n";

    // 1. 객체 초기화 (의존성 주입)
    auto manual_source = std::make_shared<ManualSource>();
    ModeMux mode_mux(manual_source, nullptr);
    
    // 가상의 시스템 시간 (0ms 부터 시작)
    uint64_t virtual_time_ms = 0;

    std::cout << ">> 시나리오 1: 정상적인 데이터 수신 (20ms 주기)\n";
    for (int i = 0; i < 3; ++i) {
        ControlPayload mock_packet = {1.5f, 10.0f, -5.0f};
        manual_source->onControlPacketReceived(mock_packet, virtual_time_ms);

        ControlState final_state = mode_mux.processControlLoop(virtual_time_ms);
        printState(virtual_time_ms, mode_mux.getMode(), final_state);

        virtual_time_ms += 20;
    }

    std::cout << "\n>> 시나리오 2: 통신 두절 (500ms 이상 경과 -> Watchdog 발동)\n";
    virtual_time_ms += 600; // 수신 없이 시간만 흐름
    
    ControlState timeout_state = mode_mux.processControlLoop(virtual_time_ms);
    printState(virtual_time_ms, mode_mux.getMode(), timeout_state);

    std::cout << "\n>> 시나리오 3: Latching(락다운) 테스트 (통신 복구 시도)\n";
    virtual_time_ms += 20;
    
    ControlPayload late_packet = {2.0f, 15.0f, 0.0f};
    manual_source->onControlPacketReceived(late_packet, virtual_time_ms);
    
    // 새로운 데이터가 들어왔어도 FAILSAFE가 유지되어야 함
    ControlState latch_state = mode_mux.processControlLoop(virtual_time_ms);
    printState(virtual_time_ms, mode_mux.getMode(), latch_state);

    std::cout << "\n>> 시나리오 4: 관리자 수동 개입 (제어권 회복)\n";
    virtual_time_ms += 20;
    
    mode_mux.setMode(SystemMode::MANUAL);
    
    ControlState recovered_state = mode_mux.processControlLoop(virtual_time_ms);
    printState(virtual_time_ms, mode_mux.getMode(), recovered_state);

    std::cout << "\n===========================================\n";
    return 0;
}
