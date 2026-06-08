#include <iostream>
#include <memory>
#include <iomanip>
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "control/ModeMux.hpp"

// 테스트 결과를 보기 좋게 출력하는 헬퍼 함수
void printState(uint64_t time_ms, SystemMode mode, const ControlState& state) {
    std::string mode_str = "UNKNOWN ";
    if (mode == SystemMode::STANDBY) mode_str = "STANDBY ";
    else if (mode == SystemMode::MANUAL) mode_str = "MANUAL  ";
    else if (mode == SystemMode::AUTO) mode_str = "AUTO    ";
    else if (mode == SystemMode::FAILSAFE) mode_str = "FAILSAFE";
    else if (mode == SystemMode::LOCKDOWN) mode_str = "LOCKDOWN";

    std::cout << "[Time: " << std::setw(4) << time_ms << "ms] "
              << "Mode: " << mode_str
              << " | V: " << std::fixed << std::setprecision(1) << state.velocity 
              << " | R: " << state.rudder 
              << " | E: " << state.elevator << std::endl;
}

int main() {
    std::cout << "========== [ ModeMux Core Test (Multiplexer Ver.) ] ==========\n\n";

    // 1. 객체 초기화
    auto manual_source = std::make_shared<ManualSource>();
    auto auto_source = std::make_shared<AutoSource>();
    ModeMux mode_mux(manual_source->getMailbox(), auto_source->getMailbox());
    
    uint64_t virtual_time_ms = 100;

    std::cout << ">> 시나리오 1: 정상적인 데이터 수신 (Manual)\n";
    for (int i = 0; i < 3; ++i) {
        ControlPayload mock_packet = {1.5f, 10.0f, -5.0f};
        manual_source->onControlPacketReceived(mock_packet, virtual_time_ms);

        auto* mb = mode_mux.getActiveMailbox(virtual_time_ms);
        ControlState final_state;
        uint64_t ts;
        mb->fetch(final_state, ts);
        
        printState(virtual_time_ms, mode_mux.getMode(), final_state);
        virtual_time_ms += 20;
    }

    std::cout << "\n>> 시나리오 2: 통신 두절 (Watchdog 발동)\n";
    virtual_time_ms += 600; 
    
    auto* mb_fail = mode_mux.getActiveMailbox(virtual_time_ms);
    ControlState fail_state;
    uint64_t ts_fail;
    mb_fail->fetch(fail_state, ts_fail);
    printState(virtual_time_ms, mode_mux.getMode(), fail_state);

    std::cout << "\n>> 시나리오 3: 하드웨어 에러 (LOCKDOWN 강제)\n";
    mode_mux.notifyHardwareError();
    
    // 데이터가 들어와도 LOCKDOWN이어야 함
    ControlPayload dummy = {5.0f, 5.0f, 5.0f};
    manual_source->onControlPacketReceived(dummy, virtual_time_ms);
    
    auto* mb_lock = mode_mux.getActiveMailbox(virtual_time_ms);
    ControlState lock_state;
    uint64_t ts_lock;
    mb_lock->fetch(lock_state, ts_lock);
    printState(virtual_time_ms, mode_mux.getMode(), lock_state);

    std::cout << "\n>> 시나리오 4: AUTO 모드 전환 및 데이터 확인\n";
    // LOCKDOWN 해제 로직이 없으므로 새로운 Mux 생성하여 테스트
    ModeMux mode_mux_v2(manual_source->getMailbox(), auto_source->getMailbox());
    mode_mux_v2.setMode(SystemMode::AUTO);
    
    ControlState auto_data = {10.0f, -20.0f, 5.0f};
    auto_source->updateTargetState(auto_data, virtual_time_ms);
    
    auto* mb_auto = mode_mux_v2.getActiveMailbox(virtual_time_ms);
    ControlState auto_state;
    uint64_t ts_auto;
    mb_auto->fetch(auto_state, ts_auto);
    printState(virtual_time_ms, mode_mux_v2.getMode(), auto_state);

    std::cout << "\n==============================================================\n";
    return 0;
}
