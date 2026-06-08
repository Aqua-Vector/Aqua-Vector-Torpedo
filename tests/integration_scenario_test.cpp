#include "TestFramework.hpp"
#include "core/TorpedoControlSystem.hpp"
#include "control/ModeMux.hpp"
#include "control/ManualSource.hpp"
#include "control/AutoSource.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/hal/system_clock.hpp"
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/domain/estimator/rps_tracker.hpp"
#include "protocol/ControlStationParser.hpp"
#include "control/STMControlParser.hpp"
#include "core/NetworkManager.hpp"
#include "communication/UartLink.hpp"
#include "utils/TimeUtils.hpp"
#include <thread>
#include <chrono>

/**
 * @brief Mock Network Manager for testing TX output
 */
template <typename T>
class MockNetworkManager : public ITxNetworkManager<T> {
public:
    T last_sent_pkt;
    int send_count = 0;
    bool start() override { return true; }
    void stop() override {}
    bool send(const T& pkt) override {
        last_sent_pkt = pkt;
        send_count++;
        return true;
    }
};

uint64_t getNow() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

TEST(IntegrationScenario, StandbyStateAtStartup) {
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux{manual_source.getMailbox(), auto_source.getMailbox()};
    torpedo::SystemClock clock;
    torpedo::FakeImu fake_imu(clock);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    MockNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_tx;
    MockNetworkManager<STMPacket> stm_tx;

    TorpedoControlSystem tcs(mode_mux, manual_source, auto_source, fake_imu, eskf, rps_tracker, gcs_tx, stm_tx);
    tcs.init(true);
    
    EXPECT_EQ(static_cast<int>(mode_mux.getMode()), static_cast<int>(SystemMode::STANDBY));
    tcs.stop();
}

TEST(IntegrationScenario, LaunchOnMidcourseFlag) {
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux{manual_source.getMailbox(), auto_source.getMailbox()};
    torpedo::SystemClock clock;
    torpedo::FakeImu fake_imu(clock);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    MockNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_tx;
    MockNetworkManager<STMPacket> stm_tx;

    TorpedoControlSystem tcs(mode_mux, manual_source, auto_source, fake_imu, eskf, rps_tracker, gcs_tx, stm_tx);
    tcs.init(true);

    // [Fix] GCS 패킷 주입 (모드 전환용)
    ControlStationPayload gcs_pkt;
    gcs_pkt.flags = 0x01; // MIDCOURSE_FLAG
    gcs_pkt.target_x = 10.0f;
    gcs_pkt.target_y = 10.0f;
    tcs.onGcsDataReceived(gcs_pkt, getNow());

    // [Fix] ManualSource에도 직접 데이터 주입 (실제로는 ControlStationHandler가 수행하는 역할)
    ControlPayload manual_cmd = {120.0f, 5.0f, 0.0f};
    manual_source.onControlPacketReceived(manual_cmd, getNow());

    tcs.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 

    EXPECT_EQ(static_cast<int>(mode_mux.getMode()), static_cast<int>(SystemMode::MANUAL));
    EXPECT_TRUE(stm_tx.send_count > 0);
    // 송신된 조향값이 입력값(-5.0f, 반전 적용됨) 근처인지 확인
    EXPECT_NEAR(stm_tx.last_sent_pkt.payload.rudder, -5.0f, 0.1f);

    tcs.stop();
}

TEST(IntegrationScenario, TerminalToInterceptSequence) {
    ManualSource manual_source;
    AutoSource auto_source;
    ModeMux mode_mux{manual_source.getMailbox(), auto_source.getMailbox()};
    torpedo::SystemClock clock;
    torpedo::FakeImu fake_imu(clock);
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::RpsPositionTracker rps_tracker;
    MockNetworkManager<GenericPacket<TorpedoUplinkPayload, uint16_t>> gcs_tx;
    MockNetworkManager<STMPacket> stm_tx;

    TorpedoControlSystem tcs(mode_mux, manual_source, auto_source, fake_imu, eskf, rps_tracker, gcs_tx, stm_tx);
    tcs.init(true);

    // 초기 상태 MANUAL로 시작하도록 유도
    ControlStationPayload gcs_pkt;
    gcs_pkt.flags = 0x01;
    tcs.onGcsDataReceived(gcs_pkt, getNow());
    manual_source.onControlPacketReceived({-60, 0, 0}, getNow());
    
    tcs.start();

    // 1. Terminal 전환 (0x02 flag)
    gcs_pkt.flags = 0x02; // TERMINAL_FLAG
    gcs_pkt.target_x = 1.0f;
    gcs_pkt.target_y = 1.0f;
    tcs.onGcsDataReceived(gcs_pkt, getNow());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_EQ(static_cast<int>(mode_mux.getMode()), static_cast<int>(SystemMode::AUTO));

    // 2. 요격 상황 연출 (거리 < 0.2m)
    gcs_pkt.target_x = 0.05f;
    gcs_pkt.target_y = 0.05f;
    tcs.onGcsDataReceived(gcs_pkt, getNow());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    EXPECT_EQ(static_cast<int>(mode_mux.getMode()), static_cast<int>(SystemMode::LOCKDOWN));
    EXPECT_NEAR(stm_tx.last_sent_pkt.payload.velocity, 0.0f, 0.01f);
    EXPECT_NEAR(stm_tx.last_sent_pkt.payload.rudder, 0.0f, 0.01f);

    tcs.stop();
}

int main() {
    RUN_TEST(IntegrationScenario, StandbyStateAtStartup);
    RUN_TEST(IntegrationScenario, LaunchOnMidcourseFlag);
    RUN_TEST(IntegrationScenario, TerminalToInterceptSequence);
    
    test::TestRunner::getInstance().report();
    return test::TestRunner::getInstance().allPassed() ? 0 : 1;
}
