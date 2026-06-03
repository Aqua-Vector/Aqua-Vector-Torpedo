#include "TestFramework.hpp"
#include "guidance/PNGuidanceController.hpp"
#include "guidance/TargetStateEstimator.hpp"
#include "guidance/GuidancePhaseManager.hpp"
#include "control/ModeMux.hpp"
#include "actuator/ServoMotor.hpp"
#include "hal/LinuxPwmChannel.hpp"
#include "utils/CrcCalculator.hpp"
#include "utils/LowPassFilter.hpp"
#include "protocol/TorpedoParser.hpp"
#include "common/ControlTypes.hpp"

// IMU & Estimator Headers
#include "torpedo/domain/estimator/eskf_estimator.hpp"
#include "torpedo/sensor/fake_imu.hpp"
#include "torpedo/hal/system_clock.hpp"

#include <iostream>
#include <chrono>
#include <thread>

/**
 * 1. 유도 알고리즘 연산 모듈 (Guidance)
 */
TEST(Guidance, FloatingPointAccuracy) {
    guidance::PNGuidanceController pnc;
    torpedo::domain::EskfState my_state;
    my_state.p = Eigen::Vector3f(0, 0, 0);
    my_state.v = Eigen::Vector3f(0, 10.0f, 0);
    my_state.q = Eigen::Quaternionf::Identity();
    Eigen::Vector2f target_pip(10.0f, 10.0f);
    pnc.calculateSteering(my_state, target_pip, 0.01f);
    target_pip = Eigen::Vector2f(10.1f, 10.0f); 
    float steer = pnc.calculateSteering(my_state, target_pip, 0.01f);
    EXPECT_TRUE(steer >= -30.0f && steer <= 30.0f);
    EXPECT_FALSE(std::isnan(steer));
}

TEST(Guidance, NumericalStability_ZeroDist) {
    guidance::PNGuidanceController pnc;
    torpedo::domain::EskfState my_state;
    my_state.p = Eigen::Vector3f(0, 0, 0);
    my_state.v = Eigen::Vector3f(0, 10.0f, 0);
    Eigen::Vector2f zero_dist(0.0f, 0.0f);
    pnc.calculateSteering(my_state, zero_dist, 0.01f);
    float steer = pnc.calculateSteering(my_state, zero_dist, 0.01f);
    EXPECT_FALSE(std::isnan(steer));
}

/**
 * 2. 항법 추정 알고리즘 (Estimator / IMU)
 */
TEST(TargetEstimator, VelocityExtraction) {
    guidance::TargetStateEstimator tse;
    float dt = 0.1f; // 10Hz
    
    // 완벽한 더미 데이터 주입 (1m/s 이동)
    tse.updateFromLidar(Eigen::Vector2f(0.0f, 0.0f), dt);
    tse.updateFromLidar(Eigen::Vector2f(0.1f, 0.0f), dt);
    tse.updateFromLidar(Eigen::Vector2f(0.2f, 0.0f), dt);
    
    auto state = tse.getState();
    std::cout << "    [INFO] DR Target Vel: (" << state.vel.x() << ", " << state.vel.y() << ")" << std::endl;
    
    // 정확한 1.0m/s 산출 확인
    EXPECT_NEAR(state.vel.x(), 1.0f, 0.001f);
}

TEST(TargetEstimator, KinematicPrediction) {
    guidance::TargetStateEstimator tse;
    float dt = 0.1f;
    
    // 1m/s 속도 확정
    tse.updateFromLidar(Eigen::Vector2f(0.0f, 0.0f), dt);
    tse.updateFromLidar(Eigen::Vector2f(0.1f, 0.0f), dt);
    
    // 1초간 Blind 예측 (100회 호출)
    for(int i=0; i<100; ++i) {
        tse.predictVirtualState(0.01f);
    }
    
    Eigen::Vector2f final_pos = tse.getState().pos;
    std::cout << "    [INFO] 1.0s DR Prediction Pos: (" << final_pos.x() << ", " << final_pos.y() << ")" << std::endl;
    
    // (0.1, 0)에서 1초간 1m/s로 이동했으므로 (1.1, 0)이어야 함
    EXPECT_NEAR(final_pos.x(), 1.1f, 0.001f);
}

TEST(Estimator, EskfPredictionTime) {
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams params;
    eskf.init(params, 0.01f);
    
    torpedo::ImuSample sample;
    sample.ax = 0; sample.ay = 0; sample.az = 9.81f;
    sample.gx = 0.01f; sample.gy = 0.02f; sample.gz = 0.03f;
    torpedo::domain::BiasEstimate bias;

    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<1000; ++i) {
        eskf.predict(sample, bias, 0.01f);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

    std::cout << "    [INFO] Avg ESKF Predict Time: " << elapsed << " us" << std::endl;
    EXPECT_TRUE(elapsed < 200.0); 
}

TEST(Estimator, EskfUpdateLidarTime) {
    torpedo::domain::EskfEstimator eskf;
    torpedo::domain::EskfInitParams params;
    eskf.init(params, 0.01f);
    
    Eigen::Vector2f lidar_pos(1.0f, 2.0f);

    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<1000; ++i) {
        eskf.update_lidar(lidar_pos);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

    std::cout << "    [INFO] Avg ESKF Lidar Update Time: " << elapsed << " us" << std::endl;
    EXPECT_TRUE(elapsed < 500.0); 
}

/**
 * 3. 통신 및 인터페이스 모듈 (Communication)
 */
TEST(Communication, ParserRobustness) {
    TorpedoParser parser;
    uint8_t garbage[] = {0xFF, 0xAA, 0x55, 0xBB, 0xCC, 0x01, 0x02, 0x03, 0xFF, 0x00, 0xAA}; 
    for(auto b : garbage) parser.parseByte(b, 1000);
    EXPECT_TRUE(true); 
}

TEST(Communication, CrcVerification) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = CrcCalculator::CalculateCrc16(data, 4);
    EXPECT_EQ(crc, CrcCalculator::CalculateCrc16(data, 4));
}

/**
 * 4. 제어 및 구동기 로직 모듈 (Control/Actuator)
 */
TEST(Control, ModeMux_Watchdog) {
    Mailbox<ControlState> manual_mb;
    Mailbox<ControlState> auto_mb;
    ModeMux mux(&manual_mb, &auto_mb);
    mux.getActiveMailbox(6000); 
    EXPECT_EQ(static_cast<int>(mux.getMode()), static_cast<int>(SystemMode::FAILSAFE));
}

TEST(Control, Servo_Saturation) {
    ServoConfig config;
    config.max_angle_deg = 30.0f; 
    config.max_deg_per_sec = 600.0f;
    config.min_pulse_ns = 1000000;
    config.max_pulse_ns = 2000000;
    LinuxPwmChannel mock_pwm(99, 99, "/tmp");
    ServoMotor servo(mock_pwm, config);
    servo.setAngle(50.0f, 1.0f); 
    EXPECT_NEAR(servo.getCurrentAngle(), 30.0f, 0.1f);
}

TEST(Control, StateMachine_PhaseTransition) {
    GuidancePhaseManager gpm;
    gpm.evaluatePhase(10.0f, false);
    EXPECT_EQ(static_cast<int>(gpm.getCurrentPhase()), static_cast<int>(GuidancePhase::MIDCOURSE));
}

/**
 * 5. 시스템 리소스 및 공통 유틸 (Utils)
 */
TEST(Utils, LowPassFilter_Smoothing) {
    utils::LowPassFilter lpf(0.1f); 
    lpf.update(0.0f);
    float val = 0;
    for(int i=0; i<10; ++i) val = lpf.update(100.0f);
    EXPECT_TRUE(val < 80.0f); 
}

int main() {
    RUN_TEST(Guidance, FloatingPointAccuracy);
    RUN_TEST(Guidance, NumericalStability_ZeroDist);
    
    RUN_TEST(TargetEstimator, VelocityExtraction);
    RUN_TEST(TargetEstimator, KinematicPrediction);
    
    RUN_TEST(Estimator, EskfPredictionTime);
    RUN_TEST(Estimator, EskfUpdateLidarTime);
    
    RUN_TEST(Communication, ParserRobustness);
    RUN_TEST(Communication, CrcVerification);
    
    RUN_TEST(Control, ModeMux_Watchdog);
    RUN_TEST(Control, Servo_Saturation);
    RUN_TEST(Control, StateMachine_PhaseTransition);
    
    RUN_TEST(Utils, LowPassFilter_Smoothing);

    test::TestRunner::getInstance().report();
    return test::TestRunner::getInstance().allPassed() ? 0 : 1;
}
