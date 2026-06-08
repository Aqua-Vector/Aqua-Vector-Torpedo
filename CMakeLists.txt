# CMakeLists.txt — Aqua-Vector-Torpedo (Integrated)
#
# 어뢰 시스템 통합 빌드 설정 (Team INS + Control/Comm/Estimator)

cmake_minimum_required(VERSION 3.16)
project(Aqua-Vector-Torpedo VERSION 1.0.0 LANGUAGES CXX)

# ─── 1. C++ 표준 및 컴파일 옵션 ───
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-poison-system-directories)

# 기본 빌드 타입 = Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ─── 2. 의존성 패키지 찾기 (Threads & Eigen3) ───
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Dense
        PATHS
            /usr/include/eigen3
            /usr/local/include/eigen3
            $ENV{HOME}/eigen3
            /home/petalinux/eigen3
        DOC "Eigen3 헤더 경로"
    )
endif()

# ─── 3. 공통 Include 경로 설정 ───
set(PROJECT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/actuator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ebimu
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ebimu/app
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ebimu/domain
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ebimu/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ebimu/sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/guidance
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/app
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/comm
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain/estimator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
)

# ─── 4. 라이브러리: ebimu_core ───
set(EBIMU_SOURCES
    src/hal/system_clock.cpp
    src/sensor/ebimu_imu.cpp
    src/domain/bias_calibrator.cpp
    src/domain/eskf.cpp
    src/domain/zupt_detector.cpp
    src/app/lidar_aided_ins.cpp
)

add_library(ebimu_core STATIC ${EBIMU_SOURCES})
target_include_directories(ebimu_core PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(ebimu_core PUBLIC Threads::Threads)
if(TARGET Eigen3::Eigen)
    target_link_libraries(ebimu_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(ebimu_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 5. 라이브러리: torpedo_core ───
set(CORE_SOURCES
    src/sensor/fake_imu.cpp
    src/sensor/ism330dhcx_imu.cpp
    src/sensor/imu_downsampler.cpp
    src/sensor/MiniImuUart.cpp
    src/comm/spidev.cpp
    src/comm/rs485_port.cpp
    src/comm/packet_io.cpp
    src/comm/crc16.cpp
    
    # 통신/유틸리티
    src/communication/UartLink.cpp
    src/communication/UdpLink.cpp
    src/protocol/Marshaller.cpp
    src/utils/BufferView.cpp
    src/utils/CrcCalculator.cpp

    # 제어 파트
    src/core/TorpedoControlSystem.cpp
    src/control/ModeMux.cpp
    src/control/ManualSource.cpp
    src/control/AutoSource.cpp
    src/control/ControlDataValidator.cpp
    src/control/ControlHandlers.cpp
    src/actuator/ActuatorManager.cpp
    src/actuator/ServoMotor.cpp
    src/hal/LinuxPwmChannel.cpp

    # 유도/추정
    src/guidance/GuidanceManager.cpp
    src/guidance/GuidancePhaseManager.cpp
    src/guidance/PNGuidanceController.cpp
    src/guidance/TargetStateEstimator.cpp
    src/guidance/EbImuUart.cpp
    
    # 신규 추정기 (Step C)
    src/domain/estimator/bias_calibrator.cpp
    src/domain/estimator/eskf_estimator.cpp
)

add_library(torpedo_core STATIC ${CORE_SOURCES})
target_include_directories(torpedo_core PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(torpedo_core PUBLIC ebimu_core Threads::Threads atomic)
if(TARGET Eigen3::Eigen)
    target_link_libraries(torpedo_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(torpedo_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 6. 메인 실행 파일 ───
# 기존 통합 메인
add_executable(Aqua-Vector-Torpedo 
    src/main.cpp 
    src/app/main_loop.cpp
)
target_link_libraries(Aqua-Vector-Torpedo PRIVATE torpedo_core)

# 신규 어뢰 메인 (ZYBO용)
add_executable(torpedo_main
    src/app/main.cpp
    src/app/main_loop.cpp
)
target_link_libraries(torpedo_main PRIVATE torpedo_core)

# ─── 7. 테스트 ───
enable_testing()

# 신규 통합 시나리오 테스트 (자체 TestFramework 활용)
add_executable(integration_scenario_test tests/integration_scenario_test.cpp)
target_link_libraries(integration_scenario_test PRIVATE torpedo_core)
add_test(NAME integration_scenario_test COMMAND integration_scenario_test)

function(add_torpedo_test name src_path)
    add_executable(${name} ${src_path})
    target_link_libraries(${name} PRIVATE torpedo_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Team INS Tests
add_torpedo_test(test_system_clock tests/test_system_clock.cpp)
add_torpedo_test(test_imu_sample tests/test_imu_sample.cpp)
add_torpedo_test(test_ebimu_parser tests/test_ebimu_parser.cpp)
add_torpedo_test(test_bias_calibrator tests/test_bias_calibrator.cpp)
add_torpedo_test(test_eskf tests/test_eskf.cpp)
add_torpedo_test(test_lidar_aided_ins tests/test_lidar_aided_ins.cpp)

# Original Tests
add_torpedo_test(test_iimu_compile tests/test_iimu_compile.cpp)
add_torpedo_test(test_imu_downsampler tests/test_imu_downsampler.cpp)
add_torpedo_test(test_eskf_state tests/test_eskf_state.cpp)
add_torpedo_test(test_actuator tests/test_actuator.cpp)
add_torpedo_test(test_mux tests/test_mux.cpp)
add_torpedo_test(test_tcs tests/test_tcs.cpp)
add_torpedo_test(test_async_integration tests/test_async_integration.cpp)
add_torpedo_test(test_control_station_protocol tests/test_control_station_protocol.cpp)
add_torpedo_test(valuation_tests tests/valuation_tests.cpp)
add_torpedo_test(system_performance_tests tests/system_performance_tests.cpp)

# Missing Tests in tests/ folder
add_torpedo_test(test_crc16 tests/test_crc16.cpp)
add_torpedo_test(test_eskf_estimator_compile tests/test_eskf_estimator_compile.cpp)
add_torpedo_test(test_eskf_integrated_cycle tests/test_eskf_integrated_cycle.cpp)
add_torpedo_test(test_eskf_predict_p_growth tests/test_eskf_predict_p_growth.cpp)
add_torpedo_test(test_eskf_predict_stationary tests/test_eskf_predict_stationary.cpp)
add_torpedo_test(test_eskf_predict_yaw_rotation tests/test_eskf_predict_yaw_rotation.cpp)
add_torpedo_test(test_eskf_update_lidar tests/test_eskf_update_lidar.cpp)
add_torpedo_test(test_eskf_update_nhc tests/test_eskf_update_nhc.cpp)
add_torpedo_test(test_integration_sim tests/test_integration_sim.cpp)
add_torpedo_test(test_ism330dhcx_compile tests/test_ism330dhcx_compile.cpp)
add_torpedo_test(test_packet_io tests/test_packet_io.cpp)
add_torpedo_test(test_rps_position_accuracy tests/test_rps_position_accuracy.cpp)
add_torpedo_test(test_rs485_port tests/test_rs485_port.cpp)
add_torpedo_test(test_miniimu_uart_class tests/test_miniimu_uart_class.cpp)
add_torpedo_test(test_ebimu_heading tests/test_ebimu_heading.cpp)
add_torpedo_test(test_ebimu_odometry tests/test_ebimu_odometry.cpp)
add_torpedo_test(test_stationary_position tests/test_stationary_position.cpp)
add_torpedo_test(test_dynamic_position_rps tests/test_dynamic_position_rps.cpp)
add_torpedo_test(test_imu_heading_log tests/test_imu_heading_log.cpp)
add_torpedo_test(test_yaw_integration_flow tests/test_yaw_integration_flow.cpp)

# HW Tests
add_torpedo_test(calib_distance tests/calib_distance.cpp)
add_torpedo_test(calib_steering_deadband tests/calib_steering_deadband.cpp)
add_torpedo_test(calib_circle_360 tests/calib_circle_360.cpp)

add_torpedo_test(basic_stm32_test tests/basic_stm32_test.cpp)
add_torpedo_test(hw_test_stm32 tests/hw_test_stm32.cpp)
add_torpedo_test(test_stm_feedback tests/test_stm_feedback.cpp)
add_torpedo_test(hw_test_control_station tests/hw_test_control_station.cpp)
add_torpedo_test(hw_test_integration_rc_car tests/hw_test_integration_rc_car.cpp)
add_torpedo_test(hw_test_command_pipeline tests/hw_test_command_pipeline.cpp)
add_torpedo_test(hw_test_uplink tests/hw_test_uplink.cpp)
add_torpedo_test(hw_test_rps_accuracy tests/hw_test_rps_accuracy.cpp)
add_torpedo_test(hw_test_rps_multi_speed tests/hw_test_rps_multi_speed.cpp)
add_torpedo_test(hw_test_ins_only_guidance tests/hw_test_ins_only_guidance.cpp)
add_torpedo_test(hw_test_pn_guidance tests/hw_test_pn_guidance.cpp)
add_torpedo_test(hw_test_kinematic_pn_guidance tests/hw_test_kinematic_pn_guidance.cpp)
add_torpedo_test(hw_test_midcourse_guidance tests/hw_test_midcourse_guidance.cpp)
add_torpedo_test(hw_test_lidar_ins_integrated tests/hw_test_lidar_ins_integrated.cpp)
add_torpedo_test(integration_hil_test tests/integration_hil_test.cpp)

# Utils/Misc
add_executable(benchmark_cpu tests/benchmark_cpu.cpp)
target_link_libraries(benchmark_cpu PRIVATE torpedo_core)

add_executable(rs_rx tests/RX.cpp)
target_link_libraries(rs_rx PRIVATE torpedo_core)

add_executable(rs_tx tests/TX.cpp)
target_link_libraries(rs_tx PRIVATE torpedo_core)

# ─── 8. 라이브 테스트 (src/ 에 위치) ───
function(add_src_test name src_path)
    add_executable(${name} ${src_path})
    target_link_libraries(${name} PRIVATE torpedo_core)
endfunction()

add_src_test(test_bias_live src/test_bias_live.cpp)
add_src_test(test_ebimu_live src/test_ebimu_live.cpp)
add_src_test(test_ins_live src/test_ins_live.cpp)
add_src_test(test_quat_check src/test_quat_check.cpp)
add_src_test(test_ins_detail_live src/test_ins_detail_live.cpp)
add_src_test(test_axis_id src/test_axis_id.cpp)

# ─── 빌드 정보 출력 ───
message(STATUS "")
message(STATUS "===== Aqua-Vector-Torpedo 빌드 설정 (완전 복구) =====")
message(STATUS "  버전:      ${PROJECT_VERSION}")
message(STATUS "  빌드 타입: ${CMAKE_BUILD_TYPE}")
message(STATUS "  컴파일러:  ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ 표준:  ${CMAKE_CXX_STANDARD}")
message(STATUS "==========================================")
message(STATUS "")
