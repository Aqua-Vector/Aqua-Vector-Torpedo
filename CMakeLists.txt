# CMakeLists.txt — Aqua-Vector-Torpedo (Integrated)
#
# 어뢰 시스템 통합 빌드 설정 (Team INS v2 + Control/Comm)

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

# ─── 3. 팀원 INS 라이브러리 (ebimu_core) ───
set(EBIMU_SOURCES
    src/hal/system_clock.cpp
    src/sensor/ebimu_imu.cpp
    src/domain/bias_calibrator.cpp
    src/domain/eskf.cpp
    src/domain/zupt_detector.cpp
    src/app/lidar_aided_ins.cpp
)

add_library(ebimu_core STATIC ${EBIMU_SOURCES})
target_include_directories(ebimu_core PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(ebimu_core PUBLIC Threads::Threads)
if(TARGET Eigen3::Eigen)
    target_link_libraries(ebimu_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(ebimu_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 4. 기존 핵심 라이브러리 (torpedo_core) ───
set(CORE_SOURCES
    src/sensor/fake_imu.cpp
    src/sensor/ism330dhcx_imu.cpp
    src/sensor/imu_downsampler.cpp
    src/sensor/MiniImuUart.cpp
    src/comm/spidev.cpp
    src/comm/rs485_port.cpp
    src/comm/packet_io.cpp
    src/comm/crc16.cpp
    
    # -- 통신/유틸리티 --
    src/communication/UartLink.cpp
    src/communication/UdpLink.cpp
    src/protocol/Marshaller.cpp
    src/utils/BufferView.cpp
    src/utils/CrcCalculator.cpp

    # -- 제어 파트 --
    src/core/TorpedoControlSystem.cpp
    src/control/ModeMux.cpp
    src/control/ManualSource.cpp
    src/control/AutoSource.cpp
    src/control/ControlDataValidator.cpp
    src/actuator/ActuatorManager.cpp
    src/actuator/ServoMotor.cpp
    src/hal/LinuxPwmChannel.cpp

    # -- 유도/추정 --
    src/guidance/GuidanceManager.cpp
    src/guidance/GuidancePhaseManager.cpp
    src/guidance/PNGuidanceController.cpp
    src/guidance/TargetStateEstimator.cpp
    src/guidance/EbImuUart.cpp
)

add_library(torpedo_core STATIC ${CORE_SOURCES})

# Include 디렉토리 설정
target_include_directories(torpedo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/actuator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/guidance
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain/estimator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/app
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/comm
)

# 라이브러리 링크 (Team INS, Eigen, Pthread, Atomic)
target_link_libraries(torpedo_core PUBLIC ebimu_core Threads::Threads atomic)

if(TARGET Eigen3::Eigen)
    target_link_libraries(torpedo_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(torpedo_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 5. 메인 실행 파일 ───
add_executable(Aqua-Vector-Torpedo 
    src/main.cpp 
    src/app/main_loop.cpp
)
target_link_libraries(Aqua-Vector-Torpedo PRIVATE torpedo_core)

# ─── 6. 테스트 ───
enable_testing()

function(add_torpedo_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE torpedo_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Team INS Tests
add_torpedo_test(test_system_clock)
add_torpedo_test(test_imu_sample)
add_torpedo_test(test_ebimu_parser)
add_torpedo_test(test_bias_calibrator)
add_torpedo_test(test_eskf)
add_torpedo_test(test_lidar_aided_ins)

# Original Tests
add_torpedo_test(test_iimu_compile)
add_torpedo_test(test_imu_downsampler)
add_torpedo_test(test_eskf_state)
add_torpedo_test(test_actuator)
add_torpedo_test(test_mux)
add_torpedo_test(test_tcs)
add_torpedo_test(test_command_routing)
add_torpedo_test(test_async_integration)
add_torpedo_test(test_control_station_protocol)

# Hardware & Manual Tests
add_executable(hw_test_uplink tests/hw_test_uplink.cpp)
target_link_libraries(hw_test_uplink PRIVATE torpedo_core)

add_executable(hw_test_lidar_ins_integrated tests/hw_test_lidar_ins_integrated.cpp)
target_link_libraries(hw_test_lidar_ins_integrated PRIVATE torpedo_core)

add_executable(hw_test_stm32 tests/hw_test_stm32.cpp)
target_link_libraries(hw_test_stm32 PRIVATE torpedo_core)

add_executable(test_stm_feedback tests/test_stm_feedback.cpp)
target_link_libraries(test_stm_feedback PRIVATE torpedo_core)

add_executable(hw_test_control_station tests/hw_test_control_station.cpp)
target_link_libraries(hw_test_control_station PRIVATE torpedo_core)

add_executable(hw_test_integration_rc_car tests/hw_test_integration_rc_car.cpp)
target_link_libraries(hw_test_integration_rc_car PRIVATE torpedo_core)

add_executable(hw_test_command_pipeline tests/hw_test_command_pipeline.cpp)
target_link_libraries(hw_test_command_pipeline PRIVATE torpedo_core)

add_executable(hw_test_uplink tests/hw_test_uplink.cpp)
target_link_libraries(hw_test_uplink PRIVATE torpedo_core)

add_executable(test_miniimu_uart_class tests/test_miniimu_uart_class.cpp)
target_link_libraries(test_miniimu_uart_class PRIVATE torpedo_core)

add_executable(test_imu_heading_log tests/test_imu_heading_log.cpp)
target_link_libraries(test_imu_heading_log PRIVATE torpedo_core)

add_executable(test_stationary_position tests/test_stationary_position.cpp)
target_link_libraries(test_stationary_position PRIVATE torpedo_core)

add_executable(test_dynamic_position_rps tests/test_dynamic_position_rps.cpp)
target_link_libraries(test_dynamic_position_rps PRIVATE torpedo_core)

add_executable(test_ebimu_heading tests/test_ebimu_heading.cpp)
target_link_libraries(test_ebimu_heading PRIVATE torpedo_core)

add_executable(test_ebimu_odometry tests/test_ebimu_odometry.cpp)
target_link_libraries(test_ebimu_odometry PRIVATE torpedo_core)

add_executable(integration_hil_test tests/integration_hil_test.cpp)
target_link_libraries(integration_hil_test PRIVATE torpedo_core)

add_executable(rs_rx tests/RX.cpp)
target_link_libraries(rs_rx PRIVATE torpedo_core)

add_executable(rs_tx tests/TX.cpp)
target_link_libraries(rs_tx PRIVATE torpedo_core)

# ─── 7. 빌드 정보 출력 ───
message(STATUS "")
message(STATUS "===== Aqua-Vector-Torpedo 빌드 설정 (통합) =====")
message(STATUS "  버전:      ${PROJECT_VERSION}")
message(STATUS "  빌드 타입: ${CMAKE_BUILD_TYPE}")
message(STATUS "  컴파일러:  ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ 표준:  ${CMAKE_CXX_STANDARD}")
message(STATUS "==========================================")
