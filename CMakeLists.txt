<<<<<<< HEAD
# CMakeLists.txt — EBIMU Torpedo INS v2

cmake_minimum_required(VERSION 3.16)
project(ebimu_torpedo VERSION 2.0.0 LANGUAGES CXX)

=======
# CMakeLists.txt — Aqua-Vector-Torpedo
#
# 어뢰 INS 시스템 빌드 설정.
# - torpedo_core: 핵심 라이브러리 (모든 src/*.cpp)
# - torpedo_main: 어뢰 실행 파일
# - 단위 테스트들 (ctest로 일괄 실행 가능)
#
# 사용:
#   cmake -B build
#   cmake --build build
#   ctest --test-dir build --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(AquaVectorTorpedo
    VERSION 0.5.0
    LANGUAGES CXX
)

# ─── C++ 표준 ───
>>>>>>> build: add CMake build system with ctest integration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

=======
# ─── 기본 빌드 타입 = Release ───
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

<<<<<<< HEAD
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Dense
        PATHS /usr/include/eigen3 /usr/local/include/eigen3
    )
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 못 찾음")
    endif()
endif()

set(CORE_SOURCES
    src/hal/system_clock.cpp
    src/sensor/ebimu_imu.cpp
    src/domain/bias_calibrator.cpp
    src/domain/eskf.cpp
    src/domain/zupt_detector.cpp
    src/app/lidar_aided_ins.cpp        # 추가: LiDAR 보정 INS 조립 계층 (버전 1)
)

add_library(ebimu_core STATIC ${CORE_SOURCES})

target_include_directories(ebimu_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ebimu_core PUBLIC Threads::Threads)

if(TARGET Eigen3::Eigen)
    target_link_libraries(ebimu_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(ebimu_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

enable_testing()

function(add_ebimu_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE ebimu_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_ebimu_test(test_system_clock)
add_ebimu_test(test_imu_sample)
add_ebimu_test(test_ebimu_parser)
add_ebimu_test(test_bias_calibrator)
add_ebimu_test(test_eskf)
add_ebimu_test(test_lidar_aided_ins)   # 추가: 버전 1 통합 검증


message(STATUS "")
message(STATUS "===== EBIMU Torpedo INS v2 =====")
message(STATUS "  버전:      ${PROJECT_VERSION}")
message(STATUS "  컴파일러:  ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  타겟:      ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "=================================")
message(STATUS "")

# 실제 EBIMU 동작 확인 (ZYBO에서 실행)
add_executable(test_ebimu_live src/test_ebimu_live.cpp)
target_link_libraries(test_ebimu_live PRIVATE ebimu_core)
add_executable(test_bias_live src/test_bias_live.cpp)
target_link_libraries(test_bias_live PRIVATE ebimu_core)
add_executable(test_ins_live src/test_ins_live.cpp)
target_link_libraries(test_ins_live PRIVATE ebimu_core)
add_executable(test_quat_check src/test_quat_check.cpp)
target_link_libraries(test_quat_check PRIVATE ebimu_core)
add_executable(test_axis_id src/test_axis_id.cpp)
target_link_libraries(test_axis_id PRIVATE ebimu_core)
add_executable(test_ins_detail_live src/test_ins_detail_live.cpp)
target_link_libraries(test_ins_detail_live PRIVATE ebimu_core)
=======
# ─── 컴파일 옵션 ───
add_compile_options(-Wall -Wextra)

# ─── Eigen 찾기 ───
# 우선순위: find_package → 시스템 경로 → 사용자 경로
find_package(Eigen3 3.3 QUIET NO_MODULE)

if(NOT Eigen3_FOUND)
    # find_package 실패 시 hint 경로
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Dense
        PATHS
            /usr/include/eigen3
            /usr/local/include/eigen3
            $ENV{HOME}/eigen3
            /home/petalinux/eigen3
        DOC "Eigen3 헤더 경로"
    )
    
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR
            "Eigen3 못 찾음. 다음 중 하나:\n"
            "  - sudo apt install libeigen3-dev\n"
            "  - 또는 -DEIGEN3_INCLUDE_DIR=/path/to/eigen3 옵션\n"
        )
    endif()
    
    message(STATUS "Eigen3 발견: ${EIGEN3_INCLUDE_DIR}")
endif()

# ─── 핵심 라이브러리 ───
add_library(torpedo_core STATIC
    src/hal/system_clock.cpp
    src/sensor/fake_imu.cpp
    src/sensor/ism330dhcx_imu.cpp
    src/comm/spidev.cpp
    src/comm/rs485_port.cpp
    src/comm/packet_io.cpp
    src/domain/estimator/eskf_estimator.cpp
)

target_include_directories(torpedo_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# Eigen include (interface — 사용처에도 전파)
if(TARGET Eigen3::Eigen)
    target_link_libraries(torpedo_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(torpedo_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 메인 실행 파일 ───
add_executable(torpedo_main
    src/app/main.cpp
    src/app/main_loop.cpp
)
target_link_libraries(torpedo_main PRIVATE torpedo_core)

# ─── 테스트 ───
enable_testing()

# 헬퍼 함수: 테스트 추가 한 줄로
function(add_torpedo_test name)
    add_executable(${name} tests/${name}.cpp)
    target_link_libraries(${name} PRIVATE torpedo_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# IIMU
add_torpedo_test(test_iimu_compile)

# ESKF
add_torpedo_test(test_eskf_state)
add_torpedo_test(test_eskf_estimator_compile)
add_torpedo_test(test_eskf_predict_p_growth)
add_torpedo_test(test_eskf_predict_stationary)
add_torpedo_test(test_eskf_predict_yaw_rotation)
add_torpedo_test(test_eskf_update_lidar)
add_torpedo_test(test_eskf_update_nhc)
add_torpedo_test(test_eskf_integrated_cycle)

# Comm
add_torpedo_test(test_packet_io)
add_torpedo_test(test_rs485_port)

# ISM330DHCX
add_torpedo_test(test_ism330dhcx_compile)

# ─── 빌드 정보 출력 ───
message(STATUS "")
message(STATUS "===== Aqua-Vector-Torpedo 빌드 설정 =====")
message(STATUS "  버전:      ${PROJECT_VERSION}")
message(STATUS "  빌드 타입: ${CMAKE_BUILD_TYPE}")
message(STATUS "  컴파일러:  ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++:       ${CMAKE_CXX_STANDARD}")
message(STATUS "==========================================")
message(STATUS "")
