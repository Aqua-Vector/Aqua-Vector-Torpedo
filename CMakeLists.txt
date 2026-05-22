# CMakeLists.txt — Aqua-Vector-Torpedo
#
# 어뢰 시스템 통합 빌드 설정 (INS + 통신 + 제어)
# - torpedo_core: 핵심 라이브러리 (모든 비즈니스 로직 및 센서)
# - Aqua-Vector-Torpedo: 메인 실행 파일 (Zynq 타겟)
# - 단위 테스트: ctest 지원

cmake_minimum_required(VERSION 3.16)

project(Aqua-Vector-Torpedo VERSION 1.0.0 LANGUAGES CXX)

# ─── 1. C++ 표준 및 컴파일 옵션 ───
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# 기본 빌드 타입 = Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ─── 2. 의존성 패키지 찾기 (Threads & Eigen3) ───
# POSIX Threads 
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Eigen3
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
    
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 못 찾음. sudo apt install libeigen3-dev 또는 -DEIGEN3_INCLUDE_DIR 옵션 필요")
    endif()
    message(STATUS "Eigen3 발견: ${EIGEN3_INCLUDE_DIR}")
endif()

# ─── 3. 핵심 라이브러리 (torpedo_core) ───
# 메인 함수(main.cpp)를 제외한 모든 소스 코드를 라이브러리로 묶습니다
set(CORE_SOURCES
    src/hal/system_clock.cpp
    src/sensor/fake_imu.cpp
    src/sensor/ism330dhcx_imu.cpp
    src/sensor/imu_downsampler.cpp
    src/comm/spidev.cpp
    src/comm/rs485_port.cpp
    src/comm/packet_io.cpp
    src/domain/estimator/eskf_estimator.cpp
    src/domain/estimator/bias_calibrator.cpp
    
    # -- 통신/유틸리티 --
    src/communication/UartLink.cpp
    src/communication/UdpLink.cpp
    src/protocol/Marshaller.cpp
    src/utils/BufferView.cpp
    src/utils/CrcCalculator.cpp

    # -- 제어 파트 --
    src/control/ModeMux.cpp
    src/control/ManualSource.cpp
    src/control/AutoSource.cpp
)

add_library(torpedo_core STATIC ${CORE_SOURCES})

# Include 디렉토리 설정 
target_include_directories(torpedo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/domain/estimator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/app
    ${CMAKE_CURRENT_SOURCE_DIR}/include/torpedo/comm
)

# 라이브러리 링크 (Eigen, Pthread)
target_link_libraries(torpedo_core PUBLIC Threads::Threads)

if(TARGET Eigen3::Eigen)
    target_link_libraries(torpedo_core PUBLIC Eigen3::Eigen)
else()
    target_include_directories(torpedo_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# ─── 4. 메인 실행 파일 ───
add_executable(${PROJECT_NAME} 
    src/main.cpp
    # src/app/main_loop.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE torpedo_core)

# ─── 5. 단위 테스트 설정 (CTest) ───
enable_testing()

function(add_torpedo_test name)
    add_executable(${name} tests/${name}.cpp)
    # 테스트 바이너리에도 torpedo_core와 pthread가 자동으로 링크됨
    target_link_libraries(${name} PRIVATE torpedo_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_torpedo_test(test_iimu_compile)
add_torpedo_test(test_imu_downsampler)
add_torpedo_test(test_eskf_state)
add_torpedo_test(test_imu_standalone)
# add_torpedo_test(test_mux)

# ─── 6. 빌드 정보 출력 ───
message(STATUS "")
message(STATUS "===== Aqua-Vector-Torpedo 빌드 설정 =====")
message(STATUS "  버전:      ${PROJECT_VERSION}")
message(STATUS "  빌드 타입: ${CMAKE_BUILD_TYPE}")
message(STATUS "  컴파일러:  ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ 표준:  ${CMAKE_CXX_STANDARD}")
message(STATUS "==========================================")
message(STATUS "")
