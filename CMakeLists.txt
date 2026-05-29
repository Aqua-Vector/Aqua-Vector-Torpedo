# CMakeLists.txt — EBIMU Torpedo INS v2

cmake_minimum_required(VERSION 3.16)
project(ebimu_torpedo VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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
target_link_libraries(test_bias_live PRIVATE ebimu_core)
add_executable(test_ins_detail_live src/test_ins_detail_live.cpp)
target_link_libraries(test_ins_detail_live PRIVATE ebimu_core)