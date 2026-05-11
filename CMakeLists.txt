cmake_minimum_required(VERSION 3.10)

project(Aqua-Vector-Torpedo VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)
# add_compile_options(-Werror)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(SOURCES
	src/main.cpp
	src/communication/UartLink.cpp
	src/communication/UdpLink.cpp
	src/protocol/Marshaller.cpp
	src/utils/BufferView.cpp
	src/utils/CrcCalculator.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/include/communication
	${CMAKE_CURRENT_SOURCE_DIR}/include/core
	${CMAKE_CURRENT_SOURCE_DIR}/include/protocol
	${CMAKE_CURRENT_SOURCE_DIR}/include/utils
	${CMAKE_CURRENT_SOURCE_DIR}/include/common
	${CMAKE_CURRENT_SOURCE_DIR}/include/control
)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
