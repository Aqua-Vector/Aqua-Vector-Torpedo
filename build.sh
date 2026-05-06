#!/bin/bash
# Day 1 골격 빌드 스크립트
# 사용: ./build.sh

set -e  # 에러 시 즉시 중단

cd "$(dirname "$0")"

echo "===> Building test_iimu..."
g++ -std=c++17 -Wall -Iinclude \
    tests/test_iimu_compile.cpp \
    src/hal/system_clock.cpp \
    src/sensor/fake_imu.cpp \
    -o test_iimu

echo "===> Build OK. Running test..."
echo ""
./test_iimu

echo ""
echo "===> Done."
