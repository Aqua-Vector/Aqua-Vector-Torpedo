// test_ebimu_parser.cpp — EBIMU ASCII parser 검증
//
// 실측 1시간 측정 데이터에서 라인 가져와 parse 검증.

#include "ebimu/sensor/ebimu_imu.hpp"

#include <cstdio>
#include <cmath>

bool nearly(float a, float b, float tol = 1e-4f) {
    return std::fabs(a - b) < tol;
}

int main() {
    std::printf("=== EBIMU parser 검증 ===\n");
    
    // [Test 1] 정상 라인 — 1시간 측정 첫 줄
    // "100-0,-0.6599,-0.0027,0.0003,0.7512,0.0,0.0,0.0,-0.006,0.005,1.000,78"
    {
        const char* line = "100-0,-0.6599,-0.0027,0.0003,0.7512,0.0,0.0,0.0,-0.006,0.005,1.000,78";
        ebimu::ImuSample s;
        if (!ebimu::EbimuImu::parse_line(line, s)) {
            std::printf("[FAIL] 정상 라인 parse 실패\n");
            return 1;
        }
        
        // Quaternion 그대로
        if (!nearly(s.qw, -0.6599f) || !nearly(s.qx, -0.0027f) || 
            !nearly(s.qy, 0.0003f) || !nearly(s.qz, 0.7512f)) {
            std::printf("[FAIL] quaternion: %f %f %f %f\n", s.qw, s.qx, s.qy, s.qz);
            return 1;
        }
        std::printf("[OK] quaternion (w=%.4f, x=%.4f, y=%.4f, z=%.4f)\n",
                    s.qw, s.qx, s.qy, s.qz);
        
        // 가속도: g → m/s² (× 9.80665)
        if (!nearly(s.ax, -0.006f * 9.80665f, 1e-3f) ||
            !nearly(s.az,  1.000f * 9.80665f, 1e-2f)) {
            std::printf("[FAIL] accel: ax=%f az=%f\n", s.ax, s.az);
            return 1;
        }
        std::printf("[OK] accel (ax=%.4f, ay=%.4f, az=%.4f) m/s²\n", s.ax, s.ay, s.az);
        
        // 자이로: 정지라 거의 0
        if (!nearly(s.gx, 0.0f) || !nearly(s.gz, 0.0f)) {
            std::printf("[FAIL] gyro: gx=%f gz=%f\n", s.gx, s.gz);
            return 1;
        }
        std::printf("[OK] gyro (gx=%.4f, gy=%.4f, gz=%.4f) rad/s\n", s.gx, s.gy, s.gz);
    }
    
    // [Test 2] 자이로 변동 — 1시간 중간 한 줄
    // "100-0,-0.9999,-0.0041,-0.0006,-0.0127,0.2,0.5,-0.1,0.002,0.006,1.001,91"
    {
        const char* line = "100-0,-0.9999,-0.0041,-0.0006,-0.0127,0.2,0.5,-0.1,0.002,0.006,1.001,91";
        ebimu::ImuSample s;
        if (!ebimu::EbimuImu::parse_line(line, s)) {
            std::printf("[FAIL] 자이로 변동 라인 parse 실패\n");
            return 1;
        }
        // 자이로 0.5 dps = 0.5 * π/180 ≈ 0.00873 rad/s
        if (!nearly(s.gy, 0.5f * 0.01745329f, 1e-4f)) {
            std::printf("[FAIL] gy 변환: %f (기대 ~0.00873)\n", s.gy);
            return 1;
        }
        std::printf("[OK] 자이로 dps→rad/s 변환 (gy=0.5dps → %.5frad/s)\n", s.gy);
    }
    
    // [Test 3] 손상된 라인 거부
    {
        // 콤마 부족 (필드 짧음)
        const char* bad1 = "100-0,1.0,2.0";
        ebimu::ImuSample s;
        if (ebimu::EbimuImu::parse_line(bad1, s)) {
            std::printf("[FAIL] 짧은 라인을 통과시킴\n");
            return 1;
        }
        std::printf("[OK] 짧은 라인 거부\n");
        
        // 비숫자 데이터
        const char* bad2 = "100-0,abc,xyz,...";
        if (ebimu::EbimuImu::parse_line(bad2, s)) {
            std::printf("[FAIL] 비숫자 라인 통과\n");
            return 1;
        }
        std::printf("[OK] 비숫자 라인 거부\n");
        
        // 빈 라인
        if (ebimu::EbimuImu::parse_line("", s)) {
            std::printf("[FAIL] 빈 라인 통과\n");
            return 1;
        }
        std::printf("[OK] 빈 라인 거부\n");
        
        // nullptr
        if (ebimu::EbimuImu::parse_line(nullptr, s)) {
            std::printf("[FAIL] nullptr 통과\n");
            return 1;
        }
        std::printf("[OK] nullptr 거부\n");
    }
    
    // [Test 4] 헤더 없는 라인 거부
    {
        // 콤마 없으면 첫 콤마 찾기 실패
        const char* no_header = "1.0 2.0 3.0";
        ebimu::ImuSample s;
        if (ebimu::EbimuImu::parse_line(no_header, s)) {
            std::printf("[FAIL] 콤마 없는 라인 통과\n");
            return 1;
        }
        std::printf("[OK] 콤마 없는 라인 거부\n");
    }
    
    std::printf("\n[ALL PASS] EBIMU parser 검증 통과\n");
    return 0;
}