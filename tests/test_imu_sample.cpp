// test_imu_sample.cpp — ImuSample / IImu / FakeImu 검증
#include "ebimu/sensor/imu_sample.hpp"
#include "ebimu/sensor/iimu.hpp"
#include "ebimu/sensor/fake_imu.hpp"

#include <cstdio>

int main() {
    std::printf("=== ImuSample / IImu / FakeImu 검증 ===\n");
    
    // [Test 1] ImuSample 기본 값
    ebimu::ImuSample s;
    if (s.t_us != 0 || s.ax != 0.0f || s.qw != 1.0f) {
        std::printf("[FAIL] 기본 값 잘못\n");
        return 1;
    }
    std::printf("[OK] ImuSample 기본 값 (qw=1, 나머지 0)\n");
    
    // [Test 2] FakeImu — init/read/close
    ebimu::FakeImu imu;
    
    // init 전 read 실패
    ebimu::ImuSample s1;
    if (imu.read_sample(s1)) {
        std::printf("[FAIL] init 전 read 성공\n");
        return 1;
    }
    std::printf("[OK] init 전 read 실패\n");
    
    if (!imu.init()) {
        std::printf("[FAIL] init 실패\n");
        return 1;
    }
    std::printf("[OK] init 성공\n");
    
    // [Test 3] 정지 + 기본 (중력만)
    {
        ebimu::ImuSample s2;
        imu.read_sample(s2);
        if (s2.az < 9.7f || s2.az > 9.9f) {
            std::printf("[FAIL] 기본 az=%f (기대 ~9.81)\n", s2.az);
            return 1;
        }
        std::printf("[OK] 정지 기본: az=%.4f (중력)\n", s2.az);
    }
    
    // [Test 4] t_us 증가 (100Hz 기본)
    {
        ebimu::ImuSample s3, s4;
        imu.read_sample(s3);
        imu.read_sample(s4);
        if (s4.t_us <= s3.t_us) {
            std::printf("[FAIL] t_us 안 증가\n");
            return 1;
        }
        std::printf("[OK] t_us 증가 (s3=%lu, s4=%lu)\n",
                    (unsigned long)s3.t_us, (unsigned long)s4.t_us);
    }
    
    // [Test 5] bias 설정 동작
    {
        imu.close();
        imu.set_bias(0.3f, -0.4f, 0.04f);
        imu.init();
        
        ebimu::ImuSample s5;
        imu.read_sample(s5);
        
        if (s5.ax < 0.29f || s5.ax > 0.31f) {
            std::printf("[FAIL] bias ax 잘못: %f\n", s5.ax);
            return 1;
        }
        std::printf("[OK] bias 적용: ax=%.4f, ay=%.4f, az=%.4f (g+bias)\n",
                    s5.ax, s5.ay, s5.az);
    }
    
    imu.close();
    std::printf("[OK] close\n");
    
    std::printf("\n[ALL PASS] ImuSample / FakeImu 검증 통과\n");
    return 0;
}