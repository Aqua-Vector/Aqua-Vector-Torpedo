#pragma once
#include "ebimu/sensor/iimu.hpp"
#include <cstdint>
#include <unistd.h>
namespace ebimu {
class FakeImu : public IImu {
public:
    FakeImu() = default;
    void set_bias(float bax, float bay, float baz){bax_=bax;bay_=bay;baz_=baz;}
    void set_gravity(float g){g_=g;}
    void set_gyro(float gx,float gy,float gz){gx_=gx;gy_=gy;gz_=gz;}
    void set_quaternion(float qw,float qx,float qy,float qz){qw_=qw;qx_=qx;qy_=qy;qz_=qz;}
    void set_accel(float ax,float ay,float az){use_accel_=true;ax_=ax;ay_=ay;az_=az;}
    void set_dt_us(uint64_t dt){dt_us_=dt;}
    // 테스트 전용: read_sample 호출당 실제 sleep(us). 실제 EBIMU 99Hz 흉내.
    void set_read_throttle_us(uint32_t us){throttle_us_=us;}
    bool init() override { initialized_=true; t_us_=0; return true; }
    bool read_sample(ImuSample& out) override {
        if(!initialized_) return false;
        if(throttle_us_) usleep(throttle_us_);
        t_us_+=dt_us_; out.t_us=t_us_;
        if(use_accel_){ out.ax=ax_; out.ay=ay_; out.az=az_; }
        else { out.ax=bax_; out.ay=bay_; out.az=g_+baz_; }
        out.gx=gx_; out.gy=gy_; out.gz=gz_;
        out.qw=qw_; out.qx=qx_; out.qy=qy_; out.qz=qz_;
        return true;
    }
    void close() override { initialized_=false; }
private:
    bool initialized_=false; uint64_t t_us_=0; uint64_t dt_us_=10000;
    uint32_t throttle_us_=0;
    float bax_=0,bay_=0,baz_=0; float g_=9.80665f;
    float gx_=0,gy_=0,gz_=0;
    float qw_=1,qx_=0,qy_=0,qz_=0;
    bool use_accel_=false; float ax_=0,ay_=0,az_=0;
};
}