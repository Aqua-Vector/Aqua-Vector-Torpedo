#include "ebimu/domain/bias_calibrator.hpp"
#include "ebimu/hal/system_clock.hpp"
#include <Eigen/Dense>
#include <cmath>
namespace ebimu {
void BiasCalibrator::start(){
    collecting_=true; t_start_us_=monotonic_us(); n_=0;
    ax_nav_sum_=ay_nav_sum_=az_nav_sum_=0.0;
    qw_sum_=qx_sum_=qy_sum_=qz_sum_=0.0;
}
void BiasCalibrator::add_sample(const ImuSample& s){
    if(!collecting_) return;
    uint64_t elapsed=monotonic_us()-t_start_us_;
    if(elapsed>=duration_us_){ collecting_=false; return; }
    // float → double 자동 승격하여 누적
    qw_sum_+=s.qw; qx_sum_+=s.qx; qy_sum_+=s.qy; qz_sum_+=s.qz;
    Eigen::Quaternionf q(s.qw,s.qx,s.qy,s.qz); q.normalize();
    Eigen::Vector3f a_body(s.ax,s.ay,s.az);
    Eigen::Vector3f a_nav=q.toRotationMatrix()*a_body;
    ax_nav_sum_+=a_nav(0); ay_nav_sum_+=a_nav(1); az_nav_sum_+=a_nav(2); n_++;
}
BiasCalibration BiasCalibrator::finalize(){
    collecting_=false; BiasCalibration result; result.n_samples=n_;
    if(n_<100){ result.valid=false; return result; }
    double inv_n=1.0/(double)n_;
    double qw_avg=qw_sum_*inv_n,qx_avg=qx_sum_*inv_n,qy_avg=qy_sum_*inv_n,qz_avg=qz_sum_*inv_n;
    double norm=std::sqrt(qw_avg*qw_avg+qx_avg*qx_avg+qy_avg*qy_avg+qz_avg*qz_avg);
    if(norm<0.001){ result.valid=false; return result; }
    result.q_start_w=(float)(qw_avg/norm); result.q_start_x=(float)(qx_avg/norm);
    result.q_start_y=(float)(qy_avg/norm); result.q_start_z=(float)(qz_avg/norm);
    double ax_m=ax_nav_sum_*inv_n,ay_m=ay_nav_sum_*inv_n,az_m=az_nav_sum_*inv_n;
    result.bx=(float)ax_m; result.by=(float)ay_m; result.bz=(float)(az_m-(double)g_);
    result.valid=true; return result;
}
float BiasCalibrator::progress() const {
    if(!collecting_) return 1.0f;
    uint64_t e=monotonic_us()-t_start_us_;
    if(e>=duration_us_) return 1.0f;
    return (float)e/(float)duration_us_;
}
}