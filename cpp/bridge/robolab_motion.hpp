#pragma once
#include "core.hpp"
namespace pi05 {
// RoboLab absolute target -> optional IIR -> per-cycle step limit -> torque PD.
// No dynamic leader offset and no requirement for leader/follower pose equality.
class RobolabMotion {
  const Config& cfg_;
  Joints desired_, target_, filtered_, velocity_{}, measured_, dq_{}, kp_, kd_;
  bool held_{true}, stopping_{false}, home_{false};
  unsigned stop_cycles_{0};
  double alpha_, home_velocity_, velocity_ff_;
  Joints ff_velocity_{};
 public:
  RobolabMotion(const Config& cfg, const Joints& q):cfg_(cfg),desired_(q),target_(q),filtered_(q),measured_(q),
      kp_{120,120,120,120,120,150,60},kd_{20,20,20,20,20,20,20},alpha_(cfg.raw.value("cmd_filter_alpha",1.)),home_velocity_(cfg.raw.value("home_max_velocity",.06)) {
    velocity_ff_=cfg.raw.value("teleop_velocity_feedforward",0.);
    if(!std::isfinite(velocity_ff_) || velocity_ff_<0 || velocity_ff_>1)
      throw std::invalid_argument("teleop_velocity_feedforward must be in [0, 1]");
    cfg.check_position(q);
    if(!std::isfinite(home_velocity_) || home_velocity_<=0 || home_velocity_>.2)
      throw std::invalid_argument("home_max_velocity must be in (0, 0.2] rad/s");
    if(!std::isfinite(alpha_) || alpha_<=0 || alpha_>1) throw std::invalid_argument("invalid command filter alpha");
  }
  void synchronize(const Joints& q,const Joints& dq) { measured_=q;dq_=dq; }
  void target(const Joints& q,bool home=false) {
    cfg_.check_position(q);
    if(stopping_)return;
    if(held_)filtered_=measured_;
    held_=false;home_=home;target_=q;
  }
  void hold() noexcept { held_=true;target_=filtered_=desired_;velocity_.fill(0);ff_velocity_.fill(0); }
  void stop() noexcept { if(!stopping_)hold();stopping_=true; }
  bool hold_ready() const noexcept {
    return held_ && !stopping_ && std::all_of(dq_.begin(),dq_.end(),[](double x){return std::abs(x)<=.01;});
  }
  bool update(double gap=.001) {
    // Original RoboLab torque control advances once per received callback,
    // independently of duration. libfranka owns communication-constraint errors;
    // never replay missed cycles or add the position controller's gap cutoff.
    if(!std::isfinite(gap)||gap<=0)
      throw std::runtime_error("invalid torque callback interval");
    velocity_.fill(0);
    if(!held_&&!stopping_)for(size_t i=0;i<7;++i){
      filtered_[i]=alpha_*target_[i]+(1-alpha_)*filtered_[i];
      const double step=std::min(home_?home_velocity_:.5,cfg_.velocity[i])*.001;
      const double delta=std::clamp(filtered_[i]-desired_[i],-step,step);
      desired_[i]+=delta;velocity_[i]=delta/.001;
    }
    // Optional reference-velocity feedforward, ramped at the existing per-axis
    // acceleration bound. No differentiation of noisy GELLO/USB positions.
    // Home/hold/shutdown retain the original damping-only behavior.
    for(size_t i=0;i<7;++i) {
      if(held_ || stopping_ || home_)ff_velocity_[i]=0.;
      else {
        const double change=cfg_.acceleration[i]*.001;
        ff_velocity_[i]+=std::clamp(velocity_[i]-ff_velocity_[i],-change,change);
      }
    }
    cfg_.check_position(desired_);
    // Hold keeps the previous reference, removing the old remote target.
    // Bounded shutdown delegates the final stop to libfranka.
    return stopping_ && (++stop_cycles_>=1000 || std::all_of(dq_.begin(),dq_.end(),[](double x){return std::abs(x)<=.01;}));
  }
  Joints torques() const {
    Joints tau{};
    for(size_t i=0;i<7;++i)tau[i]=std::clamp(kp_[i]*(desired_[i]-measured_[i])+kd_[i]*(velocity_ff_*ff_velocity_[i]-dq_[i]),-60.,60.);
    return tau;
  }
  const Joints& position()const noexcept{return desired_;}
  const Joints& velocity()const noexcept{return velocity_;}
};
}
