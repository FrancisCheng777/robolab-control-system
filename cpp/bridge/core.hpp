#pragma once
#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <ruckig/ruckig.hpp>

namespace pi05 {
using Joints = std::array<double, 7>;
using json = nlohmann::json;

// Exactly one producer and one consumer per queue. No locks, allocation or
// overwrites of unread values. Release/acquire owns each slot (no seqlock races).
template <typename T, std::size_t N> class SpscQueue {
  std::array<T, N> slots_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
 public:
  bool push(const T& value) noexcept {
    const auto h = head_.load(std::memory_order_relaxed), next = (h + 1) % N;
    if (next == tail_.load(std::memory_order_acquire)) return false;
    slots_[h] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(T& value) noexcept {
    const auto t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;
    value = slots_[t];
    tail_.store((t + 1) % N, std::memory_order_release);
    return true;
  }
};

struct Config {
  json raw;
  Joints lower{}, upper{}, delta{}, velocity{}, acceleration{}, jerk{}, impedance{}, tracking{}, torque{};
  std::array<double, 6> force{};
  double command_timeout{}, state_timeout{}, gripper_max{}, gripper_speed{}, gripper_force{}, deadband{};
  double max_feedback_gap{.002};
  bool robolab_torque{false};
  std::uint64_t state_lag{};
  explicit Config(json value) : raw(std::move(value)) {
    const auto controller=raw.value("teleop_controller",std::string("legacy_position"));
    if(controller!="legacy_position" && controller!="robolab_torque")throw std::invalid_argument("unknown teleop controller");
    robolab_torque = controller=="robolab_torque";
    max_feedback_gap = raw.value("max_feedback_gap_s", .002);
    if (!std::isfinite(max_feedback_gap) || max_feedback_gap < .001 || max_feedback_gap > .010)
      throw std::invalid_argument("max_feedback_gap_s must be within [0.001, 0.010]");
    const auto model = raw.at("robot_model").get<std::string>();
    if (model != "panda" && model != "fr3") throw std::invalid_argument("robot_model must be panda or fr3");
    if (raw.at("action_contract_id").get<std::string>().empty()) throw std::invalid_argument("empty action contract");
    lower = joints("joint_lower"); upper = joints("joint_upper");
    delta = positive_joints("joint_delta_scale"); velocity = positive_joints("max_velocity");
    acceleration = positive_joints("max_acceleration"); jerk = positive_joints("max_jerk");
    impedance = positive_joints("joint_impedance"); tracking = positive_joints("max_tracking_error");
    torque = positive_joints("collision_torque");
    if (!raw.at("collision_force").is_array() || raw.at("collision_force").size() != 6)
      throw std::invalid_argument("collision_force must have six elements");
    force = raw.at("collision_force").get<std::array<double, 6>>();
    for (double f : force) positive(f, "collision_force");
    for (std::size_t i = 0; i < 7; ++i)
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
        throw std::invalid_argument("invalid joint bounds");
    command_timeout = positive(raw.at("command_timeout_s").get<double>(), "command_timeout_s");
    state_timeout = positive(raw.at("state_timeout_s").get<double>(), "state_timeout_s");
    if (command_timeout > 1 || state_timeout > .2) throw std::invalid_argument("unsafe watchdog timeout");
    state_lag = raw.at("max_state_sequence_lag").get<std::uint64_t>();
    if (state_lag == 0 || state_lag > 200) throw std::invalid_argument("invalid state sequence lag");
    gripper_max = positive(raw.at("gripper_max_width").get<double>(), "gripper_max_width");
    gripper_speed = positive(raw.at("gripper_speed").get<double>(), "gripper_speed");
    gripper_force = positive(raw.at("gripper_force").get<double>(), "gripper_force");
    deadband = positive(raw.at("gripper_command_deadband").get<double>(), "gripper_command_deadband");
    positive(raw.at("gripper_grasp_epsilon_inner").get<double>(), "gripper_grasp_epsilon_inner");
    positive(raw.at("gripper_grasp_epsilon_outer").get<double>(), "gripper_grasp_epsilon_outer");
  }
  static double positive(double x, const char* name) {
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument(std::string(name) + " must be positive/finite");
    return x;
  }
  Joints positive_joints(const char* name) const {
    auto out = joints(name);
    for (double x : out) positive(x, name);
    return out;
  }
  Joints joints(const char* name) const {
    const auto& value = raw.at(name);
    if (!value.is_array() || value.size() != 7) throw std::invalid_argument(std::string(name) + " must have seven elements");
    return value.get<Joints>();
  }
  bool tracking_error_exceeded(const Joints& q,const Joints& reference) const {
    // This guard belongs to the position-command backend. RoboLab torque PD
    // permits reference lag while catching up, with bounded torques and steps.
    if(robolab_torque)return false;
    for(std::size_t i=0;i<7;++i)
      if(std::abs(q[i]-reference[i])>tracking[i])return true;
    return false;
  }
  void check_position(const Joints& q) const {
    for (std::size_t i = 0; i < 7; ++i)
      if (!std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i])
        throw std::runtime_error("joint soft limit violated");
  }
};

enum class CommandMode { Track, Hold, Home };
struct Command { Joints target{}; double received_s{}, width{}; std::uint64_t sequence{}; CommandMode mode{CommandMode::Track}; };
struct State { Joints q{}, dq{}; double timestamp_s{}; std::uint64_t sequence{}; };

// Jerk/acceleration/velocity-constrained target tracking. Stop switches to
// velocity control with target velocity zero, preserving braking continuity.
// Limits must be supplied for the actual robot/tool; no guessed hardware limits.
class Motion {
  const Config& cfg_;
  ruckig::Ruckig<7> otg_{.001};
  ruckig::InputParameter<7> in_;
  ruckig::OutputParameter<7> out_;
  bool stopping_{false};
  bool holding_{false}, hold_ready_{false};
 public:
  explicit Motion(const Config& cfg, const Joints& q, const Joints& dq = {}, const Joints& ddq = {}) : cfg_(cfg) {
    cfg_.check_position(q);
    in_.current_position = in_.target_position = q;
    in_.current_velocity = dq; in_.current_acceleration = ddq;
    in_.target_velocity.fill(0); in_.target_acceleration.fill(0);
    in_.max_velocity = cfg.velocity; in_.max_acceleration = cfg.acceleration; in_.max_jerk = cfg.jerk;
  }
  void target(const Joints& q, bool home = false) {
    cfg_.check_position(q);
    if (stopping_) return;  // a shutdown/fault can never be re-armed
    if (holding_ && !hold_ready_) throw std::runtime_error("wait for hold to finish before resuming");
    holding_ = hold_ready_ = false;
    in_.control_interface = ruckig::ControlInterface::Position;
    in_.target_position = q;
    for (std::size_t i = 0; i < 7; ++i) {
      in_.max_velocity[i] = home ? std::min(cfg_.velocity[i], .06) : cfg_.velocity[i];
      in_.max_acceleration[i] = home ? std::min(cfg_.acceleration[i], .15) : cfg_.acceleration[i];
      in_.max_jerk[i] = home ? std::min(cfg_.jerk[i], 1.) : cfg_.jerk[i];
    }
  }
  void hold() noexcept {
    if (stopping_) return;
    if (!holding_) hold_ready_ = false;
    holding_ = true;
    in_.control_interface = ruckig::ControlInterface::Velocity;
    in_.target_velocity.fill(0); in_.target_acceleration.fill(0);
  }
  bool hold_ready() const noexcept { return holding_ && hold_ready_ && !stopping_; }
  void stop() noexcept {
    stopping_ = true;
    in_.control_interface = ruckig::ControlInterface::Velocity;
    in_.target_velocity.fill(0); in_.target_acceleration.fill(0);
  }
  void synchronize(const Joints& q, const Joints& dq, const Joints& ddq) {
    in_.current_position = q; in_.current_velocity = dq; in_.current_acceleration = ddq;
  }
  bool update(double feedback_gap = .001) {
    if (!std::isfinite(feedback_gap) || feedback_gap <= 0 || feedback_gap > cfg_.max_feedback_gap) {
      throw std::runtime_error("feedback gap exceeds configured limit: " +
                               std::to_string(feedback_gap * 1000.) + " ms");
    }
    // real_control synchronizes to the CURRENT q_d/dq_d/ddq_d, including the
    // robot's extrapolation during missed packets. Predict the NEXT 1-ms
    // command, never replay the elapsed gap as an additional forward jump.
    otg_.delta_time = .001;
    const auto result = otg_.update(in_, out_);
    if (result < 0) throw std::runtime_error("trajectory generation failed");
    cfg_.check_position(out_.new_position);
    out_.pass_to_input(in_);
    hold_ready_ = holding_ && result == ruckig::Result::Finished;
    return stopping_ && result == ruckig::Result::Finished;
  }
  const Joints& position() const noexcept { return out_.new_position; }
  const Joints& velocity() const noexcept { return out_.new_velocity; }
  const Joints& acceleration() const noexcept { return out_.new_acceleration; }
};
}  // namespace pi05
