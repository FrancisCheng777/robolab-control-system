// Local Unix-socket bridge. Run beside the robot on a PREEMPT_RT Linux host.
// Only this C++ process owns the FCI connection. No automatic recovery/homing.
#include "core.hpp"
#include "robolab_motion.hpp"
#include <type_traits>
#include "tool_configuration.hpp"
#include "thread_placement.hpp"
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#ifdef PI05_WITH_FRANKA
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/log.h>
#include "fault_history.hpp"
#include "robotiq_gripper.h"
#include <algorithm>
#include <franka/control_types.h>
#endif

using namespace pi05;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Shared {
  SpscQueue<Command, 8> commands;
  SpscQueue<State, 4096> states;
  std::atomic<bool> stop{false}, stopped{false}, ready{false}, armed{false};
  std::atomic<int> fault{0};
  std::atomic<double> width{0}, gripper_time{0}, grip_target{0};
  std::atomic<std::uint64_t> grip_sequence{0};
  std::atomic<bool> holding{false}, arm_held{false}, grip_hold{false};
  std::atomic<std::uint64_t> grip_stop_epoch{0}, grip_stop_ack{0}, applied_sequence{0};
  std::atomic<std::uint64_t> callback_count{0}, callback_gap_count{0};
  std::atomic<double> max_callback_gap_s{0}, command_success_rate{0};
};
bool hold_ready(const Shared& s) {
  return s.holding && s.arm_held && s.grip_hold &&
      s.grip_stop_ack.load() >= s.grip_stop_epoch.load();
}
static_assert(std::atomic<double>::is_always_lock_free, "RT bridge requires lock-free doubles");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "RT bridge requires lock-free counters");

// Called from the RT loop, with fixed-size messages only.
template<class Controller>
bool update_commands(Shared& s, Controller& motion, double& last_command, const Config& cfg) {
  Command command;
  while (s.commands.pop(command)) {
    // Dequeued commands have their original arrival time; stalled processing
    // must never turn an old command into a fresh watchdog heartbeat.
    if (now_s() - command.received_s > cfg.command_timeout) {
      s.fault = 1; s.stop = true;
    }
    if (!s.stop.load()) {
      if (command.mode == CommandMode::Hold) motion.hold();
      else motion.target(command.target, command.mode == CommandMode::Home);
      s.holding = command.mode == CommandMode::Hold;
      s.arm_held = motion.hold_ready();
      last_command = command.received_s;
      s.armed = true;
      const bool freeze_gripper = command.mode != CommandMode::Track;
      if (freeze_gripper && !s.grip_hold.load()) s.grip_stop_epoch = command.sequence;
      s.grip_hold = freeze_gripper;
      s.grip_target.store(command.width);
      s.grip_sequence.store(command.sequence);
      s.applied_sequence = command.sequence;
    }
  }
  if (s.armed && now_s() - last_command > cfg.command_timeout) { s.fault = 1; s.stop = true; }
  if (s.stop) motion.stop();
  return s.stop;
}

template<class Controller>
void mock_control_impl(Shared& s, const Config& cfg) {
  Joints q{};
  for (std::size_t i = 0; i < 7; ++i) q[i] = (cfg.lower[i] + cfg.upper[i]) / 2;
  Controller motion(cfg, q);
  double last = 0;
  std::uint64_t count = 0;
  auto deadline = std::chrono::steady_clock::now();
  s.width = cfg.gripper_max;
  while (!s.stopped) {
    if constexpr(std::is_same_v<Controller,RobolabMotion>)
      motion.synchronize(motion.position(),motion.velocity());
    update_commands(s, motion, last, cfg);
    const bool finished = motion.update();
    s.arm_held = motion.hold_ready();
    if (s.grip_hold) s.grip_stop_ack = s.grip_stop_epoch.load();
    else if (s.armed && !s.stop) s.width = s.grip_target.load();
    s.gripper_time = now_s();
    s.states.push({motion.position(), motion.velocity(), now_s(), ++count});
    s.ready = true;
    if (finished) break;
    deadline += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(deadline);
  }
  s.stopped = true;
}

void mock_control(Shared& s, const Config& cfg) {
  if(cfg.raw.value("teleop_controller",std::string())=="robolab_torque") mock_control_impl<RobolabMotion>(s,cfg);
  else mock_control_impl<Motion>(s,cfg);
}

#ifdef PI05_WITH_FRANKA
void real_control(Shared& s, const Config& cfg, const std::string& ip) {
  if (!cfg.raw.value("hardware_verified", false))
    throw std::runtime_error("complete payload, gripper and tool calibration before real control");
  if (cfg.raw.at("gripper_type") != "robotiq_2f85")
    throw std::runtime_error("this fork requires Robotiq 2F-85");
  const int open_pos=cfg.raw.at("robotiq_open_position").get<int>();
  const int closed_pos=cfg.raw.at("robotiq_closed_position").get<int>();
  if (open_pos<0 || closed_pos>255 || open_pos>=closed_pos)
    throw std::runtime_error("invalid calibrated Robotiq position endpoints");
  if (cfg.gripper_speed>255 || cfg.gripper_force>255 ||
      std::floor(cfg.gripper_speed)!=cfg.gripper_speed || std::floor(cfg.gripper_force)!=cfg.gripper_force)
    throw std::runtime_error("Robotiq speed/force are integer register values 1..255");
  franka::Robot robot(ip, franka::RealtimeConfig::kEnforce);
  const auto tool_state = robot.readOnce();
  require_desk_tool(cfg.raw.at("expected_desk_tool"),
                   {{"mass", tool_state.m_ee}, {"centerOfMass", tool_state.F_x_Cee},
                    {"inertia", tool_state.I_ee}, {"transformation", tool_state.F_T_NE},
                    {"activeTransformation", tool_state.F_T_EE}});
  robot.setCollisionBehavior(cfg.torque, cfg.torque, cfg.torque, cfg.torque,
                             cfg.force, cfg.force, cfg.force, cfg.force);
  robot.setJointImpedance(cfg.impedance);
  // Additional held payload only; the gripper belongs in Desk's m_ee model.
  robot.setLoad(cfg.raw.at("load_mass").get<double>(),
                cfg.raw.at("load_center").get<std::array<double, 3>>(),
                cfg.raw.at("load_inertia").get<std::array<double, 9>>());
  robotiq::Robotiq2FGripper gripper(cfg.raw.at("gripper_device").get<std::string>());
  if (!gripper.connect() || !gripper.activate()) throw std::runtime_error("Robotiq activation failed");
  const auto read_width = [&](bool require_stopped = false) {
    const auto gs=gripper.get_status();
    if (!gs.activated || gs.status!=3 || gs.fault!=0)
      throw std::runtime_error("Robotiq not ready/fault");
    if (require_stopped && gs.goto_requested)
      throw std::runtime_error("Robotiq did not confirm hold stop");
    return std::clamp(cfg.gripper_max*(closed_pos-double(gs.position))/(closed_pos-open_pos),0.,cfg.gripper_max);
  };
  s.width=read_width(); s.gripper_time=now_s();
  // A single worker owns all Modbus transactions. No concurrent reader/mover/stopper.
  // Moves are non-blocking register writes; the worker checks stop between transactions.
  std::thread worker([&] {
    try {
      std::uint64_t seen=0;
      double previous=s.width.load();
      while (!s.stop) {
        const auto seq=s.grip_sequence.load();
        if(seq!=seen) {
          seen=seq; const double width=s.grip_target.load();
          if (s.grip_hold) {
            const auto epoch = s.grip_stop_epoch.load();
            if (s.grip_stop_ack.load() < epoch) {
              if (!gripper.stop()) throw std::runtime_error("Robotiq hold stop failed");
              s.width = read_width(true);
              s.gripper_time = now_s();
              s.grip_stop_ack = epoch;
              previous = -1.; // the next explicit follow command must resume a stopped move
            }
          } else if(std::abs(width-previous)>=cfg.deadband) {
            const auto pos=static_cast<uint8_t>(std::lround(
                closed_pos-width/cfg.gripper_max*(closed_pos-open_pos)));
            if(!gripper.move(pos,static_cast<uint8_t>(cfg.gripper_speed),
                             static_cast<uint8_t>(cfg.gripper_force)))
              throw std::runtime_error("Robotiq write failed");
            previous=width;
          }
        }
        s.width=read_width();s.gripper_time=now_s();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch(const std::exception& e) {
      std::cerr << e.what() << std::endl;s.fault=3;s.stop=true;
    }
    if(!gripper.stop()) s.fault=3;
  });
  const auto cleanup=[&] {s.stop=true;if(worker.joinable())worker.join();};
  try {
    const int control_cpu = pin_control_thread_from_env();
    if (control_cpu >= 0) std::cout << "Franka control CPU=" << control_cpu << "; non-RT workers retain launcher affinity" << std::endl;
    if(cfg.raw.value("teleop_controller",std::string())=="robolab_torque") {
      const auto initial=robot.readOnce();
      RobolabMotion motion(cfg,initial.q);
      double last=0;std::uint64_t count=0;bool first=true;
      robot.control([&](const franka::RobotState& state,franka::Duration period)->franka::Torques {
        s.states.push({state.q,state.dq,now_s(),++count});s.ready=true;
        motion.synchronize(state.q,state.dq);
        const double gap=first?.001:period.toSec();first=false;
        ++s.callback_count;if(gap>.001001)++s.callback_gap_count;
        if(gap>s.max_callback_gap_s)s.max_callback_gap_s=gap;
        s.command_success_rate=state.control_command_success_rate;
        cfg.check_position(state.q);
        if(cfg.tracking_error_exceeded(state.q,motion.position())){s.fault=4;s.stop=true;}
        update_commands(s,motion,last,cfg);
        const bool finished=motion.update(gap);s.arm_held=motion.hold_ready();
        franka::Torques output(motion.torques());
        return finished?franka::MotionFinished(output):output;
      },true,franka::kMaxCutoffFrequency);
    } else {
    std::unique_ptr<Motion> motion;
    double last = 0;
    std::uint64_t count = 0;
    // Allocate trajectory objects before entering the RT callback.
    const auto initial = robot.readOnce();
    motion = std::make_unique<Motion>(cfg, initial.q_d, initial.dq_d, initial.ddq_d);
    bool first = true;
    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
      s.states.push({state.q, state.dq, now_s(), ++count});
      s.ready = true;
      motion->synchronize(state.q_d, state.dq_d, state.ddq_d);
      if (first) { first = false; return franka::JointPositions(state.q_d); }
      const double gap = period.toSec();
      ++s.callback_count;
      if (gap > .001001) ++s.callback_gap_count;
      if (gap > s.max_callback_gap_s.load()) s.max_callback_gap_s = gap;
      s.command_success_rate = state.control_command_success_rate;
      cfg.check_position(state.q);
      if(cfg.tracking_error_exceeded(state.q,state.q_d)) {s.fault=4;s.stop=true;}
      update_commands(s, *motion, last, cfg);
      const bool finished = motion->update(period.toSec());
      s.arm_held = motion->hold_ready();
      franka::JointPositions output(motion->position());
      return finished ? franka::MotionFinished(output) : output;
    }, franka::ControllerMode::kJointImpedance, true, franka::kMaxCutoffFrequency);
    }
    cleanup();
  } catch (...) {
    // Publish the control fault BEFORE potentially blocking Modbus shutdown.
    // Otherwise the client sees stale arm state while the real cause is hidden.
    s.fault=2;s.ready=false;s.stop=true;
    cleanup();throw;
  }
  s.stopped = true;
}
#endif

std::string fault_text(int code) {
  switch (code) {
    case 0: return "";
    case 1: return "command watchdog expired; restart the bridge for a new trial";
    case 3: return "gripper operation/feedback failed";
    case 4: return "joint tracking error exceeded limit";
    default: return "libfranka/trajectory exception; see bridge stderr";
  }
}

class Server {
  int listener_{-1};
  const std::string path_, session_;
  const Config& cfg_;
  Shared& shared_;
  const bool mock_;
  State latest_{};
  std::uint64_t accepted_{0};
  std::uint64_t first_hold_sequence_{0};
  CommandMode last_mode_{CommandMode::Track};
  std::string owner_;
 public:
  Server(const std::string& path, const Config& cfg, Shared& shared, bool mock)
      : path_(path), session_(std::to_string(getpid()) + "-" + std::to_string(now_s())),
        cfg_(cfg), shared_(shared), mock_(mock) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path)) throw std::invalid_argument("socket path is too long");
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    listener_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listener_ < 0) throw std::runtime_error("socket creation failed");
    // Never unlink an existing socket or other file on startup.
    if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      close(listener_); listener_ = -1;
      throw std::runtime_error("socket bind failed (path may already exist)");
    }
    if (chmod(path.c_str(), 0600) != 0 || listen(listener_, 4) != 0) {
      close(listener_); unlink(path.c_str()); listener_ = -1;
      throw std::runtime_error("socket permissions/listen failed");
    }
  }
  ~Server() { if (listener_ >= 0) { close(listener_); unlink(path_.c_str()); } }
  void drain() { State sample; while (shared_.states.pop(sample)) latest_ = sample; }
  json handle(const json& request) {
    drain();
    if (request.at("protocol") != 1) throw std::invalid_argument("unsupported protocol");
    const std::string op = request.at("op");
    if (op == "state") {
      const double gripper_time = shared_.gripper_time.load();
      return {{"ok", true}, {"protocol", 1}, {"mock", mock_}, {"session", session_},
      {"ready", shared_.ready.load()}, {"armed", shared_.armed.load()}, {"stopped", shared_.stopped.load()}, {"stopping", shared_.stop.load()},
      {"claimed", !owner_.empty()},
      {"session_control_version", 1}, {"holding", shared_.holding.load()},
      {"hold_ready", hold_ready(shared_)}, {"applied_sequence", shared_.applied_sequence.load()},
      {"fault", fault_text(shared_.fault.load())}, {"q", latest_.q}, {"dq", latest_.dq},
      {"state_sequence", latest_.sequence}, {"state_age_s", now_s() - latest_.timestamp_s},
      {"state_bridge_receive_s", latest_.timestamp_s}, {"gripper_bridge_read_s", gripper_time},
      {"timestamp_clock", "host_monotonic_seconds"},
      {"gripper_width", shared_.width.load()}, {"gripper_age_s", now_s() - gripper_time},
      {"callback_count", shared_.callback_count.load()},
      {"callback_gap_count", shared_.callback_gap_count.load()},
      {"max_callback_gap_s", shared_.max_callback_gap_s.load()},
      {"control_command_success_rate", shared_.command_success_rate.load()},
      {"last_sequence", accepted_}, {"calibration", cfg_.raw}};
    }
    if (request.at("session") != session_) throw std::invalid_argument("wrong bridge session");
    const auto owner = request.at("controller_id").get<std::string>();
    if (op == "claim") {
      if (owner.size() != 32 || shared_.stop || shared_.stopped || !shared_.ready || shared_.fault != 0)
        throw std::invalid_argument("invalid claim or unavailable bridge");
      if (!owner_.empty() && owner_ != owner) throw std::invalid_argument("bridge already claimed");
      owner_ = owner;
      return {{"ok", true}};
    }
    if (owner_.empty() || owner != owner_) throw std::invalid_argument("controller does not own this bridge");
    if (op == "stop") { shared_.stop = true; return {{"ok", true}}; }
    if (op != "command" && op != "hold") throw std::invalid_argument("unknown operation");
    if (!shared_.ready || shared_.stop || shared_.stopped || shared_.fault != 0)
      throw std::runtime_error("bridge is stopped/unready; commands cannot re-arm it");
    if (!request.at("sequence").is_number_unsigned() || !request.at("state_sequence").is_number_unsigned())
      throw std::invalid_argument("sequence numbers must be unsigned integers");
    const auto seq = request.at("sequence").get<std::uint64_t>();
    const auto state_seq = request.at("state_sequence").get<std::uint64_t>();
    if (seq != accepted_ + 1) throw std::invalid_argument("duplicate/out-of-order command");
    if (state_seq > latest_.sequence || latest_.sequence - state_seq > cfg_.state_lag ||
        now_s() - latest_.timestamp_s > cfg_.state_timeout)
      throw std::invalid_argument("stale robot state");
    const auto profile = request.value("motion_profile", std::string("teleop"));
    if (profile != "teleop" && profile != "home") throw std::invalid_argument("unknown motion profile");
    const auto mode = op == "hold" ? CommandMode::Hold :
        (profile == "home" ? CommandMode::Home : CommandMode::Track);
    if (mode != CommandMode::Hold && last_mode_ == CommandMode::Hold &&
        (!hold_ready(shared_) || shared_.applied_sequence.load() < first_hold_sequence_))
      throw std::invalid_argument("wait until arm and gripper are held before resuming");
    if (mode == CommandMode::Home && last_mode_ == CommandMode::Track)
      throw std::invalid_argument("home profile requires a completed hold first");
    Joints target = latest_.q;
    double width = shared_.width.load();
    if (mode != CommandMode::Hold) {
    if (!request.at("q_target").is_array() || request.at("q_target").size() != 7)
      throw std::invalid_argument("q_target must have seven elements");
    target = request.at("q_target").get<Joints>();
    cfg_.check_position(target);
    if(cfg_.raw.value("teleop_controller",std::string())!="robolab_torque")
    for (std::size_t i = 0; i < 7; ++i)
      if (std::abs(target[i] - latest_.q[i]) > cfg_.delta[i] + cfg_.velocity[i] * cfg_.state_timeout)
        throw std::invalid_argument("target exceeds the per-command joint delta limit");
    width = request.at("gripper_width");
    if (!std::isfinite(width) || width < 0 || width > cfg_.gripper_max)
      throw std::invalid_argument("invalid gripper width");
    }
    const double enqueued = now_s();
    if (!shared_.commands.push({target, enqueued, width, seq, mode})) throw std::runtime_error("command mailbox is full");
    if (mode == CommandMode::Hold && last_mode_ != CommandMode::Hold) first_hold_sequence_ = seq;
    last_mode_ = mode;
    accepted_ = seq;
    return {{"ok", true}, {"sequence", seq}, {"command_enqueue_s", enqueued}};
  }
  void poll_once() {
    drain();
    pollfd descriptor{listener_, POLLIN, 0};
    if (poll(&descriptor, 1, 10) <= 0) return;
    const int client = accept(listener_, nullptr, nullptr);
    if (client < 0) return;
    timeval timeout{0, 50000};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    json reply;
    try {
      std::string bytes;
      char buffer[1024];
      while (bytes.find('\n') == std::string::npos && bytes.size() < 8192) {
        const auto count = recv(client, buffer, sizeof(buffer), 0);
        if (count <= 0) throw std::runtime_error("incomplete request or read timeout");
        bytes.append(buffer, static_cast<std::size_t>(count));
      }
      if (bytes.size() >= 8192) throw std::invalid_argument("request too large");
      reply = handle(json::parse(bytes.substr(0, bytes.find('\n'))));
    } catch (const std::exception& e) { reply = {{"ok", false}, {"error", e.what()}}; }
    const auto data = reply.dump() + "\n";
    std::size_t written = 0;
    while (written < data.size()) {
      const auto count = send(client, data.data() + written, data.size() - written, 0);
      if (count <= 0) break;
      written += static_cast<std::size_t>(count);
    }
    close(client);
  }
};
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string config_path, socket_path = "/tmp/pi05-franka.sock", robot_ip, fault_log_path;
    bool mock = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--mock") mock = true;
      else if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
      else if (arg == "--socket" && i + 1 < argc) socket_path = argv[++i];
      else if (arg == "--robot" && i + 1 < argc) robot_ip = argv[++i];
      else if (arg == "--fault-log" && i + 1 < argc) fault_log_path = argv[++i];
      else throw std::invalid_argument("usage: pi05_franka_bridge --config CALIBRATION.json --robot IP [--socket PATH] [--fault-log PATH] [--mock]");
    }
    if (config_path.empty() || (!mock && robot_ip.empty()) || (mock && !robot_ip.empty()))
      throw std::invalid_argument("supply --config and either --robot IP or explicitly --mock");
#ifndef PI05_WITH_FRANKA
    if (!mock) throw std::runtime_error("built without libfranka; only explicit --mock is available");
#endif
    std::ifstream file(config_path);
    if (!file) throw std::runtime_error("cannot read calibration file");
    const Config cfg(json::parse(file));
    Shared shared;
    Server server(socket_path, cfg, shared, mock);
    std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal); std::signal(SIGPIPE, SIG_IGN);
    std::thread control([&] {
      try {
        if (mock) mock_control(shared, cfg);
#ifdef PI05_WITH_FRANKA
        else real_control(shared, cfg, robot_ip);
#endif
      } catch (const std::exception& e) {
        shared.fault = 2; shared.stop = true; shared.stopped = true;
        std::cerr << "control stopped: " << e.what() << '\n';
#ifdef PI05_WITH_FRANKA
        // Serialize libfranka's existing exception history only after control exits.
        // No disk IO or extra allocations are added to the real-time callback.
        if (const auto* control_error = dynamic_cast<const franka::ControlException*>(&e)) {
          if (!fault_log_path.empty()) try {
            std::ofstream fault_log(fault_log_path);
            fault_log.exceptions(std::ios::failbit | std::ios::badbit);
            fault_log << franka::logToCSV(control_error->log);
            fault_log.close();
            // libfranka 0.9 CSV omits Cartesian wrench: save it explicitly too.
            std::ofstream wrench_log(fault_log_path + ".wrench.json");
            wrench_log.exceptions(std::ios::failbit | std::ios::badbit);
            wrench_log << fault_wrench_history(control_error->log).dump(2) << '\n';
            wrench_log.close();
            std::cerr << "Franka fault history: " << fault_log_path
                      << " (" << control_error->log.size() << " samples)\n";
          } catch (const std::exception& log_error) {
            std::cerr << "Could not save Franka fault history: " << log_error.what() << '\n';
          }
        }
#endif
      }
    });
    std::cout << "bridge listening at " << socket_path << " mock=" << mock << std::endl;
    try { while (!interrupted) server.poll_once(); }
    catch (...) { shared.stop = true; control.join(); throw; }
    shared.stop = true;
    control.join();
    return shared.fault == 0 ? 0 : 1;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
