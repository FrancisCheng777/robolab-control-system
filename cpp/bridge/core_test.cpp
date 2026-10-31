#include "core.hpp"
#include "robolab_motion.hpp"
#include "tool_configuration.hpp"
#include <iostream>
#include <thread>
#include <limits>

using namespace pi05;
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
json fixture() {
  // Synthetic test limits; never a physical-robot calibration.
  return {{"robot_model", "panda"}, {"action_contract_id", "test-only"},
    {"joint_lower", Joints{-2,-2,-2,-2,-2,-2,-2}}, {"joint_upper", Joints{2,2,2,2,2,2,2}},
    {"joint_delta_scale", Joints{.1,.1,.1,.1,.1,.1,.1}}, {"max_velocity", Joints{.5,.5,.5,.5,.5,.5,.5}},
    {"max_acceleration", Joints{1,1,1,1,1,1,1}}, {"max_jerk", Joints{10,10,10,10,10,10,10}},
    {"joint_impedance", Joints{100,100,100,100,100,100,100}}, {"max_tracking_error", Joints{.1,.1,.1,.1,.1,.1,.1}},
    {"collision_torque", Joints{1,1,1,1,1,1,1}}, {"collision_force", std::array<double,6>{1,1,1,1,1,1}},
    {"command_timeout_s", .3}, {"state_timeout_s", .05}, {"max_state_sequence_lag", 50},
    {"gripper_max_width", .08}, {"gripper_speed", .01}, {"gripper_force", 1},
    {"gripper_command_deadband", .001}, {"gripper_grasp_epsilon_inner", .005}, {"gripper_grasp_epsilon_outer", .005}};
}
int main() {
  try {
    const json tool = {{"mass", .921}, {"centerOfMass", {0,0,.06}},
      {"inertia", {.0012,0,0,0,.002,0,0,0,.0011}},
      {"transformation", {1,0,0,0,0,1,0,0,0,0,1,0,0,0,.174,1}}};
    auto feedback = tool;
    feedback["activeTransformation"] = tool["transformation"];
    require_desk_tool(tool, feedback);
    for (const auto& key : {"mass", "inertia", "transformation", "activeTransformation"}) {
      auto wrong = feedback;
      if (std::string(key) == "mass") wrong[key] = .73;
      else if (std::string(key) == "inertia") wrong[key][0] = 1200.;
      else wrong[key][14] = .1034;
      bool denied = false;
      try { require_desk_tool(tool, wrong); } catch (...) { denied = true; }
      require(denied, "wrong Desk tool or TCP was not rejected");
    }
    const Config cfg(fixture());
    auto torque_fixture=fixture();torque_fixture["teleop_controller"]="robolab_torque";
    const Config torque_config(torque_fixture);
    Joints lagged{};lagged.fill(.2);
    require(cfg.tracking_error_exceeded(Joints{},lagged),"position path must retain tracking guard");
    require(!torque_config.tracking_error_exceeded(Joints{},lagged),"RoboLab torque path must not inherit position tracking abort");

    auto rc=fixture();rc["max_feedback_gap_s"]=.01;const Config rconfig(rc);
    RobolabMotion leader(rconfig,Joints{});
    Joints far{};far.fill(1.);leader.target(far);
    leader.update(.007);
    require(std::abs(leader.position()[0]-.0005)<1e-12,"RoboLab must take one bounded step even after a gap");
    require(std::abs(leader.torques()[0]-.06)<1e-12,"RoboLab torque PD gain mismatch");
    leader.hold();const auto held=leader.position();
    for(int i=0;i<100;++i)leader.update();
    require(leader.position()==held && leader.hold_ready(),"hold must cancel distant target");
    far.fill(-1.);leader.target(far);leader.update();
    require(std::abs(leader.position()[0])<1e-12,"resume must preserve absolute direction without offset");
    leader.stop();leader.target(far);require(leader.update(),"shutdown must not rearm");
    RobolabMotion late_callback(rconfig,Joints{});
    late_callback.target(Joints{1,1,1,1,1,1,1});
    late_callback.update(.011);
    require(std::abs(late_callback.position()[0]-.0005)<1e-12,
            "RoboLab must use one bounded callback step, not reject or replay duration");
    bool robolab_rejected=false;
    try{late_callback.update(std::numeric_limits<double>::quiet_NaN());}catch(...){robolab_rejected=true;}
    require(robolab_rejected,"nonfinite callback interval must be rejected");

    auto fast_home_fixture=fixture();fast_home_fixture["home_max_velocity"]=.18;
    const Config fast_home_cfg(fast_home_fixture);
    RobolabMotion fast_home(fast_home_cfg,Joints{});
    fast_home.target(Joints{1,1,1,1,1,1,1},true);fast_home.update();
    require(std::abs(fast_home.position()[0]-.00018)<1e-12,"configured home speed not applied");
    auto capped_fixture=fast_home_fixture;capped_fixture["max_velocity"]=Joints{.1,.1,.1,.1,.1,.1,.1};
    const Config capped_cfg(capped_fixture);RobolabMotion capped_home(capped_cfg,Joints{});
    capped_home.target(Joints{1,1,1,1,1,1,1},true);capped_home.update();
    require(std::abs(capped_home.position()[0]-.0001)<1e-12,"home must respect per-joint velocity ceiling");
    for (double invalid : {0., -.1, .21, std::numeric_limits<double>::quiet_NaN()}) {
      auto bad=fixture();bad["home_max_velocity"]=invalid;const Config bad_cfg(bad);bool rejected=false;
      try{RobolabMotion motion(bad_cfg,Joints{});}catch(...){rejected=true;}
      require(rejected,"invalid home speed was accepted");
    }

    // A received state already includes the elapsed time/extrapolation on the
    // robot. The next command must remain one 1-ms step from that state.
    auto trial_fixture = fixture();
    trial_fixture["max_feedback_gap_s"] = .008;
    const Config trial_cfg(trial_fixture);
    for (double gap : {.001, .002, .003, .005, .007, .008}) {
      Motion delayed(trial_cfg, Joints{});
      delayed.target(Joints{1,1,1,1,1,1,1}, true);
      Joints q{}, dq{}, ddq{};
      q.fill(.02); dq.fill(.02); ddq.fill(.05);
      delayed.synchronize(q, dq, ddq);
      delayed.update(gap);
      const double step = delayed.position()[0] - q[0];
      require(step > .000019 && step < .000021, "missed period was replayed into next command");
      require(std::abs(delayed.acceleration()[0] - ddq[0]) <= .001001,
              "recovery jerk exceeded one-millisecond bound");
      require(std::abs(delayed.velocity()[0]) <= .060001, "recovery home velocity bound");
    }
    for (double gap : {0., -.001, .009, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
      Motion delayed(trial_cfg, Joints{});
      bool denied = false;
      try { delayed.update(gap); } catch (...) { denied = true; }
      require(denied, "invalid or prolonged callback gap was accepted");
    }
    auto ceiling_fixture = fixture(); ceiling_fixture["max_feedback_gap_s"] = .010;
    const Config ceiling_cfg(ceiling_fixture);
    for (double gap : {.009, .010}) {
      Motion delayed(ceiling_cfg, Joints{});
      delayed.target(Joints{1,1,1,1,1,1,1}, true);
      Joints q{}, dq{}, ddq{}; q.fill(.02); dq.fill(.02); ddq.fill(.05);
      delayed.synchronize(q, dq, ddq); delayed.update(gap);
      require(delayed.position()[0] - q[0] < .000021, "10ms ceiling replayed elapsed movement");
      require(std::abs(delayed.acceleration()[0] - ddq[0]) <= .001001, "10ms recovery jerk bound");
    }
    { Motion delayed(ceiling_cfg, Joints{}); bool denied=false;
      try { delayed.update(.011); } catch (...) { denied=true; }
      require(denied, "gap above 10ms ceiling was accepted"); }
    for (double limit : {.0005, .011, std::numeric_limits<double>::infinity()}) {
      auto invalid = fixture(); invalid["max_feedback_gap_s"] = limit;
      bool denied = false;
      try { Config bad(invalid); } catch (...) { denied = true; }
      require(denied, "unsafe callback gap configuration was accepted");
    }
    { Motion default_motion(cfg, Joints{}); bool denied = false;
      try { default_motion.update(.003); } catch (...) { denied = true; }
      require(denied, "default callback gap limit changed without a trial profile"); }
    Motion motion(cfg, Joints{});
    motion.target(Joints{.1,.1,.1,.1,.1,.1,.1});
    double last_acc = 0;
    for (int i = 0; i < 300; ++i) {
      motion.update();
      require(std::abs(motion.velocity()[0]) <= .500001, "velocity bound");
      require(std::abs(motion.acceleration()[0]) <= 1.000001, "acceleration bound");
      require(std::abs(motion.acceleration()[0] - last_acc) <= .010001, "jerk bound");
      last_acc = motion.acceleration()[0];
    }
    motion.stop();
    bool finished = false;
    for (int i = 0; i < 2000; ++i) {
      finished = motion.update();
      require(std::abs(motion.acceleration()[0] - last_acc) <= .010001, "stop jerk discontinuity");
      last_acc = motion.acceleration()[0];
      if (finished) break;
    }
    require(finished && std::abs(motion.velocity()[0]) < 1e-8, "stop did not brake to zero");
    motion.target(Joints{1,1,1,1,1,1,1});
    require(motion.update(), "hard stop was re-armed by target");
    Motion paused(cfg, Joints{});
    paused.target(Joints{1,1,1,1,1,1,1});
    for (int i=0; i<400; ++i) paused.update();
    last_acc = paused.acceleration()[0];
    paused.hold();
    bool early_resume_rejected = false;
    try { paused.target(Joints{}); } catch (...) { early_resume_rejected = true; }
    require(early_resume_rejected, "resumed before braking finished");
    for (int i=0; i<2000 && !paused.hold_ready(); ++i) {
      paused.hold();  // repeated heartbeat must not reset braking
      require(!paused.update(), "hold shut down the controller");
      require(std::abs(paused.acceleration()[0]-last_acc) <= .010001, "hold jerk discontinuity");
      last_acc = paused.acceleration()[0];
    }
    require(paused.hold_ready() && std::abs(paused.velocity()[0]) < 1e-8, "hold did not settle");
    const double held_position = paused.position()[0];
    for (int i=0; i<100; ++i) paused.update();
    require(std::abs(paused.position()[0]-held_position) < 1e-10, "hold position drifted");
    paused.target(Joints{1,1,1,1,1,1,1}, true);
    last_acc = 0.;
    for (int i=0; i<3000; ++i) {
      paused.update();
      require(std::abs(paused.velocity()[0]) <= .060001, "home velocity bound");
      require(std::abs(paused.acceleration()[0]) <= .150001, "home acceleration bound");
      require(std::abs(paused.acceleration()[0]-last_acc) <= .001001, "home jerk bound");
      last_acc = paused.acceleration()[0];
    }
    require(paused.position()[0] > held_position, "held motion did not resume");
    bool rejected = false;
    try { Motion invalid(cfg, Joints{3,0,0,0,0,0,0}); } catch (...) { rejected = true; }
    require(rejected, "initial joint limit not enforced");
    SpscQueue<int, 32> queue;
    std::thread producer([&] { for (int i = 0; i < 100000; ++i) while (!queue.push(i)) std::this_thread::yield(); });
    for (int i = 0; i < 100000; ++i) { int value; while (!queue.pop(value)) std::this_thread::yield(); require(value == i, "mailbox ordering"); }
    producer.join();
    std::cout << "PASS: Desk tool guard, trajectory bounds, continuous stop/hold/resume, home limits, 100000 ordered concurrent messages\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
