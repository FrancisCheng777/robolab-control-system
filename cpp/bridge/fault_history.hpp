#pragma once
#include <franka/log.h>
#include <nlohmann/json.hpp>
namespace pi05 {
// Use the existing libfranka exception ring; serialize only after control stops.
inline nlohmann::json fault_wrench_history(const std::vector<franka::Record>& log) {
  auto rows = nlohmann::json::array();
  for (const auto& entry : log) {
    const auto& s = entry.state;
    rows.push_back({{"robot_time_ms", s.time.toMSec()}, {"q", s.q}, {"dq", s.dq},
      {"O_F_ext_hat_K", s.O_F_ext_hat_K}, {"K_F_ext_hat_K", s.K_F_ext_hat_K},
      {"tau_ext_hat_filtered", s.tau_ext_hat_filtered},
      {"cartesian_collision", s.cartesian_collision},
      {"joint_collision", s.joint_collision}, {"joint_contact", s.joint_contact}});
  }
  return {{"source", "libfranka ControlException log; estimated external wrench"},
    {"wrench_order", {"Fx", "Fy", "Fz", "Mx", "My", "Mz"}},
    {"wrench_units", {"N", "N", "N", "Nm", "Nm", "Nm"}},
    {"O_frame", "robot base"}, {"K_frame", "stiffness frame"},
    {"samples", rows}};
}
}
