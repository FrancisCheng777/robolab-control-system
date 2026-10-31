#include "fault_history.hpp"
#include <cassert>
int main() {
  assert(pi05::fault_wrench_history({})["samples"].empty());
  std::vector<franka::Record> log(2);
  log[0].state.time = franka::Duration(123);
  log[0].state.O_F_ext_hat_K = {1,2,-23,4,5,6};
  log[0].state.K_F_ext_hat_K = {-7,8,9,-10,11,12};
  log[1].state.time = franka::Duration(124);
  log[1].state.cartesian_collision[2] = 1;
  auto report = pi05::fault_wrench_history(log);
  if(report["samples"].size()!=2 || report["samples"][0]["robot_time_ms"]!=123 ||
     report["samples"][0]["O_F_ext_hat_K"][2]!=-23 ||
     report["samples"][0]["K_F_ext_hat_K"][0]!=-7 ||
     report["samples"][1]["cartesian_collision"][2]!=1 ||
     report["wrench_units"][3]!="Nm") return 1;
  return 0;
}
