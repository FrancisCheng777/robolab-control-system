#include "thread_placement.hpp"
#include <iostream>

int main() {
  unsetenv("PI05_CONTROL_CPU");
  if (pin_control_thread_from_env() != -1) return 1;
  setenv("PI05_CONTROL_CPU", "invalid", 1);
  try {
    pin_control_thread_from_env();
    return 2;
  } catch (const std::runtime_error&) {}
#ifndef __linux__
  setenv("PI05_CONTROL_CPU", "0", 1);
  try {
    pin_control_thread_from_env();
    return 3;
  } catch (const std::runtime_error& error) {
    if (std::string(error.what()).find("requires Linux") == std::string::npos) return 4;
  }
#endif
  unsetenv("PI05_CONTROL_CPU");
  std::cout << "thread placement checks passed\n";
}
