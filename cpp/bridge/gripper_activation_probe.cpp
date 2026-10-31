// Test fixture: only invoked by the PTY tests, never a hardware launch script.
#include "robotiq_gripper.h"
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  try {
    robotiq::Robotiq2FGripper gripper(argv[1]);
    return gripper.connect() && gripper.activate() ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
