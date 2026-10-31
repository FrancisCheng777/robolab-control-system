// Line-oriented diagnostic endpoint using the same driver as the Panda bridge.
#include "robotiq_gripper.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

using json = nlohmann::json;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt(int) { interrupted = 1; }
static double now() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
static json state(const robotiq::GripperStatus& s) {
  return {{"activated", s.activated}, {"goto_requested", s.goto_requested},
          {"status", s.status}, {"object_detection", s.object_detection},
          {"fault", s.fault}, {"requested_position", s.requested_position},
          {"position", s.position}, {"current_10mA", s.current}};
}

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--device") {
    std::cerr << "Usage: pi05_gripper_tool --device /dev/serial/by-id/...\n";
    return 2;
  }
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  std::signal(SIGHUP, interrupt);
  std::signal(SIGPIPE, SIG_IGN);
  robotiq::Robotiq2FGripper gripper(argv[2]);
  bool moving = false;
  int result = 0;
  try {
    if (!gripper.connect()) throw std::runtime_error("gripper connect failed");
    robotiq::GripperStatus previous;
    double feedback_time = 0;
    std::string buffer;
    double last_request = now();
    while (!interrupted) {
      pollfd fd{STDIN_FILENO, POLLIN, 0};
      const int ready = poll(&fd, 1, 25);
      if (ready < 0) {
        if (interrupted) break;
        throw std::runtime_error("stdin poll failed");
      }
      if (moving && now() - last_request > 0.25)
        throw std::runtime_error("250 ms command watchdog expired");
      if (!ready) continue;
      char bytes[1024];
      const auto count = read(STDIN_FILENO, bytes, sizeof(bytes));
      if (count <= 0) break;
      buffer.append(bytes, static_cast<size_t>(count));
      if (buffer.size() > 4096) throw std::runtime_error("request too large");
      size_t end;
      while ((end = buffer.find('\n')) != std::string::npos) {
        const auto request = json::parse(buffer.substr(0, end));
        buffer.erase(0, end + 1);
        const std::string op = request.at("op");
        const double received = now();
        json response;
        if (op == "move") {
          if (received - feedback_time > 0.2 || !previous.activated ||
              previous.status != 3 || previous.fault != 0)
            throw std::runtime_error("fresh ready feedback required before move");
          const int position = request.at("position");
          const int speed = request.at("speed");
          const int force = request.at("force");
          if (position < 0 || position > 255 || speed < 1 || speed > 255 || force < 1 || force > 255)
            throw std::runtime_error("register value outside allowed range");
          // Mark ownership before writing: if acknowledgement is lost, still try stop.
          moving = true;
          response["command_send_s"] = now();
          if (!gripper.move(position, speed, force)) throw std::runtime_error("move write failed");
          response["command_ack_s"] = now();
        } else if (op == "stop") {
          if (!gripper.stop()) throw std::runtime_error("stop write failed");
          moving = false;
        } else if (op != "status") {
          throw std::runtime_error("unknown operation (activation is deliberately manual)");
        }
        previous = gripper.get_status();
        feedback_time = now();
        if (moving && (!previous.activated || previous.status != 3 || previous.fault != 0))
          throw std::runtime_error("gripper not ready or fault during motion");
        response.update(state(previous));
        response["request_receive_s"] = received;
        response["feedback_receive_s"] = feedback_time;
        std::cout << response.dump() << std::endl;
        if (!std::cout) throw std::runtime_error("controller disconnected");
        last_request = received;
      }
    }
  } catch (const std::exception& e) {
    std::cout << json({{"error", e.what()}}).dump() << std::endl;
    std::cerr << e.what() << std::endl;
    result = 1;
  }
  if (moving && !gripper.stop()) {
    std::cerr << "STOP NOT ACKNOWLEDGED: check the physical gripper\n";
    result = 1;
  }
  return result;
}
