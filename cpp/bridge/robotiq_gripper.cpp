// Robotiq 2F adapter over the official SDK (pinned in vendor/robotiq_grippers).
#include "robotiq_gripper.h"
#include <Robotiq/gripper/modbus_client.hpp>
#include <Robotiq/gripper/connection_config.hpp>
#include <Robotiq/gripper/command.hpp>
#include <Robotiq/gripper/status.hpp>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>

namespace robotiq {

struct Robotiq2FGripper::Impl {
  // Transactions run only on the bridge's dedicated worker (or diagnostic
  // process), never on Franka's real-time callback. The low-level SDK API lets
  // status-only diagnostics remain strictly read-only; no second exchange loop.
  std::unique_ptr<Robotiq::GripperModbusClient> client;
  int lock_fd = -1;
};

Robotiq2FGripper::Robotiq2FGripper(const std::string& device)
    : impl_(new Impl), device_(device) {}

Robotiq2FGripper::~Robotiq2FGripper() {
  close();
  delete impl_;
}

bool Robotiq2FGripper::connect() {
  if (connected_) return true;
  // Lock before constructing the SDK. TIOCEXCL must follow the SDK's own open;
  // setting it on this guard fd first would block the SDK's second open.
  impl_->lock_fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (impl_->lock_fd < 0) return false;
  if (flock(impl_->lock_fd, LOCK_EX | LOCK_NB) < 0) {
    close();
    return false;
  }
  try {
    Robotiq::ConnectionConfig config;
    config.serial.port = device_;
    config.serial.baudrate = 115200;
    config.serial.timeout = std::chrono::milliseconds(200);
    config.modbusSlaveAddress = 9;
    impl_->client = std::make_unique<Robotiq::GripperModbusClient>(config);
    // The SDK requests exclusivity; enforce success as well.
    if (ioctl(impl_->lock_fd, TIOCEXCL) < 0)
      throw std::runtime_error("cannot obtain Robotiq serial exclusivity");
    connected_ = true;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Robotiq SDK connect failed: " << e.what() << std::endl;
    close();
    return false;
  }
}

void Robotiq2FGripper::close() {
  impl_->client.reset();
  if (impl_->lock_fd >= 0) {
    ::close(impl_->lock_fd);
    impl_->lock_fd = -1;
  }
  connected_ = false;
}

bool Robotiq2FGripper::send_command(const uint8_t data[6]) {
  if (!connected_) return false;
  try {
    Robotiq::GripperCommand command{};
    std::copy_n(data, 6, command.data()); // remaining reserved bytes stay zero
    impl_->client->writeCommand(command);
    return true; // Modbus write acknowledged, not physical motion complete
  } catch (const std::exception& e) {
    std::cerr << "Robotiq SDK write failed: " << e.what() << std::endl;
    return false;
  }
}

bool Robotiq2FGripper::read_status(uint8_t data[6]) {
  if (!connected_) return false;
  try {
    const auto status = impl_->client->readStatus();
    std::copy_n(status.data(), 6, data);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Robotiq SDK read failed: " << e.what() << std::endl;
    return false;
  }
}

bool Robotiq2FGripper::activate() {
  if (!connected_) return false;

  // Preserve an already activated gripper's position. Never reset a fault
  // automatically; an operator must resolve it before starting this bridge.
  auto initial = get_status();
  // Idle devices report 0x09 until polling resumes. Reading again clears this
  // communication flag without activation/reset or a motor command.
  for (int i = 0; i < 10 && initial.fault == 0x09; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    initial = get_status();
  }
  if (initial.fault != 0) return false;
  if (initial.activated && initial.status == 3) return true;

  // 1. 发送 reset (rACT=0)
  uint8_t cmd[6] = {0, 0, 0, 0, 0, 0};
  if (!send_command(cmd)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 2. 发送激活 (rACT=1, rGTO=1)
  cmd[0] = 0x01 | (0x01 << 3);  // rACT=1, rGTO=1
  cmd[3] = 0;    // position=0 (全开)
  cmd[4] = 255;  // speed=max
  cmd[5] = 128;  // force=default
  if (!send_command(cmd)) return false;

  // 3. 等待 gSTA=3 (ready)
  for (int i = 0; i < 50; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto st = get_status();
    if (st.activated && st.status == 3) {
      std::cout << "Robotiq 夹爪已激活" << std::endl;
      return true;
    }
  }
  std::cerr << "激活超时" << std::endl;
  return false;
}

bool Robotiq2FGripper::move(uint8_t position, uint8_t speed, uint8_t force) {
  if (!connected_) return false;

  uint8_t cmd[6];
  cmd[0] = 0x01 | (0x01 << 3);  // rACT=1, rGTO=1
  cmd[1] = 0;
  cmd[2] = 0;
  cmd[3] = position;
  cmd[4] = speed;
  cmd[5] = force;
  return send_command(cmd);
}

bool Robotiq2FGripper::move_width(double width_m, uint8_t speed, uint8_t force) {
  // 2F-85 名义开度: 0 m = position 255 (全闭), 0.085 m = position 0 (全开)
  // width 越大 => position 越小
  width_m = std::max(0.0, std::min(max_width_, width_m));
  uint8_t pos = static_cast<uint8_t>((1.0 - width_m / max_width_) * 255);
  return move(pos, speed, force);
}

GripperStatus Robotiq2FGripper::get_status() {
  GripperStatus st;
  uint8_t data[6] = {0};
  if (!read_status(data)) throw std::runtime_error("Robotiq feedback read failed");

  st.activated = (data[0] & 0x01);
  st.goto_requested = (data[0] >> 3) & 0x01;
  st.status = (data[0] >> 4) & 0x03;
  st.object_detection = (data[0] >> 6) & 0x03;
  st.fault = data[2];
  st.requested_position = data[3];  // gPR (请求位置)
  st.position = data[4];  // gPO (实际位置)
  st.current = data[5];
  return st;
}

double Robotiq2FGripper::get_width() const {
  // 当前无法直接拿到 gPO，需在 get_status() 里缓存
  // 这里暂时返回 0（TODO: 需要在类里加个 last_status_ 缓存）
  return 0.0;
}

bool Robotiq2FGripper::wait_for_move(double timeout_s) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto st = get_status();
    // gOBJ != 0: 1=打开接触 2=闭合接触 3=到位无物体；仅 gGTO=1 有效
    if (st.object_detection != 0) return true;

    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (elapsed > timeout_s) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool Robotiq2FGripper::stop() {
  // rGTO=0 停止运动
  uint8_t cmd[6] = {0x01, 0, 0, 0, 0, 0};  // rACT=1, rGTO=0
  return send_command(cmd);
}

}  // namespace robotiq
