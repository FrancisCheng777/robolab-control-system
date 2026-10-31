#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pi05 {
// Compare configured Desk tool to FCI readback before any control/activation.
// setLoad is a separate payload and cannot fix a mismatched Desk end effector.
inline void require_desk_tool(const nlohmann::json& expected, const nlohmann::json& actual) {
  const double wanted_mass = expected.at("mass").get<double>();
  const double mass = actual.at("mass").get<double>();
  if (!std::isfinite(mass) || !std::isfinite(wanted_mass) ||
      wanted_mass <= 0 || std::abs(mass - wanted_mass) > 1e-4)
    throw std::runtime_error("Desk tool mass mismatch; select the installed Robotiq profile");
  const auto compare = [&](const char* key, const char* read_key, size_t count, double tolerance) {
    const auto& want = expected.at(key);
    const auto& got = actual.at(read_key);
    if (!want.is_array() || !got.is_array() || want.size() != count || got.size() != count)
      throw std::runtime_error(std::string("Desk tool field size mismatch: ") + key);
    for (size_t i = 0; i < count; ++i) {
      const double a = want.at(i).get<double>(), b = got.at(i).get<double>();
      if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a - b) > tolerance)
        throw std::runtime_error(std::string("Desk tool readback mismatch: ") + read_key);
    }
  };
  compare("centerOfMass", "centerOfMass", 3, 1e-5);
  compare("inertia", "inertia", 9, 1e-6);
  compare("transformation", "transformation", 16, 1e-5);
  compare("transformation", "activeTransformation", 16, 1e-5);
}
} // namespace pi05
