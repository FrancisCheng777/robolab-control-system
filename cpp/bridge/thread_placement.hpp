#pragma once
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>

// Call only after creating non-RT workers: they must not inherit the RT CPU.
// Unset leaves standalone/mock users' existing affinity unchanged.
inline int pin_control_thread_from_env() {
  const char* value = std::getenv("PI05_CONTROL_CPU");
  if (!value) return -1;
  char* end = nullptr;
  errno = 0;
  const long cpu = std::strtol(value, &end, 10);
  if (errno || end == value || *end || cpu < 0 || cpu >= CPU_SETSIZE)
    throw std::runtime_error("PI05_CONTROL_CPU must be a valid CPU index");
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(cpu, &mask);
  const int result = pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
  if (result) throw std::runtime_error(std::string("control CPU affinity: ") + std::strerror(result));
  return static_cast<int>(cpu);
}
