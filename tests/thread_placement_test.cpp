#include "thread_placement.hpp"
#include <future>
#include <iostream>
#include <thread>

cpu_set_t affinity() {
  cpu_set_t value;
  if (pthread_getaffinity_np(pthread_self(), sizeof(value), &value)) throw std::runtime_error("get affinity failed");
  return value;
}
int main() {
  unsetenv("PI05_CONTROL_CPU");
  const auto before = affinity();
  if (pin_control_thread_from_env() != -1) return 1;
  const auto unchanged = affinity();
  if (!CPU_EQUAL(&before, &unchanged)) return 2;
  int selected = -1;
  for (int i=0; i<CPU_SETSIZE; ++i) if (CPU_ISSET(i, &before)) {selected=i; break;}
  std::promise<void> ready, done;
  auto ready_future=ready.get_future(), done_future=done.get_future();
  bool worker_unchanged=false;
  std::thread worker([&] {ready.set_value(); done_future.wait(); const auto m=affinity();worker_unchanged=CPU_EQUAL(&before,&m);});
  ready_future.wait();
  setenv("PI05_CONTROL_CPU", std::to_string(selected).c_str(), 1);
  const int result=pin_control_thread_from_env();
  const auto pinned=affinity();
  done.set_value();worker.join();
  if(result!=selected || CPU_COUNT(&pinned)!=1 || !CPU_ISSET(selected,&pinned) || !worker_unchanged) return 3;
  for(const char* invalid : {"", "abc", "-1", "999999999999999999999", "1x", "1048576"}) {
    setenv("PI05_CONTROL_CPU",invalid,1);
    bool rejected=false;
    try {pin_control_thread_from_env();} catch(const std::runtime_error&) {rejected=true;}
    if(!rejected) return 4;
  }
  std::cout << "thread placement: unchanged default, independent worker, valid pin and invalid input checks passed\n";
}
