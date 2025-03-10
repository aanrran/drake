#include "examples/unitree_g1/includes/timing_logger.h"

#include <iostream>
namespace drake {
namespace examples {
namespace unitree_g1 {


TimingLogger& TimingLogger::GetInstance() {
  static TimingLogger instance;
  return instance;
}

void TimingLogger::StartTimer(const std::string& timer_name) {
  start_times_[timer_name] = std::chrono::high_resolution_clock::now();
}

void TimingLogger::StopTimer(const std::string& timer_name) {
  auto end_time = std::chrono::high_resolution_clock::now();
  auto start_time = start_times_[timer_name];
  auto duration =
      std::chrono::duration<double, std::micro>(end_time - start_time).count();
  std::cout << "[Timing] " << timer_name << ": " << duration << " us"
            << std::endl;
}


}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake