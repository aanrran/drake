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
  auto duration = std::chrono::duration<double, std::micro>(end_time - start_time).count();

  // Check if it's the longest duration recorded
  if (duration > longest_durations_[timer_name]) {
    longest_durations_[timer_name] = duration;

    // Clear the previous line and overwrite with new data
    std::cout << "\r[Timing] " << timer_name << "'s longest run-time: " << duration << " us    " << std::endl;
  }
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
