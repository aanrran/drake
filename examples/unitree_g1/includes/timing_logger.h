// timing_logger.h
#pragma once
#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
namespace drake {
namespace examples {
namespace unitree_g1 {


class TimingLogger {
 public:
  void StartTimer(const std::string& key);
  void StopTimer(const std::string& key);
  static TimingLogger& GetInstance();

 private:
  std::unordered_map<std::string,
                     std::chrono::high_resolution_clock::time_point>
      start_times_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
