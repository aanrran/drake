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
  /** @brief Starts the timer for a given key. */
  void StartTimer(const std::string& key);

  /** 
   * @brief Stops the timer and updates the longest recorded time.
   * If the new time is longer than the previous, it updates and prints.
   */
  void StopTimer(const std::string& key);

  /** @brief Singleton instance access */
  static TimingLogger& GetInstance();

 private:
  /** Stores the start time of each timer. */
  std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> start_times_;

  /** Stores the longest recorded duration for each timer. */
  std::unordered_map<std::string, double> longest_durations_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
