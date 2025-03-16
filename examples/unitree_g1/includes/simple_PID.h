#pragma once
#include <chrono>
#include <Eigen/Dense>  // Include Eigen for matrix operations

class SIMPLE_PID {
 private:
  double Kp_, Ki_, Kd_;
  double dt_;
  double integrator_min_, integrator_max_;
  Eigen::VectorXd integral_, prev_error_;
  std::chrono::steady_clock::time_point last_time_;

 public:
  SIMPLE_PID(double Kp, double Ki, double Kd, double dt = 0.0,
             double min_limit = -1e6, double max_limit = 1e6, size_t vector_size = 1);
  
  Eigen::VectorXd compute(const Eigen::VectorXd& setpoint, const Eigen::VectorXd& measured_value);
  void reset();
};
