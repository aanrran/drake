#include "examples/unitree_g1/includes/simple_PID.h"

#include <algorithm>

SIMPLE_PID::SIMPLE_PID(double Kp, double Ki, double Kd, double dt,
                       double min_limit, double max_limit, size_t vector_size)
    : Kp_(Kp),
      Ki_(Ki),
      Kd_(Kd),
      dt_(dt),
      integrator_min_(min_limit),
      integrator_max_(max_limit),
      integral_(Eigen::VectorXd::Zero(vector_size)),
      prev_error_(Eigen::VectorXd::Zero(vector_size)) {
  last_time_ = std::chrono::steady_clock::now();
}

Eigen::VectorXd SIMPLE_PID::compute(const Eigen::VectorXd& setpoint,
                                    const Eigen::VectorXd& measured_value) {
  // Compute actual dt
  double actual_dt = dt_;
  if (!(dt_ > 0.0)) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_time_;
    last_time_ = now;
    actual_dt = elapsed.count();
  }

  Eigen::VectorXd error = setpoint - measured_value;

  // Integral term with anti-windup
  if (Ki_ > 0) {
    integral_ += error * actual_dt;
    integral_ = integral_.unaryExpr([this](double val) {
      return std::clamp(val, integrator_min_, integrator_max_);
    });
  }

  // Derivative term
  Eigen::VectorXd derivative = (error - prev_error_) / actual_dt;
  prev_error_ = error;

  return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
}

void SIMPLE_PID::reset() {
  integral_.setZero();
  prev_error_.setZero();
  last_time_ = std::chrono::steady_clock::now();
}
