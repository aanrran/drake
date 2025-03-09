#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include <Eigen/Dense>
#include <vector>


namespace drake {
namespace examples {
namespace unitree_g1 {
namespace walking_pattern {

using Eigen::VectorXd;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::Diagram;
using drake::systems::LeafSystem;

/** StandingFSM: A simple finite state machine for standing in double support */
class StandingFSM : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StandingFSM);
  StandingFSM();

  const drake::systems::OutputPort<double>& get_output_port_com_position() const {
    return this->get_output_port(output_port_com_position_);
  }

  const drake::systems::OutputPort<double>& get_output_port_right_foot_position() const {
    return this->get_output_port(output_port_right_foot_position_);
  }

  const drake::systems::OutputPort<double>& get_output_port_left_foot_position() const {
    return this->get_output_port(output_port_left_foot_position_);
  }

 private:
  void CalcCoMPosition(const drake::systems::Context<double>&, drake::systems::BasicVector<double>*) const;
  void CalcRightFootPosition(const drake::systems::Context<double>&, drake::systems::BasicVector<double>*) const;
  void CalcLeftFootPosition(const drake::systems::Context<double>&, drake::systems::BasicVector<double>*) const;

  drake::systems::OutputPortIndex output_port_com_position_;
  drake::systems::OutputPortIndex output_port_right_foot_position_;
  drake::systems::OutputPortIndex output_port_left_foot_position_;
};

/** WalkingFSM: A finite state machine for generating a simple walking pattern */
class WalkingFSM : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WalkingFSM);
  WalkingFSM(int n_steps, double step_length, double step_height, double step_time);

    // ✅ Getter for right foot placements
    const std::vector<Eigen::VectorXd>& get_right_foot_placements() const {
      return right_foot_placements_;
  }

  // ✅ Getter for left foot placements
  const std::vector<Eigen::VectorXd>& get_left_foot_placements() const {
      return left_foot_placements_;
  }

  double get_total_time() const { return total_time_; }
  
  const drake::trajectories::PiecewisePolynomial<double>& get_zmp_trajectory() const { 
      return zmp_trajectory_; 
  }

  const drake::trajectories::PiecewisePolynomial<double>& get_com_trajectory() const { 
    return com_trajectory_; 
  }

  const drake::trajectories::PiecewisePolynomial<double>& get_right_foot_trajectory() const { 
    return right_foot_trajectory_; 
  }

  const drake::trajectories::PiecewisePolynomial<double>& get_left_foot_trajectory() const { 
    return left_foot_trajectory_; 
  }

  std::string SupportPhase(double time) const;

 private:
  void GenerateFootPlacements();
  void GenerateZMPTrajectory();
  void GenerateCoMTrajectory();
  void GenerateFootTrajectories();

  int n_steps_;
  double step_length_, step_height_, step_time_;
  int n_phases_;
  double total_time_;
  double fc_offset_;
  double foot_w1_, foot_w2_, foot_l1_, foot_l2_;
  double h_, g_;
  VectorXd x_com_init_, xd_com_init_;
  VectorXd x_right_init_, x_left_init_;
  std::vector<VectorXd> right_foot_placements_, left_foot_placements_;
  PiecewisePolynomial<double> zmp_trajectory_, com_trajectory_, right_foot_trajectory_, left_foot_trajectory_;
};

}  // namespace walking_pattern
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake