#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "examples/unitree_g1/includes/walking_pattern_generator.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace walking_pattern {

using namespace drake::trajectories;
using namespace drake::systems;
using namespace drake::solvers;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// StandingFSM Constructor
StandingFSM::StandingFSM() {
    this->DeclareVectorOutputPort("com_position", BasicVector<double>(3),
                                  &StandingFSM::CalcCoMPosition);
    this->DeclareVectorOutputPort("right_foot_position", BasicVector<double>(3),
                                  &StandingFSM::CalcRightFootPosition);
    this->DeclareVectorOutputPort("left_foot_position", BasicVector<double>(3),
                                  &StandingFSM::CalcLeftFootPosition);
}

void StandingFSM::CalcCoMPosition(const Context<double>&, BasicVector<double>* output) const {
    VectorXd x_com_init(3);
    x_com_init << 0.0, 0.0, 0.90;
    output->set_value(x_com_init);
}

void StandingFSM::CalcRightFootPosition(const Context<double>&, BasicVector<double>* output) const {
    VectorXd x_right_init(3);
    x_right_init << -0.065, -0.138, 0.1;
    output->set_value(x_right_init);
}

void StandingFSM::CalcLeftFootPosition(const Context<double>&, BasicVector<double>* output) const {
    VectorXd x_left_init(3);
    x_left_init << -0.065, 0.138, 0.1;
    output->set_value(x_left_init);
}

// WalkingFSM Constructor
WalkingFSM::WalkingFSM(int n_steps, double step_length, double step_height, double step_time) {
    DiagramBuilder<double> builder;
    
    assert(n_steps >= 2 && "Must specify at least two steps");
    
    n_steps_ = n_steps;
    step_length_ = step_length;
    step_height_ = step_height;
    step_time_ = step_time;
    n_phases_ = 2 * n_steps + 1;
    total_time_ = step_time * n_phases_;
    
    GenerateFootPlacements();
    GenerateZMPTrajectory();
    GenerateCoMTrajectory();
    GenerateFootTrajectories();
    
    builder.BuildInto(this);
}

// int main() {
//   WalkingFSM walking_fsm(4, 0.5, 0.1, 0.5);
//   StandingFSM standing_fsm;
  
//   std::cout << "Walking and Standing FSMs initialized successfully!" << std::endl;
//   return 0;
// }

}  // namespace walking_pattern
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
