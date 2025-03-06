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
    output_port_com_position_ = this->DeclareVectorOutputPort(
        "com_position", BasicVector<double>(3), &StandingFSM::CalcCoMPosition).get_index();
    output_port_right_foot_position_ = this->DeclareVectorOutputPort(
        "right_foot_position", BasicVector<double>(3), &StandingFSM::CalcRightFootPosition).get_index();
    output_port_left_foot_position_ = this->DeclareVectorOutputPort(
        "left_foot_position", BasicVector<double>(3), &StandingFSM::CalcLeftFootPosition).get_index();
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

    this->n_steps_ = n_steps;
    this->step_length_ = step_length;
    this->step_height_ = step_height;
    this->step_time_ = step_time;
    this->n_phases_ = 2 * n_steps + 1;
    this->total_time_ = step_time * n_phases_;

    // Create subcomponents and add to diagram
    // auto standing_fsm = builder.AddSystem<StandingFSM>();

    // Additional walking subcomponents (to be added in future steps)
    GenerateFootPlacements();
    GenerateZMPTrajectory();
    GenerateCoMTrajectory();
    GenerateFootTrajectories();

    // Build the Diagram
    builder.BuildInto(this);
}

void WalkingFSM::GenerateFootPlacements() {
    std::cout << "GenerateFootPlacements() called." << std::endl;

    // Initialize right and left foot positions
    x_right_init_ = Eigen::Vector3d(fc_offset_, -0.138, 0.1);
    x_left_init_ = Eigen::Vector3d(fc_offset_, 0.138, 0.1);

    right_foot_placements_.push_back(x_right_init_);
    left_foot_placements_.push_back(x_left_init_);

    // Generate footstep placements
    for (int step = 0; step < n_steps_; step++) {
        double l = (step == 0 || step == n_steps_ - 1) ? step_length_ / 2.0 : step_length_;

        if (step % 2 == 0) {
            // Move right foot forward
            Eigen::Vector3d x_right = right_foot_placements_.back();
            x_right(0) += l;
            right_foot_placements_.push_back(x_right);
        } else {
            // Move left foot forward
            Eigen::Vector3d x_left = left_foot_placements_.back();
            x_left(0) += l;
            left_foot_placements_.push_back(x_left);
        }
    }
}

void WalkingFSM::GenerateZMPTrajectory() {
    std::cout << "GenerateZMPTrajectory() called." << std::endl;

    // Define timestamps (breakpoints) for interpolation
    Eigen::VectorXd break_times = Eigen::VectorXd::LinSpaced(n_phases_ + 1, 0, total_time_);
    Eigen::MatrixXd zmp_knots(2, n_phases_ + 1);

    // Compute initial ZMP position as the midpoint between both feet
    double initial_x = (right_foot_placements_.front()(0) + left_foot_placements_.front()(0)) / 2.0 - fc_offset_;
    double initial_y = (right_foot_placements_.front()(1) + left_foot_placements_.front()(1)) / 2.0;

    zmp_knots.col(0) << initial_x, initial_y;

    // Generate ZMP reference trajectory
    int rf_idx = 1, lf_idx = 1;
    for (int i = 0; i < n_steps_ - 1; i++) {
        Eigen::Vector3d foot_center;
        if (i % 2 == 0) {
            foot_center = right_foot_placements_[rf_idx++];
        } else {
            foot_center = left_foot_placements_[lf_idx++];
        }

        foot_center(0) -= fc_offset_;  // Adjust for foot offset

        zmp_knots.col(2 * i + 1) = foot_center.head(2);
        zmp_knots.col(2 * i + 2) = foot_center.head(2);
    }

    // Final shift of ZMP to center between feet
    double last_x = zmp_knots(0, zmp_knots.cols() - 1);
    zmp_knots.col(zmp_knots.cols() - 1) << last_x, 0.0;

    // Store ZMP trajectory as a piecewise polynomial
    zmp_trajectory_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(break_times, zmp_knots);
}



void WalkingFSM::GenerateCoMTrajectory() {
    // TODO: Implement CoM trajectory generation (using MPC)
    std::cout << "GenerateCoMTrajectory() called." << std::endl;
}

void WalkingFSM::GenerateFootTrajectories() {
    // TODO: Implement foot trajectory generation
    std::cout << "GenerateFootTrajectories() called." << std::endl;
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
