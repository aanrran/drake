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

    this->n_steps_ = n_steps;                    // number of steps to take
    this->step_length_ = step_length;            // how far forward each step goes
    this->step_height_ = step_height;            // maximum height of the swing foot
    this->step_time_ = step_time;                // how long each swing phase lasts
    this->n_phases_ = 2 * n_steps;           // how many different phases (double support, left support
                                                 // right support, etc. there are throughout the whole motion.
    this->total_time_ = step_time * n_phases_; 

    // Initial CoM and foot positions
    this->x_com_init_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    this->xd_com_init_ = Eigen::Vector3d(0.0, 0.0, 0.0);    

    this->fc_offset_ = -0.065;   // The foot frame is this far from the foot's center

    // Initialize right and left foot positions
    this->x_right_init_ = Eigen::Vector3d(this->fc_offset_, -0.138, 0.1);
    this->x_left_init_ = Eigen::Vector3d(this->fc_offset_, 0.138, 0.1);

    this->foot_w1_ = 0.08;  // width left of foot frame
    this->foot_w2_ = 0.08;  // width right of foot frame
    this->foot_l1_ = 0.2;   // length in front of foot
    this->foot_l2_ = 0.07;  // length behind foot

    // LIP parameters
    this->h_ = this->x_com_init_(2);
    this->g_ = 9.81;

    // Create subcomponents and add to diagram
    auto standing_fsm = builder.AddSystem<StandingFSM>();

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
    // Generate foot ground contact placements for both feet
    this->right_foot_placements_.push_back(this->x_right_init_);
    this->left_foot_placements_.push_back(this->x_left_init_);

    // Generate footstep placements
    for (int step = 0; step < n_steps_; step++) {
        double l = (step == 0 || step == n_steps_ - 1) ? step_length_ / 2.0 : step_length_;

        if (step % 2 == 0) {
            // Move right foot forward
            Eigen::Vector3d x_right = this->right_foot_placements_.back();
            x_right(0) += l;
            this->right_foot_placements_.push_back(x_right);
        } else {
            // Move left foot forward
            Eigen::Vector3d x_left = this->left_foot_placements_.back();
            x_left(0) += l;
            this->left_foot_placements_.push_back(x_left);
        }
    }
}

void WalkingFSM::GenerateZMPTrajectory() {
    std::cout << "GenerateZMPTrajectory() called." << std::endl;

    // Define timestamps (breakpoints) for interpolation
    Eigen::VectorXd break_times = Eigen::VectorXd::LinSpaced(n_phases_, 0, total_time_);
    Eigen::MatrixXd zmp_knots(2, n_phases_);
    std::cout << "total time: " << total_time_<< std::endl;
    std::cout << "break_times len: " << break_times.size() << std::endl;
    std::cout << "zmp_knots size: " << zmp_knots.size() << std::endl;

    // Compute initial ZMP position as the midpoint between both feet
    double initial_x = (right_foot_placements_.front()(0) + left_foot_placements_.front()(0)) / 2.0 - fc_offset_;
    double initial_y = (right_foot_placements_.front()(1) + left_foot_placements_.front()(1)) / 2.0;

    // specify the desired ZMP at the break times
    zmp_knots.col(0) << initial_x, initial_y;

    // Generate ZMP reference trajectory
    Eigen::Vector3d foot_center;
    int rf_idx = 1, lf_idx = 1;
    for (int i = 0; i < n_steps_ - 1; i++) {
        
        if (i % 2 == 0) {
            foot_center = right_foot_placements_[rf_idx++]; // Right foot moved: shift ZMP under the right foot now
        } else {
            foot_center = left_foot_placements_[lf_idx++];  // Left foot moved: shift ZMP under the left foot now
        }

        foot_center(0) -= fc_offset_;  // Adjust for foot offset
        std::cout << i+1 <<"th foot placement: x->" << foot_center(0) << " y->" << foot_center(1) << std::endl;

        zmp_knots.col(2 * i + 1) = foot_center.head(2);
        zmp_knots.col(2 * i + 2) = foot_center.head(2);
    }

    // Final shift of ZMP to center between feet in double support
    zmp_knots.col(zmp_knots.cols() - 1) << foot_center.head(1), 0.0;

    // Store ZMP trajectory as a piecewise polynomial
    zmp_trajectory_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(break_times, zmp_knots);


}



void WalkingFSM::GenerateCoMTrajectory() {
    std::cout << "GenerateCoMTrajectory() called." << std::endl;

    // Define discrete time step for MPC
    double dt = step_time_ / 2.0;  // Discretization step (subdividing the step time)
    int n_steps_mpc = total_time_ / dt;

    // Create an optimization problem
    drake::solvers::MathematicalProgram prog;

    // Decision variables: CoM state (x, y, dx, dy) and control input (ux, uy)
    auto X = prog.NewContinuousVariables(4, n_steps_mpc, "X");
    auto U = prog.NewContinuousVariables(2, n_steps_mpc, "U");

    // Define LIP discrete-time dynamics matrices
    Eigen::Matrix4d A;
    A.setIdentity();
    A(0, 2) = dt;
    A(1, 3) = dt;

    Eigen::Matrix<double, 4, 2> B;
    B.setZero();
    B(0, 0) = 0.5 * dt * dt;  // ✅ Add quadratic term for position update
    B(1, 1) = 0.5 * dt * dt;
    B(2, 0) = dt;
    B(3, 1) = dt;
    
    Eigen::Matrix<double, 2, 4> C;
    C.setZero();
    C(0, 0) = 1.0;
    C(1, 1) = 1.0;

    Eigen::Matrix2d D;
    D.setIdentity();
    D *= (-h_ / g_);

    // Initial CoM state constraint
    Eigen::Vector4d x_com_mpc_init;
    x_com_mpc_init << x_com_init_.head<2>(), xd_com_init_.head<2>();  // (x, y, vx, vy)

    prog.AddLinearEqualityConstraint(X.col(0) == x_com_mpc_init);

    // Dynamics constraints for each time step
    for (int k = 0; k < n_steps_mpc - 1; ++k) {
        prog.AddLinearEqualityConstraint(X.col(k+1) == A * X.col(k) + B * U.col(k));
    }

    // ZMP tracking objective
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * 0.01;

    for (int k = 0; k < n_steps_mpc; ++k) {
        Eigen::Vector2d zmp_ref = zmp_trajectory_.value(k * dt);
        // ✅ Define new decision variable for ZMP
        drake::solvers::VectorXDecisionVariable zmp_actual = prog.NewContinuousVariables(2, "zmp_actual");

        // ✅ Add constraint to enforce ZMP equation: zmp_actual = C * X + D * U
        prog.AddLinearEqualityConstraint(zmp_actual == C * X.col(k) + D * U.col(k));

        // ✅ Apply quadratic cost for ZMP tracking
        prog.AddQuadraticErrorCost(Q, zmp_ref, zmp_actual);

        // Apply quadratic cost for control effort minimization
        prog.AddQuadraticErrorCost(R, Eigen::Vector2d::Zero(), U.col(k));
    }

    // 🔥 **Terminal Constraints (Final Time Step)**
    int k_final = n_steps_mpc - 1;

    // ✅ Terminal CoM Position: CoM should be close to the final ZMP
    Eigen::Vector2d final_zmp = zmp_trajectory_.value(total_time_);
    prog.AddLinearEqualityConstraint(X.topRows(2).col(k_final) == final_zmp);

    // ✅ Terminal Velocity: CoM velocity should approach zero
    prog.AddLinearEqualityConstraint(X.middleRows(2, 2).col(k_final) == Eigen::Vector2d::Zero());

    // Solve the optimization problem
    MathematicalProgramResult result = Solve(prog);
    if (!result.is_success()) {
        throw std::runtime_error("MPC failed to solve CoM trajectory!");
    }

    // Extract optimal CoM trajectory
    Eigen::MatrixXd X_sol = result.GetSolution(X);

    // Convert solution into a PiecewisePolynomial
    Eigen::VectorXd break_times = Eigen::VectorXd::LinSpaced(n_steps_mpc, 0, total_time_);
    // Store the CoM trajectory using cubic polynomial interpolation
    com_trajectory_ = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
        break_times, 
        X_sol.topRows(2),  // Extract CoM positions x, y
        X_sol.middleRows(2, 2) // Extract CoM velocities xd, yd
    );
}

void WalkingFSM::GenerateFootTrajectories() {
    std::cout << "GenerateFootTrajectories() called." << std::endl;

    // **Step 1: Create full break times**
    Eigen::VectorXd break_times = Eigen::VectorXd::LinSpaced(2 * n_phases_ + 1, 0, total_time_);

    // **Step 2: Initialize foot position & velocity matrices**
    Eigen::Matrix3Xd lf_knots = Eigen::Matrix3Xd::Zero(3, break_times.size());
    Eigen::Matrix3Xd rf_knots = Eigen::Matrix3Xd::Zero(3, break_times.size());

    Eigen::Matrix3Xd lf_dot_knots = Eigen::Matrix3Xd::Zero(3, break_times.size());
    Eigen::Matrix3Xd rf_dot_knots = Eigen::Matrix3Xd::Zero(3, break_times.size());

    int lf_idx = 0, rf_idx = 0;

    // **Step 3: Iterate Over Break Times to Assign Foot Placements**
    for (int i = 0; i < break_times.size(); ++i) {
        double t = break_times(i);
        double phase_time = std::fmod(t / step_time_, 4.0); // Compute phase within 4-phase cycle
        Eigen::Vector3d lf_ref, rf_ref;

        if (1.0 < phase_time && phase_time < 2.0) {
            // **Right Foot Swing Phase**
            rf_ref = right_foot_placements_[rf_idx]; 
            rf_ref(0) += step_length_ / 2.0;  // Move forward to midpoint
            rf_ref(2) += step_height_;  // Lift up to designated height
            rf_dot_knots(0, i) = step_length_ / step_time_; // Forward velocity

            // Left foot remains at the same place
            lf_ref = left_foot_placements_[lf_idx];

            rf_idx++;  // Move right foot to the next placement

        } else if (3.0 < phase_time && phase_time < 4.0) {
            // **Left Foot Swing Phase**
            lf_ref = left_foot_placements_[lf_idx];
            lf_ref(0) += step_length_ / 2.0;  // Move forward to midpoint
            lf_ref(2) += step_height_;  // Lift up to designated height
            lf_dot_knots(0, i) = step_length_ / step_time_; // Forward velocity

            // Right foot remains at the same place
            rf_ref = right_foot_placements_[rf_idx];

            lf_idx++;  // Move left foot to the next placement

        } else {
            // **Double Support Phase**
            lf_ref = left_foot_placements_[lf_idx];
            rf_ref = right_foot_placements_[rf_idx];
        }

        lf_knots.col(i) = lf_ref;
        rf_knots.col(i) = rf_ref;
    }

    // **Step 4: Create Piecewise Polynomial Trajectories**
    left_foot_trajectory_ = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
        break_times, lf_knots, lf_dot_knots);

    right_foot_trajectory_ = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
        break_times, rf_knots, rf_dot_knots);
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
