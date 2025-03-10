// QPController.cc - Source File
#include "examples/unitree_g1/includes/QPController.h"
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree.h>
#include <drake/common/eigen_types.h>
#include <iostream>


QPController::QPController(const drake::multibody::MultibodyPlant<double>& plant)
    : plant_(plant) {
    // Step 1: Initialize solver
    solver_ = std::make_unique<drake::solvers::OsqpSolver>();
    context_ = plant_.CreateDefaultContext();
}

Eigen::VectorXd QPController::SolveQP(
    const Eigen::VectorXd& q, 
    const Eigen::VectorXd& q_dot, 
    const Eigen::VectorXd& x_task_des, 
    const Eigen::VectorXd& f_task_des) {
    using namespace drake::solvers;
    MathematicalProgram prog;

    // Step 2: Extract system properties
    const int num_joints = plant_.num_positions();
    const int num_contacts = f_task_des.size();

    // Step 3: Define variables: joint torques (tau) and contact forces (f_c)
    auto tau = prog.NewContinuousVariables(num_joints, "tau");
    auto f_c = prog.NewContinuousVariables(num_contacts, "f_c");

    // Step 4: Cost function - minimize weighted joint torques and forces
    prog.AddQuadraticCost((W_tau_ * (tau - tau_ref_)).squaredNorm());
    prog.AddQuadraticCost((W_f_ * (f_c - f_ref_)).squaredNorm());

    // Step 5: Compute rigid-body dynamics terms
    Eigen::MatrixXd M(num_joints, num_joints);
    plant_.CalcMassMatrix(*context_, &M);
    
    Eigen::VectorXd C(num_joints);
    plant_.CalcBiasTerm(*context_, &C); // Only pass two arguments
    
    Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(*context_);

    // Step 6: Compute Contact Jacobian
    Eigen::MatrixXd J_c(num_contacts * 3, num_joints);
    Eigen::MatrixXd J_temp(3, num_joints); // Temporary storage for each contact point
    
    for (int i = 0; i < num_contacts; ++i) {
        const drake::multibody::Frame<double>& frame = plant_.get_body(contact_bodies_[i]).body_frame();
        Eigen::Vector3d p_WC = contact_points_[i]; // Contact point in world frame
    
        plant_.CalcJacobianTranslationalVelocity(
            *context_, 
            drake::multibody::JacobianWrtVariable::kQDot, 
            frame, 
            Eigen::Matrix<double, 3, 1>(p_WC), 
            plant_.world_frame(), 
            plant_.world_frame(),
            &J_temp // Pass a pointer to store the result
        );
    
        J_c.block(i * 3, 0, 3, num_joints) = J_temp;
    }
    

    // Step 7: Rigid-body dynamics constraint
    prog.AddLinearEqualityConstraint(
        (M * ddq_ + C + tau_g - S_.transpose() * tau - J_c.transpose() * f_c).eval(),
        Eigen::VectorXd::Zero(num_joints)
    );

    // Step 8: Contact force constraints (Friction cone)
    prog.AddLinearConstraint(A_friction_ * f_c <= b_friction_);

    // Step 9: Torque limits
    prog.AddBoundingBoxConstraint(tau_min_, tau_max_, tau);

    // Step 10: Task constraints
    Eigen::MatrixXd A_eq = J_task_;  // Coefficients of ddq_
    Eigen::VectorXd b_eq = x_task_des - J_dot_task_ * q_dot;  // Right-hand side
    
    // Convert ddq_ into a vector of symbolic variables
    auto ddq_ = prog.NewContinuousVariables(num_joints, "ddq");

    // Add the linear equality constraint: A_eq * ddq_ = b_eq
    prog.AddLinearEqualityConstraint(A_eq, b_eq, ddq_);


    
    
    // Step 11: Solve the QP problem
    const auto result = solver_->Solve(prog);
    if (!result.is_success()) {
        throw std::runtime_error("QP solve failed!");
    }

    // Step 12: Return optimal torques
    return result.GetSolution(tau);
}
