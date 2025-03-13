// QPController.cc - Source File
#include "examples/unitree_g1/includes/QPController.h"
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/common/eigen_types.h>
#include <iostream>

QPController::QPController(const drake::multibody::MultibodyPlant<double>& plant, const drake::systems::Context<double>& context)
    : plant_(plant), context_(context) {
    // Step 1: Initialize solver
    solver_ = std::make_unique<drake::solvers::OsqpSolver>();
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

    // Step 3: Define variables: joint torques (tau), accelerations (ddq), contact forces (f_c), and next-step velocities (v_next)
    auto tau = prog.NewContinuousVariables(num_joints, "tau");
    auto ddq = prog.NewContinuousVariables(num_joints, "ddq");
    auto f_c = prog.NewContinuousVariables(num_contacts, "f_c");
    auto v_next = prog.NewContinuousVariables(num_joints, "v_next");
    auto q_next = prog.NewContinuousVariables(num_joints, "q_next");

    // Step 4: Cost function - minimize weighted joint torques and forces
    prog.AddQuadraticCost((W_tau_ * (tau - tau_ref_)).squaredNorm());
    prog.AddQuadraticCost((W_f_ * (f_c - f_ref_)).squaredNorm());

    // Step 5: Compute rigid-body dynamics terms
    Eigen::MatrixXd M(num_joints, num_joints);
    plant_.CalcMassMatrix(context_, &M);
    
    Eigen::VectorXd C(num_joints);
    plant_.CalcBiasTerm(context_, &C);

    Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

    // Step 6: Compute Contact Jacobian
    Eigen::MatrixXd J_c(num_contacts * 3, num_joints);
    Eigen::MatrixXd J_temp(3, num_joints);
    for (int i = 0; i < num_contacts; ++i) {
        const drake::multibody::Frame<double>& frame = plant_.get_body(contact_bodies_[i]).body_frame();
        Eigen::Vector3d p_WC = contact_points_[i];

        plant_.CalcJacobianTranslationalVelocity(
            context_, 
            drake::multibody::JacobianWrtVariable::kQDot, 
            frame, 
            p_WC, 
            plant_.world_frame(), 
            plant_.world_frame(),
            &J_temp
        );

        J_c.block(i * 3, 0, 3, num_joints) = J_temp;
    }

    // Step 7: Rigid-body dynamics constraint
    prog.AddLinearEqualityConstraint(
        (M * ddq + C + tau_g - S_.transpose() * tau - J_c.transpose() * f_c).eval(),
        Eigen::VectorXd::Zero(num_joints)
    );

    // Step 7: Contact force constraints (Friction cone)
    prog.AddLinearConstraint(A_friction_ * f_c <= b_friction_);

    // Step 8: Torque limits
    prog.AddBoundingBoxConstraint(tau_min_, tau_max_, tau);

    // Step 8: Center-of-mass constraint (CoM acceleration)
    prog.AddLinearEqualityConstraint(J_task_ * ddq + J_dot_task_ * q_dot, x_task_des);

    // Step 9: Angular momentum constraint
    prog.AddLinearEqualityConstraint(A_am_ * v_next, hdot_des_);

    // Step 10: Predictive velocity constraints (Euler integration)
    // v_next - dt_*ddq = q_dot
    Eigen::MatrixXd A_eq = Eigen::MatrixXd::Zero(num_joints, 2 * num_joints);
    A_eq << Eigen::MatrixXd::Identity(num_joints, num_joints), -dt_ * Eigen::MatrixXd::Identity(num_joints, num_joints);
    prog.AddLinearEqualityConstraint(A_eq, q_dot, {v_next, ddq});
    

    // Step 11: Predictive position constraints (Euler integration)
    // q_next - dt_*v_next = q
    A_eq << Eigen::MatrixXd::Identity(num_joints, num_joints), -dt_ * Eigen::MatrixXd::Identity(num_joints, num_joints);
    prog.AddLinearEqualityConstraint(A_eq, q, {q_next, v_next});
    

    // Step 12: Torque limits
    prog.AddBoundingBoxConstraint(tau_min_, tau_max_, tau);

    // Step 13: Solve the QP problem
    const auto result = solver_->Solve(prog);
    if (!result.is_success()) {
        throw std::runtime_error("QP solve failed!");
    }

    // Step 14: Return optimal torques
    return result.GetSolution(tau);
}
