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
    const Eigen::VectorXd& q_dot) {

    using namespace drake::solvers;
    MathematicalProgram prog;

    // Step 2: Extract system properties
    const int num_joints = plant_.num_positions();

    // Step 3: Define variables: joint torques (tau), accelerations (ddq), contact forces (f_c), and next-step velocities (v_next)
    auto tau = prog.NewContinuousVariables(num_joints, "tau");
    auto ddq = prog.NewContinuousVariables(num_joints, "ddq");
    auto f_c = prog.NewContinuousVariables(contact_bodies_.size() * 3, "f_c");
    // Step 4: Cost function - minimize weighted joint torques and forces
    // 1. CoM acceleration tracking cost
    Eigen::MatrixXd J_com_E_(6, num_joints);
    Eigen::VectorXd Jd_qd_com_E_(6);
    Eigen::MatrixXd W_com_ = Eigen::MatrixXd::Identity(6, 6) * 100;
    Eigen::VectorXd xdd_com_des(6);
    xdd_com_des << 0, 0, 0,  0, 0, 0.0;  

    // Get the current CoM position
    drake::Vector3<double> com_position = plant_.CalcCenterOfMassPositionInWorld(context_);

    // Compute full 6D Spatial Jacobian for CoM
    plant_.CalcJacobianSpatialVelocity(
        context_,
        drake::multibody::JacobianWrtVariable::kQDot,
        plant_.world_frame(),  // Measured frame
        com_position,          // point of interest (CoM position)
        plant_.world_frame(),  // expressed in frame
        plant_.world_frame(),  // measured frame
        &J_com_E_
    );

    // Compute bias term analytically (Correct Extraction)
    drake::multibody::SpatialAcceleration<double> bias_accel_com_ =
        plant_.CalcBiasSpatialAcceleration(
            context_,
            drake::multibody::JacobianWrtVariable::kQDot,
            plant_.world_frame(),  // Measured frame
            com_position,          // point of interest (CoM position)
            plant_.world_frame(),  // expressed in frame
            plant_.world_frame()   // measured frame
        );

    // Extract rotational (angular) and translational (linear) acceleration components
    Jd_qd_com_E_.head<3>() = bias_accel_com_.rotational();   // Extract angular acceleration bias
    Jd_qd_com_E_.tail<3>() = bias_accel_com_.translational(); // Extract linear acceleration bias

    
    prog.AddQuadraticCost((J_com_E_ * ddq + Jd_qd_com_E_ - xdd_com_des).transpose() * W_com_ *(J_com_E_ * ddq + Jd_qd_com_E_ - xdd_com_des));

    // 3. Joint acceleration regularization cost
    Eigen::VectorXd ddq_nom_ = Eigen::VectorXd::Zero(num_joints);  // Default nominal value;
    Eigen::MatrixXd W_joint_accel_ = Eigen::MatrixXd::Identity(num_joints, num_joints); // Example weight;

    prog.AddQuadraticCost((ddq - ddq_nom_).transpose() * W_joint_accel_ * (ddq - ddq_nom_));

    // 4. Left foot position tracking cost
    Eigen::MatrixXd J_left_E_(6, num_joints);
    Eigen::VectorXd Jd_qd_left_E_(6);
    Eigen::MatrixXd W_foot_= Eigen::MatrixXd::Identity(6, 6) * 100;
    Eigen::VectorXd xdd_left_des(6);
    xdd_left_des << 0, 0, 0,  0, 0, 0.0;
    const auto& left_foot_frame_ = plant_.GetBodyByName("left_foot").body_frame();
    // Compute full 6D Spatial Jacobian for Left Foot
    plant_.CalcJacobianSpatialVelocity(
        context_,
        drake::multibody::JacobianWrtVariable::kQDot,
        plant_.world_frame(),  // Measured frame
        com_position,          // point of interest
        left_foot_frame_,      // expressed in frame
        plant_.world_frame(),  // measured frame
        &J_left_E_
    );
    // Compute bias term analytically (Correct Extraction)
    drake::multibody::SpatialAcceleration<double> bias_accel_left_foot_ =
        plant_.CalcBiasSpatialAcceleration(
            context_,
            drake::multibody::JacobianWrtVariable::kQDot,
            plant_.world_frame(),  // Measured frame
            com_position,          // point of interest
            left_foot_frame_,      // expressed in frame
            plant_.world_frame()   // measured frame
        );

    // Extract rotational (angular) and translational (linear) acceleration components
    Jd_qd_left_E_.head<3>() = bias_accel_left_foot_.rotational();   // Extract angular acceleration bias
    Jd_qd_left_E_.tail<3>() = bias_accel_left_foot_.translational(); // Extract linear acceleration bias
    
    prog.AddQuadraticCost((J_left_E_ * ddq + Jd_qd_left_E_ - xdd_left_des).transpose() * W_foot_ *
            (J_left_E_ * ddq + Jd_qd_left_E_ - xdd_left_des));
    
    // 4. Left foot position tracking cost
    Eigen::MatrixXd J_right_E_(6, num_joints);
    Eigen::VectorXd Jd_qd_right_E_(6);
    Eigen::VectorXd xdd_right_des(6);
    xdd_right_des << 0, 0, 0,  0, 0, 0.0;
    const auto& right_foot_frame_ = plant_.GetBodyByName("right_foot").body_frame();
    // Compute full 6D Spatial Jacobian for Left Foot
    plant_.CalcJacobianSpatialVelocity(
        context_,
        drake::multibody::JacobianWrtVariable::kQDot,
        plant_.world_frame(),  // Measured frame
        com_position,          // point of interest
        right_foot_frame_,      // expressed in frame
        plant_.world_frame(),  // measured frame
        &J_right_E_
    );
    // Compute bias term analytically (Correct Extraction)
    drake::multibody::SpatialAcceleration<double> bias_accel_right_foot_ =
        plant_.CalcBiasSpatialAcceleration(
            context_,
            drake::multibody::JacobianWrtVariable::kQDot,
            plant_.world_frame(),  // Measured frame
            com_position,          // point of interest
            right_foot_frame_,      // expressed in frame
            plant_.world_frame()   // measured frame
        );

    // Extract rotational (angular) and translational (linear) acceleration components
    Jd_qd_right_E_.head<3>() = bias_accel_right_foot_.rotational();   // Extract angular acceleration bias
    Jd_qd_right_E_.tail<3>() = bias_accel_right_foot_.translational(); // Extract linear acceleration bias
    
    prog.AddQuadraticCost((J_right_E_ * ddq + Jd_qd_right_E_ - xdd_right_des).transpose() * W_foot_ *
            (J_right_E_ * ddq + Jd_qd_right_E_ - xdd_right_des));

    // Step 5: Compute rigid-body dynamics terms
    Eigen::MatrixXd M(num_joints, num_joints);
    plant_.CalcMassMatrix(context_, &M);
    
    Eigen::VectorXd C(num_joints);
    plant_.CalcBiasTerm(context_, &C);

    Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

    // Step 6: Compute Contact Jacobian
    Eigen::MatrixXd J_c(contact_points_.size() * 3, num_joints);
    Eigen::MatrixXd J_temp(3, num_joints);
    for (int i = 0; i < contact_points_.size(); ++i) {
        const auto& frame = plant_.get_body(contact_bodies_[i]).body_frame();
        plant_.CalcJacobianTranslationalVelocity(context_, drake::multibody::JacobianWrtVariable::kQDot,
                                                frame, contact_points_[i],
                                                plant_.world_frame(), plant_.world_frame(), &J_temp);
        J_c.block(i * 3, 0, 3, num_joints) = J_temp;
    }

    // Step 7: Rigid-body dynamics constraint
    prog.AddLinearEqualityConstraint(
        (M * ddq + C + tau_g - S_.transpose() * tau - J_c.transpose() * f_c).eval(),
        Eigen::VectorXd::Zero(num_joints)
    );
    prog.AddLinearEqualityConstraint(M * ddq + C + tau_g - S_.transpose() * tau - J_c.transpose() * f_c,
                                     Eigen::VectorXd::Zero(num_joints));
    // Step 7: Contact force constraints (Friction cone)
    prog.AddLinearConstraint(A_friction_ * f_c <= b_friction_);

    // Step 8: Torque limits
    prog.AddBoundingBoxConstraint(tau_min_, tau_max_, tau);

    // Step 8: Center-of-mass constraint (CoM acceleration)
    // prog.AddLinearEqualityConstraint(J_task_ * ddq + J_dot_task_ * q_dot, x_task_des);

    

    // Step 13: Solve the QP problem
    const auto result = solver_->Solve(prog);
    if (!result.is_success()) {
        throw std::runtime_error("QP solve failed!");
    }

    // Step 14: Return optimal torques
    return result.GetSolution(tau);
}
