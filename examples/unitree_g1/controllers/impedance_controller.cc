// impedance_controller.cc - Source File
#include "examples/unitree_g1/includes/impedance_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree.h"

#include <iostream>

#include <drake/common/eigen_types.h>

namespace drake {
namespace examples {
namespace unitree_g1 {
ImpedanceController::ImpedanceController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    Eigen::VectorX<double> stiffness, Eigen::VectorX<double> damping_ratio)
    : plant_(plant),
      context_(context),
      stiffness_(stiffness),
      damping_ratio_(damping_ratio) {
  contact_points_ = GetFootContactPoints();
}

std::vector<Eigen::Vector3d> ImpedanceController::GetFootContactPoints() const {
  return {Eigen::Vector3d(-0.007, 0.008, -0.1),
          Eigen::Vector3d(-0.007, -0.008, -0.1),
          Eigen::Vector3d(0.020, -0.008, -0.1),
          Eigen::Vector3d(0.020, 0.008, -0.1)};
}

// Efficient computation of the null-space projection matrix using QR
MatrixX<double> ImpedanceController::ComputeNullSpaceProjectionQR(
    const MatrixX<double>& J_c) {
  int n = J_c.cols();  // Number of generalized velocities (DOFs)
  int m = J_c.rows();  // Number of constraints
  MatrixX<double> I = MatrixX<double>::Identity(n, n);
  // Compute Jc * Jc^T
  MatrixX<double> JcJT = J_c * J_c.transpose();

  // Efficiently add damping to the diagonal to prevents singularity
  JcJT.diagonal().array() += 1e-6;

  // Solve for the inverse Use regularized LLT
  MatrixX<double> JcJT_inv = JcJT.llt().solve(MatrixX<double>::Identity(m, m));

  // Compute the null-space projection matrix
  MatrixX<double> N_c = I - J_c.transpose() * JcJT_inv * J_c;

  return N_c;
}

MatrixX<double> ImpedanceController::ComputeContactJacobian(
    const drake::multibody::Frame<double>& foot_frame,
    const std::vector<Eigen::Vector3d>& contact_points) {
  int num_contacts = contact_points.size();
  int num_velocities = plant_.num_velocities();

  // Contact Jacobian: Each point contributes a 3xN matrix (x, y, z velocity)
  MatrixX<double> J_c(3 * num_contacts, num_velocities);

  for (int i = 0; i < num_contacts; ++i) {
    MatrixX<double> J_temp(3, num_velocities);

    // Compute translational velocity Jacobian for each contact point
    plant_.CalcJacobianTranslationalVelocity(
        context_,
        drake::multibody::JacobianWrtVariable::kV,  // Compute w.r.t. joint
                                                    // velocities
        foot_frame,
        contact_points[i],     // Contact point relative to the foot frame
        plant_.world_frame(),  // Express in world frame
        plant_.world_frame(),  // Measured in world frame
        &J_temp);

    // Stack the Jacobian
    J_c.block(3 * i, 0, 3, num_velocities) = J_temp;
  }

  return J_c;
}

Eigen::VectorXd ImpedanceController::CalcTorque(
    Eigen::VectorXd desired_position) {
  // **Compute stiffness torque**
  const drake::VectorX<double> state_position = plant_.GetPositions(context_);
  Eigen::VectorXd position_error = desired_position - state_position;
  Eigen::VectorXd u_stiffness =
      (stiffness_.array() * position_error.array()).matrix();
  // Compute damping torque
  const drake::VectorX<double> state_velocity = plant_.GetVelocities(context_);
  const int num_v = plant_.num_velocities();
  // Compute mass matrix H
  Eigen::MatrixXd Mass_matrix(num_v, num_v);
  plant_.CalcMassMatrixViaInverseDynamics(context_, &Mass_matrix);
  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = Mass_matrix.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();

  // Compute damping torque.
  Eigen::VectorXd u_damping =
      -(damping_gains * state_velocity.array()).matrix();

  std::vector<Eigen::Vector3d> contact_points = GetFootContactPoints();
  const auto& right_foot = plant_.GetBodyByName("right_ankle_roll_link");
  MatrixX<double> J_c_right_feet =
      ComputeContactJacobian(right_foot.body_frame(), contact_points);
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");
  MatrixX<double> J_c_left_feet =
      ComputeContactJacobian(left_foot.body_frame(), contact_points);
  // Create final stacked Jacobian matrix
  MatrixX<double> J_c(J_c_right_feet.rows() * 2, num_v);
  // // Stack left and right foot Jacobians
  J_c.topRows(J_c_right_feet.rows()) = J_c_right_feet;
  J_c.bottomRows(J_c_left_feet.rows()) = J_c_left_feet;

  MatrixX<double> N_c = ComputeNullSpaceProjectionQR(J_c);
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // Compute the midpoint between the left and right foot positions
  const auto& X_WR = plant_.EvalBodyPoseInWorld(context_, right_foot);
  const auto& X_WL = plant_.EvalBodyPoseInWorld(context_, left_foot);

  Eigen::Vector3d com_cmd = 0.5 * (X_WL.translation() + X_WR.translation());

  // set 0.5 meter in the z-direction
  com_cmd.z() = 0.7;

  // Get the current CoM position
  drake::Vector3<double> com_position = plant_.CalcCenterOfMassPositionInWorld(context_);

  // Resize Jacobian explicitly
  Eigen::MatrixXd J_com(3, num_v);

  // Call Drake's API correctly (do NOT dereference context_)
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV,
      plant_.world_frame(), plant_.world_frame(), &J_com);

  // Compute CoM velocity
  Eigen::Vector3d com_velocity = J_com * state_velocity;

  // Compute task-space inertia matrix (Lambda)
  Eigen::MatrixXd Lambda_com =
      (J_com * Mass_matrix.inverse() * J_com.transpose()).inverse();

  // PD Controller for CoM
  Eigen::Vector3d Kp_com(100.0, 100.0, 100.0);
  Eigen::Vector3d Kd_com(10.0, 10.0, 10.0);
  Eigen::Vector3d com_accel_desired =
      Kp_com.cwiseProduct(com_cmd - com_position) +
      Kd_com.cwiseProduct(-com_velocity);

  // Compute raw torque command
  Eigen::VectorXd u_com_raw =
      J_com.transpose() * Lambda_com * com_accel_desired;

  // Project torques into null space
  Eigen::VectorXd u_com = N_c * u_com_raw;

  // Final torque command
  return u_stiffness.tail(num_v) + u_damping - N_c * tau_g + u_com;
  // return Eigen::VectorXd::Zero(num_v);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake