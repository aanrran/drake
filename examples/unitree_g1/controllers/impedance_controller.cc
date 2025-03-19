// impedance_controller.cc - Source File
#include "examples/unitree_g1/includes/impedance_controller.h"

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
  Eigen::MatrixXd H(num_v, num_v);
  plant_.CalcMassMatrixViaInverseDynamics(context_, &H);
  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = H.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();

  // Compute damping torque.
  Eigen::VectorXd u_damping =
      -(damping_gains * state_velocity.array()).matrix();

  std::vector<Eigen::Vector3d> contact_points = GetFootContactPoints();
  const auto& right_foot_frame_ =
      plant_.GetBodyByName("right_ankle_roll_link").body_frame();
  MatrixX<double> J_c_right_feet =
      ComputeContactJacobian(right_foot_frame_, contact_points);
  const auto& left_foot_frame_ =
      plant_.GetBodyByName("left_ankle_roll_link").body_frame();
  MatrixX<double> J_c_left_feet =
      ComputeContactJacobian(left_foot_frame_, contact_points);
  // Create final stacked Jacobian matrix
  MatrixX<double> J_c(J_c_right_feet.rows() * 2, num_v);
  // // Stack left and right foot Jacobians
  J_c.topRows(J_c_right_feet.rows()) = J_c_right_feet;
  J_c.bottomRows(J_c_left_feet.rows()) = J_c_left_feet;

  MatrixX<double> N_c = ComputeNullSpaceProjectionQR(J_c);
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  return u_stiffness.tail(num_v) + u_damping - N_c*tau_g;
  // return Eigen::VectorXd::Zero(num_v);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake