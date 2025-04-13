// QPController.cc - Source File
#include "examples/unitree_g1/includes/QPController.h"

#include <iostream>

#include <drake/common/eigen_types.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
QPController::QPController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    Eigen::VectorX<double> stiffness, Eigen::VectorX<double> damping_ratio)
    : plant_(plant),
      context_(context),
      stiffness_(stiffness),
      damping_ratio_(damping_ratio) {
  contact_points_ = GetFootContactPoints();
}

std::vector<Eigen::Vector3d> QPController::GetFootContactPoints() const {
  return {Eigen::Vector3d(-0.02, 0.03, -0.001),
          Eigen::Vector3d(-0.02, -0.03, -0.001),
          Eigen::Vector3d(0.050, -0.03, -0.001),
          Eigen::Vector3d(0.050, 0.03, -0.001)};
}

// Robust dynamically consistent null-space projection with
// CompleteOrthogonalDecomposition fallback
MatrixX<double> QPController::ComputeNullSpaceProjection(
    const MatrixX<double>& J_c, const MatrixX<double>& Mass_matrix) {
  int n = J_c.cols();  // Number of generalized velocities (DOFs)

  int num_a = plant_.num_actuators();  // number of actuated DoFs

  // Projected Jacobian and actuated mass submatrix
  Eigen::MatrixXd J_c_U = J_c.bottomRightCorner(J_c.rows(), num_a);
  Eigen::MatrixXd M = Mass_matrix.block(n - num_a, n - num_a, num_a, num_a);

  // Regularize M to avoid numerical issues during inversion
  M.diagonal().array() += 1e-6;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_a, num_a);

  // Compute the operational space inertia matrix: Lambda = J_c_U * M^-1 *
  // J_c_U^T
  Eigen::MatrixXd M_inv;
  Eigen::LLT<Eigen::MatrixXd> llt(M);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error(
        "Mass matrix inversion failed: M is not positive definite.");
  }
  M_inv = llt.solve(Eigen::MatrixXd::Identity(num_a, num_a));

  Eigen::MatrixXd Lambda = J_c_U * M_inv * J_c_U.transpose();

  // Regularize Lambda to avoid singularities
  Lambda.diagonal().array() += 1e-6;

  // Compute pseudo-inverse of Lambda using CompleteOrthogonalDecomposition
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(Lambda);
  Eigen::MatrixXd Lambda_pinv = cod.pseudoInverse();

  // Dynamically consistent pseudoinverse
  Eigen::MatrixXd J_dyn_pinv = M_inv * J_c_U.transpose() * Lambda_pinv;

  // Compute null-space projection in actuator subspace
  Eigen::MatrixXd N_c = I - J_dyn_pinv * J_c_U;

  // Embed into full-dimension matrix (n x n) with top-left block identity and
  // bottom-right N_c
  Eigen::MatrixXd N_c_return = Eigen::MatrixXd::Identity(n, n);
  N_c_return.block(n - num_a, n - num_a, num_a, num_a) = N_c;

  return N_c_return;
}

MatrixX<double> QPController::ComputeJacobianPseudoInverse(
    const MatrixX<double>& J, double damping_eps) {
  if (J.rows() < J.cols()) {
    // Underdetermined: use Jᵗ (JJᵗ)⁻¹
    MatrixX<double> JJt = J * J.transpose();
    JJt.diagonal().array() += damping_eps;
    MatrixX<double> JJt_inv =
        JJt.llt().solve(MatrixX<double>::Identity(J.rows(), J.rows()));
    return J.transpose() * JJt_inv;
  } else {
    // Overdetermined: use (JᵗJ)⁻¹ Jᵗ
    MatrixX<double> JtJ = J.transpose() * J;
    JtJ.diagonal().array() += damping_eps;
    MatrixX<double> JtJ_inv =
        JtJ.llt().solve(MatrixX<double>::Identity(J.cols(), J.cols()));
    return JtJ_inv * J.transpose();
  }
}

MatrixX<double> QPController::ComputeContactJacobian(
    const drake::multibody::Frame<double>& foot_frame,
    const std::vector<Eigen::Vector3d>& contact_points) {
  int num_contacts = contact_points.size();
  const int num_v = plant_.num_velocities();
  //   const int num_a = plant_.num_actuators();   // number of actuated DoFs

  //   // Assumes the first 6 DoFs are unactuated (floating base)
  //   Eigen::MatrixXd U = Eigen::MatrixXd::Zero(num_v, num_v);
  //   U.block(num_v - num_a, num_v - num_a, num_a, num_a) =
  //   Eigen::MatrixXd::Identity(num_a, num_a); Contact Jacobian: Each point
  //   contributes a 3xN matrix (x, y, z velocity)
  MatrixX<double> J_c(3 * num_contacts, num_v);

  for (int i = 0; i < num_contacts; ++i) {
    MatrixX<double> J_temp(3, num_v);

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
    J_c.block(3 * i, 0, 3, num_v) = J_temp;
  }

  return J_c;
}

MatrixX<double> QPController::ComputContactBias(
    const drake::multibody::Frame<double>& foot_frame,
    const std::vector<Eigen::Vector3d>& contact_points) {
  int num_contacts = contact_points.size();
  int num_velocities = plant_.num_velocities();

  // Contact Jacobian: Each point contributes a 3xN vector (x, y, z velocity)
  VectorX<double> JdotV_r(3 * num_contacts);

  for (int i = 0; i < num_contacts; ++i) {
    MatrixX<double> J_temp(3, num_velocities);

    VectorX<double> JdotV_r_temp = plant_.CalcBiasTranslationalAcceleration(
        context_, drake::multibody::JacobianWrtVariable::kV,
        foot_frame,            // Measured frame
        contact_points[0],     // Point of interest
        plant_.world_frame(),  // Expressed in frame
        plant_.world_frame()   // Measured in frame
    );
    // Stack the Jacobian
    JdotV_r.segment<3>(3 * i) = JdotV_r_temp;
  }

  return JdotV_r;
}

Eigen::VectorXd QPController::CalcTorque(Eigen::VectorXd desired_position,
                                         Eigen::VectorXd tau_sensor) {
  return Eigen::VectorXd::Zero(plant_.num_velocities());
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake