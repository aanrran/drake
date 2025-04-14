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

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
QPController::ContactJacobianAndBias(
    const drake::multibody::Frame<double>& foot_frame,
    const std::vector<Eigen::Vector3d>& contact_points) {
  const int num_contacts = contact_points.size();
  const int num_v = plant_.num_velocities();

  Eigen::MatrixXd J_c(3 * num_contacts, num_v);
  Eigen::VectorXd JdotV_r(3 * num_contacts);

  for (int i = 0; i < num_contacts; ++i) {
    Eigen::MatrixXd J_temp(3, num_v);

    // Compute translational Jacobian at contact point i
    plant_.CalcJacobianTranslationalVelocity(
        context_, drake::multibody::JacobianWrtVariable::kV, foot_frame,
        contact_points[i],  // ✅ use i here
        plant_.world_frame(), plant_.world_frame(), &J_temp);

    // Compute J̇·v at the same contact point i
    Eigen::Vector3d JdotV_r_temp = plant_.CalcBiasTranslationalAcceleration(
        context_, drake::multibody::JacobianWrtVariable::kV, foot_frame,
        contact_points[i],  // ✅ use i here
        plant_.world_frame(), plant_.world_frame());

    J_c.block<3, Eigen::Dynamic>(3 * i, 0, 3, num_v) = J_temp;
    JdotV_r.segment<3>(3 * i) = JdotV_r_temp;
  }

  return std::make_pair(J_c, JdotV_r);
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> QPController::GetBodyJacobian(
    const drake::multibody::Frame<double>& foot_frame) {
  const int num_v = plant_.num_velocities();

  MatrixX<double> J_trans(3, num_v);
  MatrixX<double> J_rot(3, num_v);

  MatrixX<double> J_spacial(6, num_v);

  // Compute translational velocity Jacobian for each contact point
  plant_.CalcJacobianSpatialVelocity(
      context_,
      drake::multibody::JacobianWrtVariable::kV,  // Compute w.r.t. joint
                                                  // velocities
      foot_frame,
      Eigen::VectorXd::Zero(3),  // Center to the foot frame
      plant_.world_frame(),      // Express in world frame
      plant_.world_frame(),      // Measured in world frame
      &J_spacial);

  // Stack the Jacobian
  J_rot = J_spacial.topRows(3);
  J_trans = J_spacial.bottomRows(3);

  return std::make_pair(J_trans, J_rot);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> QPController::GetBodyBias(
    const drake::multibody::Frame<double>& foot_frame) {
  // Contact Jacobian: a 3x1 vector (x, y, z velocity)
  VectorX<double> JdotV_trans(3);
  VectorX<double> JdotV_rot(3);

  // Compute the bias acceleration (Coriolis and centrifugal effects)
  const drake::multibody::SpatialAcceleration<double> spacial_bias =
      plant_.CalcBiasSpatialAcceleration(
          context_, drake::multibody::JacobianWrtVariable::kV,
          foot_frame,                // Measured frame
          Eigen::VectorXd::Zero(3),  // Point of interest
          plant_.world_frame(),      // Expressed in frame
          plant_.world_frame()       // Measured in frame
      );
  JdotV_trans = spacial_bias.translational();
  JdotV_rot = spacial_bias.rotational();

  return std::make_pair(JdotV_trans, JdotV_rot);
}

Eigen::VectorXd QPController::CalcTorque(Eigen::VectorXd desired_position,
                                         Eigen::VectorXd tau_sensor) {
  // Dynamic
  const drake::VectorX<double> state_position = plant_.GetPositions(context_);
  const drake::VectorX<double> state_velocity = plant_.GetVelocities(context_);
  // compute the sensor torque
  const int num_v = plant_.num_velocities();
  const int num_a = plant_.num_actuators();  // number of actuated DoFs
  Eigen::VectorXd tau_sensor_full = Eigen::VectorXd::Zero(num_v);
  tau_sensor_full.tail(num_a) = tau_sensor;

  // Compute mass matrix H
  Eigen::MatrixXd M(num_v, num_v);
  plant_.CalcMassMatrixViaInverseDynamics(context_, &M);
  // Compute gravity forces
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);
  // Compute Coriolis and centrifugal forces
  VectorX<double> Cv(num_v);
  plant_.CalcBiasTerm(context_, &Cv);  // b
  // Extract the actuation matrix from the plant (B matrix).
  Eigen::MatrixXd S = plant_.MakeActuationMatrix().transpose();
  // Center of mass Jacobian
  Eigen::MatrixXd J_com(3, num_v);
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_com);
  // Nullspace projection
  Eigen::MatrixXd N = ComputeNullSpaceProjection(J_com, M);
  // CoM angular momentum jacobian
  Eigen::MatrixXd J_k(3, num_v);
  plant_.CalcJacobianAngularVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), plant_.world_frame(), &J_k);
  // coner of mass mementum matrix time derivative
  // Get the current CoM position
  drake::Vector3<double> com_position =
      plant_.CalcCenterOfMassPositionInWorld(context_);
  // Compute the bias acceleration (Coriolis and centrifugal effects)
  const drake::multibody::SpatialAcceleration<double> com_spacial_bias =
      plant_.CalcBiasSpatialAcceleration(
          context_,
          drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
          plant_.world_frame(),                       // Measured frame
          com_position,          // Point of interest (CoM position)
          plant_.world_frame(),  // Expressed in world frame
          plant_.world_frame()   // Measured relative to world
      );
  // const auto com_bias_trans = com_spacial_bias.translational();
  const auto Jd_qd_k = com_spacial_bias.rotational();

  // Compute Foot Jacobian
  const auto& right_foot = plant_.GetBodyByName("right_ankle_roll_link");
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");

  auto [J_right, J_rpyright] = GetBodyJacobian(right_foot.body_frame());
  auto [J_left, J_rpyleft] = GetBodyJacobian(left_foot.body_frame());

  auto [Jd_qd_right, Jd_qd_rpyright] = GetBodyBias(right_foot.body_frame());
  auto [Jd_qd_left, Jd_qd_rpyleft] = GetBodyBias(left_foot.body_frame());

  // Contact Jacobians
  std::vector<Eigen::Vector3d> contact_points = GetFootContactPoints();

  auto [J_right_contact, JdotV_right_contact] =
      ContactJacobianAndBias(right_foot.body_frame(), contact_points);
  auto [J_left_contact, JdotV_left_contact] =
      ContactJacobianAndBias(left_foot.body_frame(), contact_points);
  // stack the jacobians and bias
  MatrixX<double> contact_jacobians = Eigen::MatrixXd::Zero(
      J_right_contact.rows() + J_left_contact.rows(), num_v);
  contact_jacobians.topRows(J_right_contact.rows()) = J_right_contact;
  contact_jacobians.bottomRows(J_left_contact.rows()) = J_left_contact;
  Eigen::VectorXd contact_jacobians_dot_v = Eigen::VectorXd::Zero(
      JdotV_right_contact.size() + JdotV_left_contact.size());
  contact_jacobians_dot_v.head(JdotV_right_contact.size()) =
      JdotV_right_contact;
  contact_jacobians_dot_v.tail(JdotV_left_contact.size()) = JdotV_left_contact;

  // torso jacobian and bias
  const auto& torso = plant_.GetBodyByName("torso_link");

  auto [J_trans_torso, J_torso] = GetBodyJacobian(torso.body_frame());
  auto [Jd_qd_trans_torso, Jd_qd_torso] = GetBodyBias(torso.body_frame());

  // **Compute stiffness torque**
  Eigen::VectorXd position_error = desired_position - state_position;
  Eigen::VectorXd u_stiffness =
      (stiffness_.array() * position_error.array()).matrix();

  // Compute damping torque
  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = M.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();
  Eigen::VectorXd u_damping =
      -(damping_gains * state_velocity.array()).matrix();

  Eigen::VectorXd tau_o = u_stiffness.tail(num_v) + u_damping;

  // // compute the selection matrix S to
  // // unselect the floating base and upper body
  // Eigen::MatrixXd S_com = Eigen::MatrixXd::Zero(num_v, num_v);
  // // Opt out the floating joint (first 6 axes) and the last few joints from
  // // joint 14 to joint 31
  // S_com.bottomRightCorner(num_a, num_a) =
  //     Eigen::MatrixXd::Identity(num_a, num_a);
  // S_com.bottomRightCorner(12, 12) = Eigen::MatrixXd::Zero(12, 12);

  // // Compute the null space projection jacobian
  // MatrixX<double> J_com_null = J_com * S_com;
  // // Compute CoM velocity
  // Eigen::Vector3d com_velocity = J_com * state_velocity;
  // // Compute the bias acceleration (Coriolis and centrifugal effects)
  // const drake::multibody::SpatialAcceleration<double> com_spacial_bias =
  //     plant_.CalcBiasSpatialAcceleration(
  //         context_,
  //         drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
  //         plant_.world_frame(),                       // Measured frame
  //         com_position,          // Point of interest (CoM position)
  //         plant_.world_frame(),  // Expressed in world frame
  //         plant_.world_frame()   // Measured relative to world
  //     );
  // const auto com_bias_trans = com_spacial_bias.translational();
  // const auto com_bias_rot = com_spacial_bias.rotational();
  // // Compute inverse of λ com

  // MatrixX<double> J_com_pinv = ComputeJacobianPseudoInverse(J_com_null);

  // // 1. Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_M(M);
  // double lambda_M = eigen_solver_M.eigenvalues().maxCoeff();

  // double mass_total = plant_.CalcTotalMass(context_);

  return Eigen::VectorXd::Zero(plant_.num_velocities());
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake