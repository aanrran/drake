// impedance_controller.cc - Source File
#include "examples/unitree_g1/includes/impedance_controller.h"

#include <iostream>

#include <drake/common/eigen_types.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

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
  return {Eigen::Vector3d(-0.02, 0.03, -0.001),
          Eigen::Vector3d(-0.02, -0.03, -0.001),
          Eigen::Vector3d(0.050, -0.03, -0.001),
          Eigen::Vector3d(0.050, 0.03, -0.001)};
}

// Efficient computation of the null-space projection matrix using QR
MatrixX<double> ImpedanceController::ComputeNullSpaceProjection(
    const MatrixX<double>& J_c,  const MatrixX<double>& Mass_matrix) {
  int n = J_c.cols();  // Number of generalized velocities (DOFs)
  int m = J_c.rows();  // Number of constraints
  // Default Mass_matrix to identity if not provided
  MatrixX<double> M = Mass_matrix;
  if (M.size() == 0) {
    M = MatrixX<double>::Identity(n, n);
  }

  MatrixX<double> I = MatrixX<double>::Identity(n, n);

  // Compute the operational space inertia matrix: Lambda = (J * M^{-1} * J^T)^{-1}
  MatrixX<double> Lambda_inv = J_c * M.inverse() * J_c.transpose();

  // Regularize Lambda_inv to avoid singularities
  Lambda_inv.diagonal().array() += 1e-6;

  // Compute the dynamically consistent generalized inverse of J_c
  MatrixX<double> J_dyn_pinv = M.inverse() * J_c.transpose() * Lambda_inv.llt().solve(MatrixX<double>::Identity(m, m));

  // Dynamically consistent null-space projection matrix
  MatrixX<double> N_c = I - J_dyn_pinv * J_c;

  return N_c;
}

MatrixX<double> ImpedanceController::ComputeJacobianPseudoInverse(
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

MatrixX<double> ImpedanceController::ComputeContactJacobian(
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

MatrixX<double> ImpedanceController::ComputContactBias(
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
  const int num_a = plant_.num_actuators();  // number of actuated DoFs

  // Assumes the first 6 DoFs are unactuated (floating base)
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(num_v, num_v);
  U.block(num_v - num_a, num_v - num_a, num_a, num_a) =
      Eigen::MatrixXd::Identity(num_a, num_a);
  // Compute mass matrix H
  Eigen::MatrixXd Mass_matrix(num_v, num_v);
  plant_.CalcMassMatrixViaInverseDynamics(context_, &Mass_matrix);

  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // 3. Compute vdot (generalized accelerations)
  auto vdot = plant_.get_generalized_acceleration_output_port().Eval(context_);

  // 4. Compute dynamics terms

  VectorX<double> C(num_v);
  plant_.CalcBiasTerm(context_, &C);  // b + g

  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = Mass_matrix.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();

  // Compute damping torque.
  Eigen::VectorXd u_damping =
      -(damping_gains * state_velocity.array()).matrix();

  // 6. Compute residual: J^T f_r
  VectorX<double> contact_estimate =
      Mass_matrix * vdot + C - (u_stiffness.tail(num_v) + u_damping);

  // Compute contact Jacobian
  std::vector<Eigen::Vector3d> contact_points = GetFootContactPoints();
  const auto& right_foot = plant_.GetBodyByName("right_ankle_roll_link");
  MatrixX<double> J_right_feet =
      ComputeContactJacobian(right_foot.body_frame(), contact_points);

  // Create final stacked Jacobian matrix
  MatrixX<double> J_r = J_right_feet;

  //     MatrixX<double> J_r(J_right_feet.rows() * 2, num_v);
  //     // Stack left and right foot Jacobians
  //     J_r.topRows(J_left_feet.rows()) = J_right_feet;
  //     J_r.bottomRows(J_left_feet.rows()) = J_left_feet;

  MatrixX<double> N_r = ComputeNullSpaceProjection(J_r, Mass_matrix);

  // (1) Compute J_r and pseudoinverse
  MatrixX<double> J_r_pinv = ComputeJacobianPseudoInverse(J_r * U);
  // (2) Compute J̇·v
  VectorX<double> JdotV_r =
      ComputContactBias(right_foot.body_frame(), contact_points);
  // (3) Compute desired acceleration
  double alpha_r = 0.7;  // damping factor
  VectorX<double> accel_task_r =
      vdot - J_r_pinv * (JdotV_r + alpha_r * J_r * state_velocity + J_r * vdot);

  // (4) Compute task torque
  VectorX<double> u_r = Mass_matrix * accel_task_r;

  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");
  Eigen::Vector3d left_translation_pos =
      plant_.EvalBodyPoseInWorld(context_, left_foot)
          .translation();  // [x, y, z] in world frame
  std::cout << "left_translation_pos: " << left_translation_pos.transpose()
            << std::endl;
  MatrixX<double> J_left_feet(3, num_v);
  // Compute the Jacobian for the left foot
  plant_.CalcJacobianTranslationalVelocity(
      context_,
      drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
      left_foot.body_frame(),                     // frame to differentiate
      Eigen::Vector3d::Zero(),                    // Point of interest
      plant_.world_frame(),                       // expressed in world
      plant_.world_frame(),                       // measured in world
      &J_left_feet);

  Eigen::MatrixXd N_left_r = ComputeNullSpaceProjection(J_left_feet * N_r, Mass_matrix);

  // (1) Compute J_r and pseudoinverse
  MatrixX<double> J_left_r_pinv =
      ComputeJacobianPseudoInverse(J_left_feet * U * N_r);

  // Compute the bias acceleration (Coriolis and centrifugal effects)
  Eigen::VectorXd left_bias_trans = plant_.CalcBiasTranslationalAcceleration(
      context_,
      drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
      left_foot.body_frame(),                     // Measured frame
      Eigen::Vector3d::Zero(),                    // Point of interest
      plant_.world_frame(),                       // Expressed in world frame
      plant_.world_frame()                        // Measured relative to world
  );

  // 1. Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::MatrixXd Lambda_left_inv =
      (J_left_feet * Mass_matrix.inverse() * J_left_feet.transpose());
  // PD Controller for torso
  Eigen::Vector3d Kp_left(30.0, 30.0, 30.0);
  Eigen::Vector3d Kd_left(1.7, 1.7, 1.7);
  Eigen::Vector3d left_trans_desired(-0.0, 0.1, 0.3);  // [x, y, z]
  Eigen::Vector3d left_accel_desired =
      Lambda_left_inv *
          Kp_left.cwiseProduct(left_trans_desired - left_translation_pos) +
      Lambda_left_inv * Kd_left.cwiseProduct(-J_left_feet * state_velocity);
  // (3) Compute desired acceleration
  VectorX<double> accel_task_left =
      vdot + J_left_r_pinv *
                 (left_accel_desired - left_bias_trans - J_left_feet * vdot);

  // (4) Compute task torque
  VectorX<double> u_left = Mass_matrix * accel_task_left;

  // Compute the midpoint between the left and right foot positions
  const auto& X_WR = plant_.EvalBodyPoseInWorld(context_, right_foot);
  //   const auto& X_WL = plant_.EvalBodyPoseInWorld(context_, left_foot);

  Eigen::Vector3d com_cmd = X_WR.translation() + Eigen::Vector3d(0.0, 0.0, 0.0);

  // set 0.5 meter in the z-direction
  com_cmd.z() += 0.7;

  // Get the current CoM position
  drake::Vector3<double> com_position =
      plant_.CalcCenterOfMassPositionInWorld(context_);

  // Resize Jacobian explicitly
  Eigen::MatrixXd J_com(3, num_v);

  // Call Drake's API correctly (do NOT dereference context_)
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_com);
  Eigen::MatrixXd J_com_r = J_com * N_left_r * N_r;
  Eigen::MatrixXd N_com_r = ComputeNullSpaceProjection(J_com_r, Mass_matrix);
  // Compute CoM velocity
  Eigen::Vector3d com_velocity = J_com * state_velocity;
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
  const auto com_bias_trans = com_spacial_bias.translational();
  const auto com_bias_rot = com_spacial_bias.rotational();
  // PD Controller for CoM
  Eigen::Vector3d Kp_com(18.0, 18.0, 18.0);
  Eigen::Vector3d Kd_com(0.7, 0.7, 0.7);
  Eigen::Vector3d com_accel_desired =
      Kp_com.cwiseProduct(com_cmd - com_position) +
      Kd_com.cwiseProduct(-com_velocity);

  MatrixX<double> J_com_r_pinv =
      ComputeJacobianPseudoInverse(J_com * U * N_left_r * N_r);

  // Compute the desired acceleration for the CoM
  Eigen::VectorXd com_accel =
      vdot + J_com_r_pinv * (com_accel_desired - com_bias_trans - J_com * vdot);

  // 1. Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_M(Mass_matrix);
  double lambda_M = eigen_solver_M.eigenvalues().maxCoeff();

  // 2. Compute λ_J_com (Largest eigenvalue of J_com^T * J_com)
  Eigen::MatrixXd JcomJcomt = J_com.transpose() * J_com;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_Jcom(JcomJcomt);
  double lambda_J_com = eigen_solver_Jcom.eigenvalues().maxCoeff();
  double alpha_com = 0.2;  // chosen conservatively
  double mass_total =
      plant_.CalcTotalMass(context_);  // Drake's total mass retrieval function

  // Compute κ based on the eigenvalues computed above
  double kappa_max_com =
      (2.0 * lambda_M) /
      (alpha_com * alpha_com * mass_total * mass_total * lambda_J_com);

  // Choose κ smaller than this limit
  double kappa = 0.9 * kappa_max_com;  // safety margin of 10%
  Eigen::MatrixXd KD_com = 0.9 * Mass_matrix;
  // Compute raw torque command
  Eigen::VectorXd u_com = -tau_g +
                          2 * kappa * N_left_r * N_r * U * J_com.transpose() *
                              (com_cmd - com_position) -
                          KD_com * J_com_r_pinv * J_com * (state_velocity);
  // Mass_matrix * com_accel;

  const auto& torso = plant_.GetBodyByName("torso_link");
  const math::RigidTransform<double> torso_pose =
      plant_.EvalBodyPoseInWorld(context_, torso);
  drake::math::RotationMatrix<double> torso_rotaion_matrix =
      torso_pose.rotation();
  // Convert to RPY (in radians)
  drake::math::RollPitchYaw<double> torso_rpy(torso_rotaion_matrix);
  Eigen::Vector3d torso_rpy_angles = torso_rpy.vector();  // [roll, pitch, yaw]
  //   Eigen::Vector3d torso_translation_pos =
  //       torso_pose.translation();  // [x, y, z] in world frame
  const auto& torso_spatial_vel =
      plant_.EvalBodySpatialVelocityInWorld(context_, torso);
  MatrixX<double> J_torso(3, plant_.num_velocities());

  plant_.CalcJacobianAngularVelocity(
      context_,
      drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
      torso.body_frame(),                         // frame to differentiate
      plant_.world_frame(),                       // expressed in world
      plant_.world_frame(),                       // measured in world
      &J_torso);

  Eigen::MatrixXd N_torso_com_r =
      ComputeNullSpaceProjection(J_torso * N_com_r * N_left_r * N_r, Mass_matrix);
  // Compute the bias acceleration (Coriolis and centrifugal effects)
  const drake::multibody::SpatialAcceleration<double> torso_spacial_bias =
      plant_.CalcBiasSpatialAcceleration(
          context_,
          drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
          torso.body_frame(),                         // Measured frame
          Eigen::Vector3d::Zero(),                    // Point of interest
          plant_.world_frame(),  // Expressed in world frame
          plant_.world_frame()   // Measured relative to world
      );
  const auto torso_bias_rot = torso_spacial_bias.rotational();
  //   const auto torso_bias_trans = torso_spacial_bias.translational();

  // PD Controller for torso
  Eigen::Vector3d Kp_torso(28.0, 28.0, 28.0);
  Eigen::Vector3d Kd_torso(2.7, 2.7, 2.7);
  Eigen::Vector3d torso_rpy_desired =
      Eigen::Vector3d::Zero();         // [roll, pitch, yaw]
  torso_rpy_desired << 0.0, 0.0, 0.0;  // [roll, pitch, yaw]
  Eigen::Vector3d torso_accel_desired =
      Kp_torso.cwiseProduct(torso_rpy_desired - torso_rpy_angles) +
      Kd_torso.cwiseProduct(-torso_spatial_vel.rotational());

  MatrixX<double> J_torso_com_r_pinv =
      ComputeJacobianPseudoInverse(J_torso * U * N_com_r * N_left_r * N_r);

  // Compute the desired acceleration for the CoM
  Eigen::VectorXd torso_accel =
      vdot + J_torso_com_r_pinv *
                 (torso_accel_desired - torso_bias_rot - J_torso * vdot);
  // Compute raw torque command
  Eigen::VectorXd u_torsoRot = Mass_matrix * torso_accel;

  //   const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");
  //   Eigen::Vector3d left_translation_pos =
  //       plant_.EvalBodyPoseInWorld(context_, left_foot)
  //           .translation();  // [x, y, z] in world frame
  //   MatrixX<double> J_left_feet(3, num_v);
  //   // Compute the Jacobian for the left foot
  //   plant_.CalcJacobianTranslationalVelocity(
  //       context_,
  //       drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
  //       left_foot.body_frame(),                     // frame to differentiate
  //       Eigen::Vector3d::Zero(),                    // Point of interest
  //       plant_.world_frame(),                       // expressed in world
  //       plant_.world_frame(),                       // measured in world
  //       &J_left_feet);

  //   Eigen::MatrixXd N_left_torso_com_r =
  //       ComputeNullSpaceProjection(J_left_feet * N_torso_com_r * N_com_r *
  //       N_r);

  //   // (1) Compute J_r and pseudoinverse
  //   MatrixX<double> J_left_com_toso_r_pinv = ComputeJacobianPseudoInverse(
  //       J_left_feet * U * N_torso_com_r * N_com_r * N_r);

  //   // Compute the bias acceleration (Coriolis and centrifugal effects)
  //   Eigen::VectorXd left_bias_trans =
  //   plant_.CalcBiasTranslationalAcceleration(
  //       context_,
  //       drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
  //       left_foot.body_frame(),                     // Measured frame
  //       Eigen::Vector3d::Zero(),                    // Point of interest
  //       plant_.world_frame(),                       // Expressed in world
  //       frame plant_.world_frame()                        // Measured
  //       relative to world
  //   );
  //   // PD Controller for torso
  //   Eigen::Vector3d Kp_left(28.0, 28.0, 28.0);
  //   Eigen::Vector3d Kd_left(5.7, 5.7, 5.7);
  //   Eigen::Vector3d left_trans_desired =
  //       Eigen::Vector3d::Zero();          // [roll, pitch, yaw]
  //   left_trans_desired << -0.3, 0.1, 0.4;  // [roll, pitch, yaw]
  //   Eigen::Vector3d left_accel_desired =
  //       Kp_left.cwiseProduct(left_trans_desired - left_translation_pos) +
  //       Kd_left.cwiseProduct(-J_left_feet * state_velocity);
  //   // (3) Compute desired acceleration
  //   VectorX<double> accel_task_left =
  //       vdot + J_left_com_toso_r_pinv *
  //                  (left_accel_desired - left_bias_trans - J_left_feet *
  //                  vdot);

  //   // (4) Compute task torque
  //   VectorX<double> u_left = Mass_matrix * accel_task_left;

  //   return u_stiffness.tail(num_v) + u_damping - contact_estimate;
  //   return 0.0 * u_r + 0.0 * u_com + 0.0 * u_torsoRot + 1.0 * u_left +
  //          0.0 * N_r * N_left_r * N_com_r * N_torso_com_r *
  //              (u_stiffness.tail(num_v) + u_damping) -
  //          0.0 * contact_estimate;
  return u_left - tau_g +  ComputeNullSpaceProjection(J_left_feet, Mass_matrix).transpose()*(u_stiffness.tail(num_v) + u_damping);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake