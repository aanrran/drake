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
  return {Eigen::Vector3d(-0.007, 0.008, -0.001),
          Eigen::Vector3d(-0.007, -0.008, -0.001),
          Eigen::Vector3d(0.020, -0.008, -0.001),
          Eigen::Vector3d(0.020, 0.008, -0.001)};
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
  MatrixX<double> J_right_feet =
      ComputeContactJacobian(right_foot.body_frame(), contact_points);
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");
  MatrixX<double> J_left_feet =
      ComputeContactJacobian(left_foot.body_frame(), contact_points);
  // Create final stacked Jacobian matrix
  MatrixX<double> J_r = J_right_feet;

  //     MatrixX<double> J_r(J_right_feet.rows() * 2, num_v);
  //     // Stack left and right foot Jacobians
  //     J_r.topRows(J_left_feet.rows()) = J_right_feet;
  //     J_r.bottomRows(J_left_feet.rows()) = J_left_feet;

  MatrixX<double> N_r = ComputeNullSpaceProjectionQR(J_r);
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // 3. Compute vdot (generalized accelerations)
  auto vdot = plant_.get_generalized_acceleration_output_port().Eval(context_);

  // 4. Compute dynamics terms

  VectorX<double> C(num_v);
  plant_.CalcBiasTerm(context_, &C);  // b + g

  // 5. Get tau from actuator input

  // 6. Compute residual: J^T f_r
  VectorX<double> contact_estimate =
      Mass_matrix * vdot + C + tau_g - (u_damping + u_stiffness.tail(num_v));

  // (1) Compute J_r and pseudoinverse
  MatrixX<double> J_r_pinv = ComputeJacobianPseudoInverse(J_r * U);
  // (2) Compute J̇·v
  VectorX<double> JdotV_r =
      ComputContactBias(right_foot.body_frame(), contact_points);
  // (3) Compute desired acceleration
  double alpha_r = 0.7;  // damping factor
  VectorX<double> accel_task_r =
      -J_r_pinv * (JdotV_r + alpha_r * J_r * state_velocity);

  // (4) Compute task torque
  VectorX<double> tau_r =
      Mass_matrix * accel_task_r + C + tau_g - contact_estimate;

  // Compute the midpoint between the left and right foot positions
  const auto& X_WR = plant_.EvalBodyPoseInWorld(context_, right_foot);
  const auto& X_WL = plant_.EvalBodyPoseInWorld(context_, left_foot);

  Eigen::Vector3d com_cmd = 0.5 * (X_WR.translation() + X_WL.translation());

  // set 0.5 meter in the z-direction
  com_cmd.z() += 0.65;

  // Get the current CoM position
  drake::Vector3<double> com_position =
      plant_.CalcCenterOfMassPositionInWorld(context_);

  // Resize Jacobian explicitly
  Eigen::MatrixXd J_com(3, num_v);

  // Call Drake's API correctly (do NOT dereference context_)
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_com);
  Eigen::MatrixXd J_com_r = J_com * N_r;
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

  MatrixX<double> J_com_r_pinv = ComputeJacobianPseudoInverse(J_com * U * N_r);

  // Compute the desired acceleration for the CoM
  Eigen::Vector3d com_accel =
      J_com_r_pinv * (com_accel_desired - com_bias_trans);
  // Compute raw torque command
  Eigen::VectorXd u_com =
      Mass_matrix * com_accel + N_r * (C + tau_g - contact_estimate);
  Eigen::MatrixXd N_com_r = ComputeNullSpaceProjectionQR(J_com_r);

  const auto& torso = plant_.GetBodyByName("torso_link");
  const math::RigidTransform<double> torso_pose =
      plant_.EvalBodyPoseInWorld(context_, torso);
  drake::math::RotationMatrix<double> torso_rotaion_matrix =
      torso_pose.rotation();
  // Convert to RPY (in radians)
  drake::math::RollPitchYaw<double> torso_rpy(torso_rotaion_matrix);
  Eigen::Vector3d torso_rpy_angles = torso_rpy.vector();  // [roll, pitch, yaw]
  Eigen::Vector3d torso_translation_pos =
      torso_pose.translation();  // [x, y, z] in world frame
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
  const auto torso_bias_trans = torso_spacial_bias.translational();
  // PD Controller for torso translation
  Eigen::Vector3d Kp_torso_trans(18.0, 18.0, 18.0);
  Eigen::Vector3d Kd_torso_trans(1.7, 1.7, 1.7);
  Eigen::Vector3d torso_trans_accel_desired =
      Kp_torso_trans.cwiseProduct(com_cmd - torso_translation_pos) +
      Kd_torso_trans.cwiseProduct(-torso_spatial_vel.translational());

  MatrixX<double> J_torsoTrans_r_pinv =
      ComputeJacobianPseudoInverse(J_torso * U * N_r);

  // Compute the desired acceleration for the CoM
  Eigen::Vector3d torso_trans_accel =
      J_torsoTrans_r_pinv * (torso_trans_accel_desired - torso_bias_trans);
  // Compute raw torque command
  Eigen::VectorXd u_torsoTrans =
      Mass_matrix * torso_trans_accel + N_r * (C + tau_g - contact_estimate);
  Eigen::MatrixXd N_torsoTrans_r = ComputeNullSpaceProjectionQR(J_torso * N_r);

  // PD Controller for torso
  Eigen::Vector3d Kp_torso(18.0, 18.0, 18.0);
  Eigen::Vector3d Kd_torso(0.7, 0.7, 0.7);
  Eigen::Vector3d torso_rpy_desired =
      Eigen::Vector3d::Zero();         // [roll, pitch, yaw]
  torso_rpy_desired << 0.0, 0.0, 0.0;  // [roll, pitch, yaw]
  Eigen::Vector3d torso_accel_desired =
      Kp_torso.cwiseProduct(torso_rpy_desired - torso_rpy_angles) +
      Kd_torso.cwiseProduct(-torso_spatial_vel.rotational());

  MatrixX<double> J_torso_com_r_pinv =
      ComputeJacobianPseudoInverse(J_torso * U * N_torsoTrans_r * N_r);

  // Compute the desired acceleration for the CoM
  Eigen::Vector3d torso_accel =
      J_torso_com_r_pinv * (torso_accel_desired - torso_bias_rot);
  // Compute raw torque command
  Eigen::VectorXd u_torsoRot =
      Mass_matrix * torso_accel +
      N_r * N_torsoTrans_r * (C + tau_g - contact_estimate);
  Eigen::MatrixXd N_torso_com_r =
      ComputeNullSpaceProjectionQR(J_torso * N_torsoTrans_r * N_r);

  return 1.0 * tau_r + u_com + u_torsoRot +
         0.3 * N_torso_com_r * (u_stiffness.tail(num_v) + u_damping) +
         0.7 * N_r * N_torsoTrans_r * N_torso_com_r *
             (u_stiffness.tail(num_v) + u_damping - contact_estimate);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake