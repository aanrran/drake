// WBController.cc - Source File
#include "examples/unitree_g1/includes/WBController.h"

#include <iostream>

#include <drake/common/eigen_types.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
WBController::WBController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    Eigen::VectorX<double> stiffness, Eigen::VectorX<double> damping_ratio)
    : plant_(plant),
      context_(context),
      stiffness_(stiffness),
      damping_ratio_(damping_ratio) {
  num_pos_ = plant_.num_positions();
  num_q_ = plant_.num_velocities();
  num_a_ = plant_.num_actuators();
  Ivv_ = Eigen::MatrixXd::Identity(num_q_, num_q_);
  Iaa_ = Eigen::MatrixXd::Identity(num_a_, num_a_);

  solver_ = std::make_unique<drake::solvers::OsqpSolver>();
  // Define variables: joint torques (tau), accelerations (ddq)
  tau_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau");
  qdd_ = prog_.NewContinuousVariables(plant_.num_velocities(), "qdd");
  u2_ = prog_.NewContinuousVariables(3, "u2");
  tau0_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau0");
  JTfr_ = prog_.NewContinuousVariables(plant_.num_velocities(), "JTfr");
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> WBController::GetBodyJacobian(
    const drake::multibody::Frame<double>& foot_frame) {
  MatrixX<double> J_trans(3, num_q_);
  MatrixX<double> J_rot(3, num_q_);

  MatrixX<double> J_spacial(6, num_q_);

  // Compute translational velocity Jacobian for each contact point
  plant_.CalcJacobianSpatialVelocity(
      context_,
      drake::multibody::JacobianWrtVariable::kV,  // Compute w.r.t. joint
                                                  // velocities
      foot_frame,
      Eigen::Vector3d::Zero(),  // Center to the foot frame
      plant_.world_frame(),     // Express in world frame
      plant_.world_frame(),     // Measured in world frame
      &J_spacial);

  // Stack the Jacobian
  J_rot = J_spacial.topRows(3);
  J_trans = J_spacial.bottomRows(3);

  return std::make_pair(J_trans, J_rot);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> WBController::GetBodyBias(
    const drake::multibody::Frame<double>& foot_frame) {
  // Contact Jacobian: a 3x1 vector (x, y, z velocity)
  VectorX<double> JdotV_trans(3);
  VectorX<double> JdotV_rot(3);

  // Compute the bias acceleration (Coriolis and centrifugal effects)
  const drake::multibody::SpatialAcceleration<double> spacial_bias =
      plant_.CalcBiasSpatialAcceleration(
          context_, drake::multibody::JacobianWrtVariable::kV,
          foot_frame,               // Measured frame
          Eigen::Vector3d::Zero(),  // Point of interest
          plant_.world_frame(),     // Expressed in frame
          plant_.world_frame()      // Measured in frame
      );
  JdotV_trans = spacial_bias.translational();
  JdotV_rot = spacial_bias.rotational();

  return std::make_pair(JdotV_trans, JdotV_rot);
}

Eigen::Vector3d WBController::GetRPYInWorld(
    const drake::multibody::Body<double>& body) const {
  const drake::math::RotationMatrix<double> R_WB =
      plant_.EvalBodyPoseInWorld(context_, body).rotation();
  return drake::math::RollPitchYaw<double>(R_WB).vector();
}

Eigen::MatrixXd WBController::ComputeJacobianPseudoInv(
    const Eigen::MatrixXd& Jacobian, const Eigen::MatrixXd& M_inverse,
    const Eigen::MatrixXd& N_pre) {
  Eigen::MatrixXd JN_pre = Jacobian * N_pre;
  Eigen::MatrixXd Lambda = JN_pre * M_inverse * JN_pre.transpose();

  // Regularize Lambda to avoid singularities
  Lambda.diagonal().array() += 1e-6;

  // Compute pseudo-inverse of Lambda using CompleteOrthogonalDecomposition
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(Lambda);
  Eigen::MatrixXd Lambda_pinv = cod.pseudoInverse();

  // Dynamically consistent pseudoinverse
  Eigen::MatrixXd J_dyn_pinv = M_inverse * JN_pre.transpose() * Lambda_pinv;
  return J_dyn_pinv;
}

Eigen::VectorXd WBController::CalcTorque(Eigen::VectorXd desired_position,
                                         Eigen::VectorXd tau_sensor) {
  // Dynamic
  const drake::VectorX<double> state_pos = plant_.GetPositions(context_);
  const drake::VectorX<double> state_q = plant_.GetVelocities(context_);
  // compute the sensor torque
  Eigen::VectorXd tau_sensor_full = Eigen::VectorXd::Zero(num_q_);
  tau_sensor_full.tail(num_a_) = tau_sensor;

  // Compute mass matrix H
  Eigen::MatrixXd Mass_matrix(num_q_, num_q_);
  plant_.CalcMassMatrix(context_, &Mass_matrix);
  // Compute gravity forces
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);
  // Compute Coriolis and centrifugal forces
  VectorX<double> Cv(num_q_);
  plant_.CalcBiasTerm(context_, &Cv);  // b
  // Extract the actuation matrix from the plant (B matrix).
  Eigen::MatrixXd Selection_matirx = Eigen::MatrixXd::Zero(num_q_, num_q_);
  Selection_matirx.bottomRightCorner(num_a_, num_a_) = Iaa_;
  // Center of mass Jacobian
  Eigen::MatrixXd J_com(3, num_q_);
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_com);
  // Nullspace projection
  Eigen::LLT<Eigen::MatrixXd> llt(Mass_matrix);
  DRAKE_DEMAND(llt.info() == Eigen::Success);
  Eigen::MatrixXd M_inverse = llt.solve(Ivv_);

  // Compute the pseudo-inverse of the Jacobian
  Eigen::MatrixXd J_dyn_pinv = ComputeJacobianPseudoInv(J_com, M_inverse, Ivv_);

  // Compute null-space projection in actuator subspace
  Eigen::MatrixXd N_com = -J_dyn_pinv * J_com;

  // Compute Foot Jacobian
  const auto& right_foot = plant_.GetBodyByName("right_ankle_roll_link");
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");

  auto [J_right, J_rpyright] = GetBodyJacobian(right_foot.body_frame());
  auto [J_left, J_rpyleft] = GetBodyJacobian(left_foot.body_frame());

  auto [Jd_qd_right, Jd_qd_rpyright] = GetBodyBias(right_foot.body_frame());
  auto [Jd_qd_left, Jd_qd_rpyleft] = GetBodyBias(left_foot.body_frame());

    // torso jacobian and bias
  const auto& torso = plant_.GetBodyByName("torso_link");

  auto [J_trans_torso, J_torso] = GetBodyJacobian(torso.body_frame());
  auto [Jd_qd_trans_torso, Jd_qd_torso] = GetBodyBias(torso.body_frame());

  // **Compute stiffness torque**
  Eigen::VectorXd position_error = desired_position - state_pos;
  Eigen::VectorXd u_stiffness =
      (stiffness_.array() * position_error.array()).matrix();

  // Compute damping torque
  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = Mass_matrix.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();
  Eigen::VectorXd u_damping = -(damping_gains * state_q.array()).matrix();

  Eigen::VectorXd tau_def_pose = u_stiffness.tail(num_q_) + u_damping;

  // Compute desired linear accelerations of the feet
  // Compute he desired acceleration for the left foot
  double Kp_foot = 30.0;
  double Kd_foot = 3.7;
  Eigen::Vector3d x_left = plant_.EvalBodyPoseInWorld(context_, left_foot)
                               .translation();  // [x, y, z] in world
  Eigen::Vector3d xd_left = J_left * state_q;
  // PD Controller for left foot
  Eigen::Vector3d x_left_nom(-0.0, 0.1, 0.3), xd_left_nom(0.0, 0.0, 0.0);
  // Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::MatrixXd Lambda_left_inv = (J_left * M_inverse * J_left.transpose());
  Eigen::Vector3d xdd_left_des =
      Lambda_left_inv *
      (Kp_foot * (x_left_nom - x_left) + Kd_foot * (xd_left_nom - xd_left));

  // Compute he desired acceleration for the right foot
  Eigen::Vector3d x_right = plant_.EvalBodyPoseInWorld(context_, right_foot)
                                .translation();  // [x, y, z] in world
  Eigen::Vector3d xd_right = J_right * state_q;
  // PD Controller for right foot
  Eigen::Vector3d x_right_nom(-0.0, 0.1, 0.3), xd_right_nom(0.0, 0.0, 0.0);
  // Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::MatrixXd Lambda_right_inv =
      (J_right * M_inverse * J_right.transpose());
  Eigen::Vector3d xdd_right_des =
      Lambda_right_inv *
      (Kp_foot * (x_right_nom - x_right) + Kd_foot * (xd_right_nom - xd_right));

  // Compute desired angular accelerations of the feet
  // Compute the desired rpy acceleration for the left foot
  Eigen::Vector3d rpy_left = GetRPYInWorld(left_foot);  // [roll, pitch, yaw]
  Eigen::Vector3d rpyd_left = J_rpyleft * state_q;
  Eigen::Vector3d rpy_left_nom(0.0, 0.0, 0.0), rpyd_left_nom(0.0, 0.0, 0.0);
  // Compute inverse of λ left rotational
  Eigen::MatrixXd Lambda_left_rpy_inv =
      (J_rpyleft * M_inverse * J_rpyleft.transpose());
  Eigen::Vector3d rpydd_left_des =
      Lambda_left_rpy_inv * (Kp_foot * (rpy_left_nom - rpy_left) +
                             Kd_foot * (rpyd_left_nom - rpyd_left));

  // Compute the desired rpy acceleration for the right foot
  Eigen::Vector3d rpy_right = GetRPYInWorld(right_foot);  // [roll, pitch, yaw]
  Eigen::Vector3d rpyd_right = J_rpyright * state_q;
  Eigen::Vector3d rpy_right_nom(0.0, 0.0, 0.0), rpyd_right_nom(0.0, 0.0, 0.0);
  // Compute inverse of λ right rotational
  Eigen::MatrixXd Lambda_right_rpy_inv =
      (J_rpyright * M_inverse * J_rpyright.transpose());
  Eigen::Vector3d rpydd_right_des =
      Lambda_right_rpy_inv * (Kp_foot * (rpy_right_nom - rpy_right) +
                              Kd_foot * (rpyd_right_nom - rpyd_right));

  // Compute desired torso angular acceleration
  double Kp_torso = 28.0;
  double Kd_torso = 3.7;
  Eigen::Vector3d rpy_torso = GetRPYInWorld(torso);  // [roll, pitch, yaw]
  Eigen::Vector3d rpyd_torso = J_torso * state_q;
  Eigen::Vector3d rpy_torso_nom(0.0, 0.0, 0.0), rpyd_torso_nom(0.0, 0.0, 0.0);
  // Compute inverse of λ torso rotational
  Eigen::MatrixXd Lambda_torso_rpy_inv =
      (J_torso * M_inverse * J_torso.transpose());
  Eigen::Vector3d rpydd_torso_des =
      Lambda_torso_rpy_inv * (Kp_torso * (rpy_torso_nom - rpy_torso) +
                              Kd_torso * (rpyd_torso_nom - rpyd_torso));

  // Compute energy shaping-based interface as a linear constraint
  // uV = tau_g - kappa*J'*(x_task-x2) - Kd*(qd-J'*u2) = A_int*u2 + b_int
  // Compute the midpoint between the left and right foot positions
  const auto& X_WR = plant_.EvalBodyPoseInWorld(context_, right_foot);
  const auto& X_WL = plant_.EvalBodyPoseInWorld(context_, left_foot);

  Eigen::Vector3d com_cmd = 0.5 * (X_WR.translation() + X_WL.translation()) +
                            Eigen::Vector3d(0.0, 0.0, 0.0);

  // set 0.5 meter in the z-direction
  com_cmd.z() += 0.5;
  // Get the current CoM position
  drake::Vector3<double> x_com =
      plant_.CalcCenterOfMassPositionInWorld(context_);
  // 1. Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_M(Mass_matrix);
  double lambda_M = eigen_solver_M.eigenvalues().maxCoeff();

  // 2. Compute λ_J_com (Largest eigenvalue of J_com^T * J_com)
  Eigen::MatrixXd JcomJcomt = J_com.transpose() * J_com;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_Jcom(JcomJcomt);
  double lambda_J_com = eigen_solver_Jcom.eigenvalues().maxCoeff();
  double alpha_com = 0.2;  // chosen conservatively
  double mass_total = plant_.CalcTotalMass(context_);

  // Compute κ based on the eigenvalues computed above
  double kappa_max_com =
      (2.0 * lambda_M) /
      (alpha_com * alpha_com * mass_total * mass_total * lambda_J_com);

  // Choose κ smaller than this limit
  double kappa_com = 0.9 * kappa_max_com;  // safety margin of 10%
  Eigen::MatrixXd KD_com = 0.9 * Mass_matrix;
  KD_com.diagonal().array() += 1e-6;  // Regularize
  Eigen::MatrixXd A_int = KD_com * J_com.transpose();

  Eigen::VectorXd b_int =
      tau_g - 2 * kappa_com * J_com.transpose() * (x_com - com_cmd) -
      KD_com * state_q;

  return Eigen::VectorXd::Zero(num_q_);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake