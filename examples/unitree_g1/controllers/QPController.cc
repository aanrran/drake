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
  solver_ = std::make_unique<drake::solvers::OsqpSolver>();
  // Define variables: joint torques (tau), accelerations (ddq)
  tau_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau");
  qdd_ = prog_.NewContinuousVariables(plant_.num_velocities(), "qdd");
  u2_ = prog_.NewContinuousVariables(3, "u2");
  tau0_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau0");
  JTfr_ = prog_.NewContinuousVariables(plant_.num_velocities(), "JTfr");
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

  int num_a = plant_.num_velocities();  // number of actuated DoFs

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
      Eigen::Vector3d::Zero(),  // Center to the foot frame
      plant_.world_frame(),     // Express in world frame
      plant_.world_frame(),     // Measured in world frame
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
          foot_frame,               // Measured frame
          Eigen::Vector3d::Zero(),  // Point of interest
          plant_.world_frame(),     // Expressed in frame
          plant_.world_frame()      // Measured in frame
      );
  JdotV_trans = spacial_bias.translational();
  JdotV_rot = spacial_bias.rotational();

  return std::make_pair(JdotV_trans, JdotV_rot);
}

Eigen::Vector3d QPController::GetRPYInWorld(
    const drake::multibody::Body<double>& body) const {
  const drake::math::RotationMatrix<double> R_WB =
      plant_.EvalBodyPoseInWorld(context_, body).rotation();
  return drake::math::RollPitchYaw<double>(R_WB).vector();
}

void QPController::AddJacobianTypeCost(const Eigen::MatrixXd& J,
                                       const Eigen::VectorXd& Jd_qd,
                                       const Eigen::VectorXd& xdd_des,
                                       double weight) {
  // Cost: weight * || J*qdd_ + Jd_qd - xdd_des ||²
  // Expand: 0.5 * qdd_' * Q * qdd_ + b' * qdd_ (ignore constant term)

  Eigen::MatrixXd Q = weight * J.transpose() * J;
  Eigen::VectorXd b = weight * J.transpose() * (Jd_qd - xdd_des);

  prog_.AddQuadraticCost(Q, b, qdd_);
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
  plant_.CalcMassMatrix(context_, &M);
  Eigen::LLT<Eigen::MatrixXd> llt(M);
  DRAKE_DEMAND(llt.info() == Eigen::Success);
  Eigen::MatrixXd M_inverse =
      llt.solve(Eigen::MatrixXd::Identity(M.rows(), M.cols()));

  // Compute gravity forces
  Eigen::VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);
  // Compute Coriolis and centrifugal forces
  VectorX<double> Cv(num_v);
  plant_.CalcBiasTerm(context_, &Cv);  // b
  // Extract the actuation matrix from the plant (B matrix).
  Eigen::MatrixXd Selection_matirx = Eigen::MatrixXd::Zero(num_v, num_a);
  Selection_matirx.bottomRightCorner(num_a, num_a) =
      Eigen::MatrixXd::Identity(num_a, num_a);
  // Center of mass Jacobian
  Eigen::MatrixXd J_com(3, num_v);
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_com);
  // Nullspace projection
  Eigen::MatrixXd N_com = ComputeNullSpaceProjection(J_com, M);

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
  int num_contacts = static_cast<int>(contact_points.size() * 2);
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

  Eigen::VectorXd tau_def_pose = u_stiffness.tail(num_v) + u_damping;

  // Compute desired linear accelerations of the feet
  // Compute he desired acceleration for the left foot
  double Kp_foot = 30.0;
  double Kd_foot = 3.7;
  Eigen::Vector3d x_left = plant_.EvalBodyPoseInWorld(context_, left_foot)
                               .translation();  // [x, y, z] in world
  Eigen::Vector3d xd_left = J_left * state_velocity;
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
  Eigen::Vector3d xd_right = J_right * state_velocity;
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
  Eigen::Vector3d rpyd_left = J_rpyleft * state_velocity;
  Eigen::Vector3d rpy_left_nom(0.0, 0.0, 0.0), rpyd_left_nom(0.0, 0.0, 0.0);
  // Compute inverse of λ left rotational
  Eigen::MatrixXd Lambda_left_rpy_inv =
      (J_rpyleft * M_inverse * J_rpyleft.transpose());
  Eigen::Vector3d rpydd_left_des =
      Lambda_left_rpy_inv * (Kp_foot * (rpy_left_nom - rpy_left) +
                             Kd_foot * (rpyd_left_nom - rpyd_left));

  // Compute the desired rpy acceleration for the right foot
  Eigen::Vector3d rpy_right = GetRPYInWorld(right_foot);  // [roll, pitch, yaw]
  Eigen::Vector3d rpyd_right = J_rpyright * state_velocity;
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
  Eigen::Vector3d rpyd_torso = J_torso * state_velocity;
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
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver_M(M);
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
  Eigen::MatrixXd KD_com = 0.9 * M;
  KD_com.diagonal().array() += 1e-6;  // Regularize
  Eigen::MatrixXd A_int = KD_com * J_com.transpose();

  Eigen::VectorXd b_int =
      tau_g - 2 * kappa_com * J_com.transpose() * (x_com - com_cmd) -
      KD_com * state_velocity;

  // Nominal u2 tracking cost
  const double w1 = 1e5;  // or whatever you tuned
  // Nominal u2 tracking cost
  Eigen::Vector3d u2_nom(0, 0, 0);
  prog_.AddQuadraticErrorCost(w1 * Eigen::MatrixXd::Identity(3, 3), u2_nom,
                              u2_);
  // Joint tracking cost
  const double w2 = 1e5;
  prog_.AddQuadraticErrorCost(w2 * Eigen::MatrixXd::Identity(num_v, num_v),
                              tau_def_pose, tau_);
  // Foot tracking costs: position and orientation for each foot
  const double w3 = 0;
  AddJacobianTypeCost(J_left, Jd_qd_left, xdd_left_des, w3);
  AddJacobianTypeCost(J_rpyleft, Jd_qd_rpyleft, rpydd_left_des, w3);
  AddJacobianTypeCost(J_right, Jd_qd_right, xdd_right_des, w3);
  AddJacobianTypeCost(J_rpyright, Jd_qd_rpyright, rpydd_right_des, w3);
  // torso orientation cost
  const double w4 = 0;
  AddJacobianTypeCost(J_torso, Jd_qd_torso, rpydd_torso_des, w4);

  // Contact acceleration constraints
  const double Kd_contact = 5.0;
  const double nu_min = -1e-4;
  const double nu_max = 1e-4;

  for (int i = 0; i < num_contacts; ++i) {
    Eigen::MatrixXd J = contact_jacobians.block(3 * i, 0, 3, num_v);
    Eigen::VectorXd Jd_qd = contact_jacobians_dot_v.segment(3 * i, 3);

    Eigen::VectorXd xd = J * state_velocity;
    Eigen::VectorXd xdd_des = -Kd_contact * xd;
    Eigen::VectorXd rhs = xdd_des - Jd_qd;

    Eigen::VectorXd lb = rhs.array() + nu_min;
    Eigen::VectorXd ub = rhs.array() + nu_max;

    prog_.AddLinearConstraint(J, lb, ub, qdd_);
  }
  // Dynamic constraints
  Eigen::MatrixXd A_dyn(M.rows(), 3 * num_v);  // [qdd, tau, JTfr]
  A_dyn << M, -Selection_matirx, -Eigen::MatrixXd::Identity(num_v, num_v);

  drake::solvers::VariableRefList vars_dyn = {
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(qdd_),
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(tau_),
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(JTfr_)};

  prog_.AddLinearEqualityConstraint(
      A_dyn, tau_g - Cv, drake::solvers::ConcatenateVariableRefList(vars_dyn));

  // Add interface constraint S'*tau + sum(J'*f_ext) = uV + N'*tau_0
  // Left-hand side (LHS): S^T * tau + JTfr
  Eigen::MatrixXd A_lhs = Selection_matirx;  // size: [num_v x num_v]
  Eigen::MatrixXd I_lhs = Eigen::MatrixXd::Identity(num_v, num_v);  // JTfr term

  // Right-hand side: A_int * u2 + N^T * tau0 + b_int
  // So move everything to the left side to make:
  // S^T * tau + JTfr - A_int * u2 - N^T * tau0 = b_int

  // Build constraint matrix: [tau, tau0, u2, JTfr]
  int dim = num_v + num_v + 3 + num_v;
  Eigen::MatrixXd A_iface = Eigen::MatrixXd::Zero(num_v, dim);

  // Fill columns
  A_iface.block(0, 0, num_v, num_v) = Selection_matirx;        // tau
  A_iface.block(0, num_v, num_v, num_v) = -N_com.transpose();  // tau0
  A_iface.block(0, 2 * num_v, num_v, 3) = -A_int;              // u2
  A_iface.block(0, 2 * num_v + 3, num_v, num_v) = I_lhs;       // JTfr

  // Build variable list
  drake::solvers::VariableRefList vars_iface = {
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(tau_),
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(tau0_),
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(u2_),
      Eigen::Ref<const drake::solvers::VectorXDecisionVariable>(JTfr_),
  };

  prog_.AddLinearEqualityConstraint(
      A_iface, b_int, drake::solvers::ConcatenateVariableRefList(vars_iface));

  return Eigen::VectorXd::Zero(plant_.num_velocities());
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake