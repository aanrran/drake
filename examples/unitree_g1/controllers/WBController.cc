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
using std::string;

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

// Compute desired linear accelerations
std::pair<Eigen::VectorXd, Eigen::MatrixXd> WBController::NspacePDctrl(
    const double& Kp_task, const double& Kd_task, const Eigen::MatrixXd& N_pre,
    const Eigen::VectorXd x_cmd, const Eigen::VectorXd xd_cmd,
    const Eigen::VectorXd& state_qd, const Eigen::VectorXd& state_qdd,
    const drake::multibody::Body<double>& task_body,
    const Eigen::MatrixXd& M_inverse, const std::string& task_type) {
  // Compute the Jacobian
  auto [J_trans, J_rpy] = GetBodyJacobian(task_body.body_frame());
  // Compute the bias acceleration for the right foot
  auto [Jd_qd_trans, Jd_qd_rpy] = GetBodyBias(task_body.body_frame());
  // Compute the position of the task body in world frame
  auto [x_trans, x_rpy] = GetPosInWorld(task_body);
  // Compute the positions and Jacobians based on task type
  Eigen::VectorXd x_task;
  Eigen::MatrixXd J_task, Jd_qd_task;
  if (task_type == "trans") {
    x_task = x_trans;
    J_task = J_trans;
    Jd_qd_task = Jd_qd_trans;
  } else if (task_type == "rpy") {
    x_task = x_rpy;
    J_task = J_rpy;
    Jd_qd_task = Jd_qd_rpy;
  } else if (task_type == "both") {
    x_task.resize(6);
    x_task << x_trans, x_rpy;

    J_task.resize(6, num_q_);
    J_task << J_trans, J_rpy;

    Jd_qd_task.resize(6, num_q_);
    Jd_qd_task << Jd_qd_trans, Jd_qd_rpy;
  } else {
    throw std::runtime_error("Invalid task type");
  }

  // Compute the pseudo-inverse of the Jacobian
  Eigen::MatrixXd J_dyn_pinv =
      ComputeJacobianPseudoInv(J_task, M_inverse, N_pre);

  // Compute null-space projection in actuator subspace
  Eigen::MatrixXd N_task = Ivv_ - J_dyn_pinv * J_task;
  // Compute the desired acceleration for the task
  Eigen::VectorXd xd_task = J_task * state_qd;
  // PD Controller
  // Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  Eigen::MatrixXd Lambda_inv = (J_task * M_inverse * J_task.transpose());

  Eigen::VectorXd xdd_des =
      Lambda_inv * (Kp_task * (x_cmd - x_task) + Kd_task * (xd_cmd - xd_task));

  // Compute desired acceleration
  Eigen::VectorXd accel_task =
      state_qdd + J_dyn_pinv * (xdd_des - Jd_qd_task - J_task * state_qdd);

  return std::make_pair(accel_task, N_task);
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

std::pair<Eigen::Vector3d, Eigen::Vector3d> WBController::GetPosInWorld(
    const drake::multibody::Body<double>& body) const {
  const auto& body_pose = plant_.EvalBodyPoseInWorld(context_, body);
  Eigen::Vector3d x_trans = body_pose.translation();

  const drake::math::RotationMatrix<double> R_WB = body_pose.rotation();
  Eigen::Vector3d x_rpy = drake::math::RollPitchYaw<double>(R_WB).vector();
  return std::make_pair(x_trans, x_rpy);
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
  const drake::VectorX<double> state_qd = plant_.GetVelocities(context_);
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
  Eigen::VectorXd u_damping = -(damping_gains * state_qd.array()).matrix();

  Eigen::VectorXd tau_def_pose = u_stiffness.tail(num_q_) + u_damping;

  // Compute desired torso angular acceleration
  double Kp_left_foot = 28.0;
  double Kd_left_foot = 3.7;
  Eigen::VectorXd x_cmd_left_foot(6);
  x_cmd_left_foot << -0.0, 0.1, 0.3, 0.0, 0.0, 0.0;
  Eigen::VectorXd xd_cmd_left_foot(6);
  xd_cmd_left_foot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ;
  string left_foot_ctrl_type = "both";
  Eigen::VectorXd state_qdd = Eigen::VectorXd::Zero(num_q_);
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");
  auto [accel_left_foot, N_left_foot] =
      NspacePDctrl(Kp_left_foot, Kd_left_foot, Selection_matirx,
                   x_cmd_left_foot, xd_cmd_left_foot, state_qd, state_qdd,
                   left_foot, M_inverse, left_foot_ctrl_type);

  //   // Compute energy shaping-based interface as a linear constraint
  //   // uV = tau_g - kappa*J'*(x_task-x2) - Kd*(qd-J'*u2) = A_int*u2 + b_int
  //   // Compute the midpoint between the left and right foot positions
  //   const auto& X_WR = plant_.EvalBodyPoseInWorld(context_, right_foot);
  //   const auto& X_WL = plant_.EvalBodyPoseInWorld(context_, left_foot);

  //   Eigen::Vector3d com_cmd = 0.5 * (X_WR.translation() + X_WL.translation())
  //   +
  //                             Eigen::Vector3d(0.0, 0.0, 0.0);

  //   // set 0.5 meter in the z-direction
  //   com_cmd.z() += 0.5;
  //   // Get the current CoM position
  //   drake::Vector3<double> x_com =
  //       plant_.CalcCenterOfMassPositionInWorld(context_);
  //   // 1. Compute λ_M (Largest eigenvalue of Mass matrix M(q))
  //   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>
  //   eigen_solver_M(Mass_matrix); double lambda_M =
  //   eigen_solver_M.eigenvalues().maxCoeff();

  //   // 2. Compute λ_J_com (Largest eigenvalue of J_com^T * J_com)
  //   Eigen::MatrixXd JcomJcomt = J_com.transpose() * J_com;
  //   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>
  //   eigen_solver_Jcom(JcomJcomt); double lambda_J_com =
  //   eigen_solver_Jcom.eigenvalues().maxCoeff(); double alpha_com = 0.2;  //
  //   chosen conservatively double mass_total = plant_.CalcTotalMass(context_);

  //   // Compute κ based on the eigenvalues computed above
  //   double kappa_max_com =
  //       (2.0 * lambda_M) /
  //       (alpha_com * alpha_com * mass_total * mass_total * lambda_J_com);

  //   // Choose κ smaller than this limit
  //   double kappa_com = 0.9 * kappa_max_com;  // safety margin of 10%
  //   Eigen::MatrixXd KD_com = 0.9 * Mass_matrix;
  //   KD_com.diagonal().array() += 1e-6;  // Regularize
  //   Eigen::MatrixXd A_int = KD_com * J_com.transpose();

  //   Eigen::VectorXd b_int =
  //       tau_g - 2 * kappa_com * J_com.transpose() * (x_com - com_cmd) -
  //       KD_com * state_qd;

  return Eigen::VectorXd::Zero(num_q_);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake