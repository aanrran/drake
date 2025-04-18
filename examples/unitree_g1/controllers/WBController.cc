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
  //   tau_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau");
  //   qdd_ = prog_.NewContinuousVariables(plant_.num_velocities(), "qdd");
  //   u2_ = prog_.NewContinuousVariables(3, "u2");
  //   tau0_ = prog_.NewContinuousVariables(plant_.num_velocities(), "tau0");
  //   JTfr_ = prog_.NewContinuousVariables(plant_.num_velocities(), "JTfr");
}

// Shared helper function for task-space PD computation with velocity-projected
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd>
WBController::ComputeTaskSpaceAccel(
    const Eigen::MatrixXd& J_task, const Eigen::VectorXd& x_task,
    const Eigen::VectorXd& x_cmd, const Eigen::VectorXd& xd_cmd,
    const Eigen::VectorXd& Jd_qd_task, const Eigen::VectorXd& state_qd,
    const Eigen::VectorXd& state_qdd, const Eigen::MatrixXd& M_inverse,
    const Eigen::MatrixXd& N_pre, const double& Kp_task,
    const double& Kd_task) {
  // Compute the pseudo-inverse of the Jacobian
  Eigen::MatrixXd J_dyn_pinv =
      ComputeJacobianPseudoInv(J_task, M_inverse, N_pre);

  // Compute null-space projection in actuator subspace
  Eigen::MatrixXd N_task = Ivv_ - J_dyn_pinv * J_task;
  // Compute the desired acceleration for the task
  Eigen::VectorXd xd_task = J_task * state_qd;
  // PD Controller
  // 1. Compute Lambda_inv
  Eigen::MatrixXd Lambda_inv = J_task * M_inverse * J_task.transpose();
  Lambda_inv.diagonal().array() += 1e-6;  // regularization

  // 2. Approximate Lambda using diagonal inverse
  Eigen::VectorXd Lambda_diag_inv = Lambda_inv.diagonal();
  Eigen::VectorXd Lambda_diag = Lambda_diag_inv.cwiseInverse();

  // 3. Fast critical damping: D_task = 2 * sqrt(Lambda * Kp)
  Eigen::VectorXd D_diag =
      2.0 * Lambda_diag.array().sqrt() * std::sqrt(Kp_task);

  // 4. Task-space desired acceleration
  Eigen::VectorXd xdd_des =
      Lambda_inv *
      (Kp_task * (x_cmd - x_task) + D_diag.asDiagonal() * (xd_cmd - xd_task));
  // Comput the task-space Velocity
  Eigen::VectorXd qd_task = N_task * state_qd;

  // Compute desired acceleration
  Eigen::VectorXd qdd_task =
      N_task * state_qdd + J_dyn_pinv * (xdd_des - Jd_qd_task);

  return std::make_tuple(qd_task, qdd_task, N_pre * N_task);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd>
WBController::NspaceCoMctrl(const double& Kp_task, const double& Kd_task,
                            const Eigen::MatrixXd& N_pre,
                            const Eigen::VectorXd x_cmd,
                            const Eigen::VectorXd xd_cmd,
                            const Eigen::VectorXd& state_qd,
                            const Eigen::VectorXd& state_qdd,
                            const Eigen::MatrixXd& M_inverse) {
  // Get the current CoM position
  drake::Vector3<double> x_task =
      plant_.CalcCenterOfMassPositionInWorld(context_);
  // Resize Jacobian explicitly
  Eigen::MatrixXd J_task(3, num_q_);
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context_, drake::multibody::JacobianWrtVariable::kV, plant_.world_frame(),
      plant_.world_frame(), &J_task);

  // Compute the bias acceleration (Coriolis and centrifugal effects)
  const drake::multibody::SpatialAcceleration<double> com_spacial_bias =
      plant_.CalcBiasSpatialAcceleration(
          context_,
          drake::multibody::JacobianWrtVariable::kV,  // ✅ Use kV
          plant_.world_frame(),                       // Measured frame
          x_task,                // Point of interest (CoM position)
          plant_.world_frame(),  // Expressed in world frame
          plant_.world_frame()   // Measured relative to world
      );
  const auto Jd_qd_task = com_spacial_bias.translational();

  return ComputeTaskSpaceAccel(J_task, x_task, x_cmd, xd_cmd, Jd_qd_task,
                               state_qd, state_qdd, M_inverse, N_pre, Kp_task,
                               Kd_task);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd>
WBController::NspaceContactrl(
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
    DRAKE_DEMAND(x_trans.size() == 3 && x_rpy.size() == 3);
    DRAKE_DEMAND(J_trans.rows() == 3 && J_rpy.rows() == 3);
    DRAKE_DEMAND(J_trans.cols() == num_q_ && J_rpy.cols() == num_q_);

    x_task.resize(6);
    x_task.head(3) = x_trans;
    x_task.tail(3) = x_rpy;

    J_task.resize(6, num_q_);
    J_task.topRows(3) = J_trans;
    J_task.bottomRows(3) = J_rpy;

    Jd_qd_task.resize(6, num_q_);
    Jd_qd_task.topRows(3) = Jd_qd_trans;
    Jd_qd_task.bottomRows(3) = Jd_qd_rpy;
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

  // === Foot Command Calculation with Directional Velocity Projection ===
  // Define foot position goal
  Eigen::Vector3d x_goal = x_cmd.head(3);
  // Compute directional projection
  Eigen::Vector3d x_delta = x_goal - x_trans;
  double x_delta_norm = x_delta.norm();
  Eigen::Vector3d x_cmd_trans;

  if (x_delta_norm > 1e-4) {
    Eigen::Vector3d x_delta_hat = x_delta / x_delta_norm;

    // Project velocity onto direction to goal
    double v_along_goal = x_delta_hat.dot(xd_task);
    Eigen::Vector3d v_proj = v_along_goal * x_delta_hat;

    // Bias command forward using velocity projection
    double alpha = 0.05;  // scaling factor
    x_cmd_trans = x_trans + alpha * v_proj;

    // Optional: Clip near goal to avoid oscillation
    if ((x_cmd_trans - x_goal).norm() < 0.002) {
      x_cmd_trans = x_goal;
    }
  } else {
    // Close enough to goal, stop moving
    x_cmd_trans = x_goal;
  }
  Eigen::VectorXd x_cmd_full(6);
  x_cmd_full.head(3) = x_cmd_trans;
  x_cmd_full.tail(3) = x_cmd.tail(3);
  return ComputeTaskSpaceAccel(J_task, x_task, x_cmd_full, xd_cmd, Jd_qd_task,
                               state_qd, state_qdd, M_inverse, N_pre, Kp_task,
                               Kd_task);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd>
WBController::NspacePDctrl(
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
    DRAKE_DEMAND(x_trans.size() == 3 && x_rpy.size() == 3);
    DRAKE_DEMAND(J_trans.rows() == 3 && J_rpy.rows() == 3);
    DRAKE_DEMAND(J_trans.cols() == num_q_ && J_rpy.cols() == num_q_);

    x_task.resize(6);
    x_task.head(3) = x_trans;
    x_task.tail(3) = x_rpy;

    J_task.resize(6, num_q_);
    J_task.topRows(3) = J_trans;
    J_task.bottomRows(3) = J_rpy;

    Jd_qd_task.resize(6, num_q_);
    Jd_qd_task.topRows(3) = Jd_qd_trans;
    Jd_qd_task.bottomRows(3) = Jd_qd_rpy;
  } else {
    throw std::runtime_error("Invalid task type");
  }

  return ComputeTaskSpaceAccel(J_task, x_task, x_cmd, xd_cmd, Jd_qd_task,
                               state_qd, state_qdd, M_inverse, N_pre, Kp_task,
                               Kd_task);
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
  // Compute the state speed
  Eigen::VectorXd state_qdd =
      plant_.get_generalized_acceleration_output_port().Eval(context_);
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
  // Nullspace projection
  Eigen::LLT<Eigen::MatrixXd> llt(Mass_matrix);
  DRAKE_DEMAND(llt.info() == Eigen::Success);
  Eigen::MatrixXd M_inverse = llt.solve(Ivv_);

  // Compute the desired acceleration for supporting foot task
  const auto& left_foot = plant_.GetBodyByName("left_ankle_roll_link");

  double Kp_left_foot = 50.0;
  double Kd_left_foot = 0.7;
  Eigen::VectorXd x_cmd_left_foot(6);
  x_cmd_left_foot << 0.0, 0.0, 0.04, 0.0, 0.0, 0.0;
  Eigen::VectorXd xd_cmd_left_foot(6);
  xd_cmd_left_foot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto [qd_left_foot, qdd_left_foot, N_left_foot] = NspacePDctrl(
      Kp_left_foot, Kd_left_foot, Ivv_, x_cmd_left_foot, xd_cmd_left_foot,
      state_qd, state_qdd, left_foot, M_inverse, "both");

  // Compute desired CoM acceleration
  double Kp_com = 50.0;
  double Kd_com = 0.7;
  Eigen::VectorXd x_cmd_com(3);
  x_cmd_com << 0.0, 0.05, 0.5;
  Eigen::VectorXd xd_cmd_com(3);
  xd_cmd_com << 0.0, 0.0, 0.0;
  auto [qd_com, qdd_com, N_com] =
      NspaceCoMctrl(Kp_com, Kd_com, N_left_foot, x_cmd_com, xd_cmd_com,
                    qd_left_foot, qdd_left_foot, M_inverse);

  // Compute desired torso acceleration
  const auto& torso = plant_.GetBodyByName("torso_link");

  double Kp_torso = 50.0;
  double Kd_torso = 0.7;
  Eigen::VectorXd x_cmd_torso(3);
  x_cmd_torso << 0.0, 0.0, 0.0;
  Eigen::VectorXd xd_cmd_torso(3);
  xd_cmd_torso << 0.0, 0.0, 0.0;

  auto [qd_torso, qdd_torso, N_torso] =
      NspacePDctrl(Kp_torso, Kd_torso, N_com, x_cmd_torso, xd_cmd_torso, qd_com,
                   qdd_com, torso, M_inverse, "rpy");

  // Compute desired right leg acceleration
  const auto& right_foot = plant_.GetBodyByName("right_ankle_roll_link");

  double Kp_right_foot = 50.0;
  double Kd_right_foot = 0.7;
  Eigen::VectorXd x_cmd_right_foot(6);
  x_cmd_right_foot << 0.0, -0.1, 0.1, 0.0, 0.0, 0.0;
  Eigen::VectorXd xd_cmd_right_foot(6);
  xd_cmd_right_foot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto [qd_right_foot, qdd_right_foot, N_right_foot] = NspacePDctrl(
      Kp_right_foot, Kd_right_foot, N_torso, x_cmd_right_foot,
      xd_cmd_right_foot, qd_torso, qdd_torso, right_foot, M_inverse, "both");

  // Comput the default pose acceleration
  double Kp_pose = 10.5;
  double Kd_pose = 0.7;
  Eigen::MatrixXd J_pose = Selection_matirx;
  Eigen::VectorXd x_pose = state_pos.tail(num_q_);
  Eigen::VectorXd x_pose_cmd = desired_position.head(num_q_);
  Eigen::VectorXd xd_pose_cmd = Eigen::VectorXd::Zero(num_q_);
  Eigen::VectorXd Jd_qd_pose = Eigen::VectorXd::Zero(num_q_);
  Eigen::MatrixXd N_pose_pre = Ivv_;
  auto [qd_pose, qdd_pose, N_pose] = ComputeTaskSpaceAccel(
      J_pose, x_pose, x_pose_cmd, xd_pose_cmd, Jd_qd_pose, state_qd, state_qdd,
      M_inverse, N_pose_pre, Kp_pose, Kd_pose);
  //   return Mass_matrix * qdd_right_foot + Cv - tau_g +
  //          (N_right_foot).transpose() * tau_def_pose;
  //   return Mass_matrix * qdd_com + Cv - tau_g +
  //          (N_com).transpose() * tau_def_pose;
  return Mass_matrix * qdd_pose;
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake