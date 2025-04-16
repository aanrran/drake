// WBController.h - Header File
#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/systems/framework/context.h>

namespace drake {
namespace examples {
namespace unitree_g1 {
class WBController {
 public:
  explicit WBController(const drake::multibody::MultibodyPlant<double>& plant,
                        const drake::systems::Context<double>& context,
                        Eigen::VectorX<double> stiffness,
                        Eigen::VectorX<double> damping_ratio);

  Eigen::VectorXd CalcTorque(Eigen::VectorXd desired_position,
                             Eigen::VectorXd tau_sensor);

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::systems::Context<double>& context_;

  const Eigen::VectorX<double> stiffness_, damping_ratio_;

  Eigen::MatrixXd Ivv_, Iaa_;
  int num_q_, num_a_, num_pos_;

  // QP solver
  std::unique_ptr<drake::solvers::OsqpSolver> solver_;
  drake::solvers::MathematicalProgram prog_;
  drake::solvers::VectorXDecisionVariable tau_, qdd_, u2_, tau0_, JTfr_;

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> GetBodyJacobian(
      const drake::multibody::Frame<double>& foot_frame);

  std::pair<Eigen::VectorXd, Eigen::VectorXd> GetBodyBias(
      const drake::multibody::Frame<double>& foot_frame);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> GetPosInWorld(
      const drake::multibody::Body<double>& body) const;

  Eigen::MatrixXd ComputeJacobianPseudoInv(const Eigen::MatrixXd& Jacobian,
                                           const Eigen::MatrixXd& M_inverse,
                                           const Eigen::MatrixXd& N_pre);
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> NspacePDctrl(
      const double& Kp_task, const double& Kd_task,
      const Eigen::MatrixXd& N_pre, const Eigen::VectorXd x_cmd,
      const Eigen::VectorXd xd_cmd, const Eigen::VectorXd& state_qd,
      const Eigen::VectorXd& state_qdd,
      const drake::multibody::Body<double>& task_body,
      const Eigen::MatrixXd& M_inverse, const std::string& task_type);
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake