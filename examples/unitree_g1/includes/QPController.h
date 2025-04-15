// QPController.h - Header File
#pragma once

#include <memory>

#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/systems/framework/context.h>

namespace drake {
namespace examples {
namespace unitree_g1 {
class QPController {
 public:
  explicit QPController(const drake::multibody::MultibodyPlant<double>& plant,
                        const drake::systems::Context<double>& context,
                        Eigen::VectorX<double> stiffness,
                        Eigen::VectorX<double> damping_ratio);

  Eigen::VectorXd CalcTorque(Eigen::VectorXd desired_position,
                             Eigen::VectorXd tau_sensor);

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::systems::Context<double>& context_;

  const Eigen::VectorX<double> stiffness_, damping_ratio_;
  // QP solver
  std::unique_ptr<drake::solvers::OsqpSolver> solver_;
  drake::solvers::MathematicalProgram prog_;
  drake::solvers::VectorXDecisionVariable tau_, qdd_, u2_, tau0_, JTfr_;

  std::vector<Eigen::Vector3d> GetFootContactPoints() const;
  MatrixX<double> ComputeNullSpaceProjection(
      const MatrixX<double>& J_c, const MatrixX<double>& Mass_matrix);

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> GetBodyJacobian(
      const drake::multibody::Frame<double>& foot_frame);

  std::pair<Eigen::VectorXd, Eigen::VectorXd> GetBodyBias(
      const drake::multibody::Frame<double>& foot_frame);

  std::pair<Eigen::MatrixXd, Eigen::VectorXd> ContactJacobianAndBias(
      const drake::multibody::Frame<double>& foot_frame,
      const std::vector<Eigen::Vector3d>& contact_points);

  MatrixX<double> ComputeJacobianPseudoInverse(const MatrixX<double>& J,
                                               double damping_eps = 1e-6);

  Eigen::Vector3d GetRPYInWorld(
      const drake::multibody::Body<double>& body) const;

  void AddJacobianTypeCost(const Eigen::MatrixXd& J,
                           const Eigen::VectorXd& Jd_qd,
                           const Eigen::VectorXd& xdd_des, double weight);
};
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake