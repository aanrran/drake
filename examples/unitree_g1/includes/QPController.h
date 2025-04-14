// QPController.h - Header File
#pragma once

#include <memory>

#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
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

  // Contact information
  std::vector<drake::multibody::BodyIndex> contact_bodies_;
  std::vector<Eigen::Vector3d> contact_points_;

  // System matrices and vectors

  int num_joints_;

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
};
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake