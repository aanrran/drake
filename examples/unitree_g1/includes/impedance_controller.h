// impedance_controller.h - Header File
#pragma once

#include <memory>
#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>

namespace drake {
namespace examples {
namespace unitree_g1 {
class ImpedanceController {
 public:
  explicit ImpedanceController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::Context<double>& context,
      Eigen::VectorX<double> stiffness,
      Eigen::VectorX<double> damping_ratio);

  Eigen::VectorXd CalcTorque(Eigen::VectorXd desired_position);

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::systems::Context<double>& context_;

  const Eigen::VectorX<double> stiffness_, damping_ratio_;

  // Angular momentum constraint matrices
  Eigen::MatrixXd A_am_;
  Eigen::VectorXd hdot_des_;

  // Contact information
  std::vector<drake::multibody::BodyIndex> contact_bodies_;
  std::vector<Eigen::Vector3d> contact_points_;

  // System matrices and vectors
  // Eigen::MatrixXd W_tau_, W_f_, S_, J_task_, J_dot_task_, A_friction_;
  // Eigen::VectorXd tau_ref_, f_ref_, tau_min_, tau_max_, b_friction_;

  int num_joints_;

  std::vector<Eigen::Vector3d> GetFootContactPoints() const;
  MatrixX<double> ComputeNullSpaceProjectionQR(const MatrixX<double>& J_c);
  MatrixX<double> ComputeContactJacobian(
    const drake::multibody::Frame<double>& foot_frame,
    const std::vector<Eigen::Vector3d>& contact_points);
};
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake