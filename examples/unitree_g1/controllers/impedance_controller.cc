// impedance_controller.cc - Source File
#include "examples/unitree_g1/includes/impedance_controller.h"

#include <iostream>

#include <drake/common/eigen_types.h>

namespace drake {
namespace examples {
namespace unitree_g1 {
ImpedanceController::ImpedanceController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    Eigen::VectorX<double> stiffness,
    Eigen::VectorX<double> damping_ratio)
    : plant_(plant),
      context_(context),
      stiffness_(stiffness),
      damping_ratio_(damping_ratio) {
  contact_points_ = GetFootContactPoints();
}

std::vector<Eigen::Vector3d> ImpedanceController::GetFootContactPoints() const {
  return {
      Eigen::Vector3d(-0.07, 0.08, -0.1), Eigen::Vector3d(-0.07, -0.08, -0.1),
      Eigen::Vector3d(0.20, -0.08, -0.1), Eigen::Vector3d(0.20, 0.08, -0.1)};
}

Eigen::VectorXd ImpedanceController::CalcTorque(Eigen::VectorXd desired_position) {

    
  // **Compute stiffness torque**
  const drake::VectorX<double> state_position = plant_.GetPositions(context_);
  Eigen::VectorXd position_error = state_position - desired_position;
  Eigen::VectorXd u_stiffness = (stiffness_.array() * position_error.array()).matrix();
  // Compute damping torque
  const drake::VectorX<double> state_velocity = plant_.GetVelocities(context_);
  const int num_v = plant_.num_velocities();
  // Compute mass matrix H
  Eigen::MatrixXd H(num_v, num_v);
  plant_.CalcMassMatrixViaInverseDynamics(context_, &H);
  // Compute critical damping gains and scale by damping ratio. Use Eigen
  // arrays (rather than matrices) for elementwise multiplication.
  Eigen::ArrayXd temp = H.diagonal().array() * stiffness_.array();
  Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
  damping_gains *= damping_ratio_.array();

  // Compute damping torque.
  Eigen::VectorXd u_damping = -(damping_gains * state_velocity.array()).matrix();
  return u_stiffness + u_damping;
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake