#include "./../include/unitree_g1_controller.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

UnitreeG1Controller::UnitreeG1Controller(const multibody::MultibodyPlant<double>* plant)
    : ControllerBase(plant) {
  // Declare input port for the robot's full state
  this->DeclareInputPort("state", systems::kVectorValued, plant->num_multibody_states());

  // Declare output port for torque commands
  this->DeclareVectorOutputPort(
      "torques", plant->num_actuated_dofs(),
      &UnitreeG1Controller::CalcTorques);
}

// ComputeControl remains unchanged, but is now used inside CalcTorques
void UnitreeG1Controller::ComputeControl(
    const systems::Context<double>& context,
    Eigen::VectorXd* torques) const {
  *torques = Eigen::VectorXd::Zero(plant_->num_actuated_dofs());
}

// Compute torques based on system state
void UnitreeG1Controller::CalcTorques(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  Eigen::VectorXd torques;
  ComputeControl(context, &torques);
  output->set_value(torques);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
