#include "examples/unitree_g1/includes/unitree_g1_controller.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

template <typename T>
UnitreeG1Controller<T>::UnitreeG1Controller(const MultibodyPlant<T>& plant)
    : plant_(plant) {
  const int num_q = plant_.num_positions();
  const int num_v = plant_.num_velocities();
  const int num_x = num_q + num_v;

  this->DeclareInputPort(systems::kUseDefaultName, drake::systems::kVectorValued, num_x);
  this->DeclareVectorOutputPort("torque_output", num_v,
                                &UnitreeG1Controller<T>::CalcTorque,
                                {this->all_input_ports_ticket()});

  plant_context_ = plant_.CreateDefaultContext();
  damping_gains_ = Eigen::ArrayXd::Constant(num_v, 0.1);
}

template <typename T>
void UnitreeG1Controller<T>::CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
  DRAKE_DEMAND(plant_context_ != nullptr);

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input != nullptr);
  const Eigen::VectorXd& x = input->value();

  plant_.SetPositionsAndVelocities(plant_context_.get(), x);

  Eigen::VectorXd v = x.tail(plant_.num_velocities());
  torque->get_mutable_value() = -(damping_gains_ * v.array()).matrix();
}

// Explicit template instantiation
template class UnitreeG1Controller<double>;

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
