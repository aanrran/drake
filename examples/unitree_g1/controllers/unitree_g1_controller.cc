#include "examples/unitree_g1/includes/unitree_g1_controller.h"

#include "examples/unitree_g1/includes/timing_logger.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

template <typename T>
UnitreeG1Controller<T>::UnitreeG1Controller(const MultibodyPlant<T>& plant)
    : plant_(plant) {
  // Number of generalized coordinates (q) and velocities (v) in the system
  const int num_q = plant_.num_positions();
  const int num_v = plant_.num_velocities();
  const int num_x = num_q + num_v;
  // Declare input port for receiving the full robot state (positions +
  // velocities)
  this->DeclareInputPort(systems::kUseDefaultName,
                         drake::systems::kVectorValued, num_x);
  // Declare output port for computing and sending torque commands
  this->DeclareVectorOutputPort("torque_output", num_v,
                                &UnitreeG1Controller<T>::CalcTorque,
                                {this->all_input_ports_ticket()});
  // Create a default context for the plant model
  plant_context_ = plant_.CreateDefaultContext();
  // Initialize damping gains for passive joint damping (can be tuned for better
  // performance)
  damping_gains_ = Eigen::ArrayXd::Constant(num_v, 0.1);
}

template <typename T>
void UnitreeG1Controller<T>::CalcTorque(const Context<T>& context,
                                        BasicVector<T>* torque) const {
  TimingLogger::GetInstance().StartTimer("RunController"); // timer started
  // Ensure the plant context is initialized before use
  DRAKE_DEMAND(plant_context_ != nullptr);
  // Retrieve the state input (q, v) from the input port
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input != nullptr);
  const Eigen::VectorXd& x = input->value();  // Extract full state vector
  // Update plant's internal state representation (positions + velocities)
  plant_.SetPositionsAndVelocities(plant_context_.get(), x);
  // Extract velocity portion (v) of the state
  Eigen::VectorXd v = x.tail(plant_.num_velocities());
  // Compute damping torque: τ = -D * v, where D is a diagonal damping matrix
  torque->get_mutable_value() = -(damping_gains_ * v.array()).matrix();
  TimingLogger::GetInstance().StopTimer("RunController"); // timer stopped
}

// Explicit template instantiation for double-precision floating point
// operations
template class UnitreeG1Controller<double>;

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
