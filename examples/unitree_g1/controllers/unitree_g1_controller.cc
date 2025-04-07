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
  this->DeclareInputPort(systems::kUseDefaultName, drake::systems::kVectorValued, num_x);
  // Declare output port for computing and sending torque commands
  this->DeclareVectorOutputPort("torque_output", num_v,
                                &UnitreeG1Controller<T>::CalcTorque,
                                {this->all_input_ports_ticket()});
  // Create a default context for the plant model
  plant_context_ = plant_.CreateDefaultContext();
  // Initialize the Impedance Controller
  Eigen::VectorXd stiffness = Eigen::VectorXd::Constant(num_q, 10.5);
  Eigen::VectorXd damping_ratio = Eigen::VectorXd::Constant(num_v, 0.7);
  my_controller_ = std::make_unique<ImpedanceController>(plant_, *plant_context_, stiffness, damping_ratio);

  desired_position_ = Eigen::VectorXd::Zero(num_q);
  desired_position_ << 
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      -3.0,               // left_hip_pitch_joint
      0.0,               // left_hip_roll_joint
      0.0,               // left_hip_yaw_joint
      3.75,               // left_knee_joint
      -1.25,               // left_ankle_pitch_joint
      0.0,               // left_ankle_roll_joint
      -2.0,               // right_hip_pitch_joint
      0.0,               // right_hip_roll_joint
      0.0,               // right_hip_yaw_joint
      0.5,               // right_knee_joint
      -0.25,               // right_ankle_pitch_joint
      0.0,               // right_ankle_roll_joint
      0.0,               // waist_yaw_joint
      0.0,               // left_shoulder_pitch_joint
      0.0,               // left_shoulder_roll_joint
      0.0,               // left_shoulder_yaw_joint
      0.0,               // left_elbow_joint
      0.0,               // left_wrist_roll_joint
      0.0,               // right_shoulder_pitch_joint
      0.0,               // right_shoulder_roll_joint
      0.0,               // right_shoulder_yaw_joint
      0.0,               // right_elbow_joint
      0.0;               // right_wrist_roll_joint
}

template <typename T>
void UnitreeG1Controller<T>::CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
  TimingLogger::GetInstance().StartTimer("RunController"); // timer started
  // Ensure the plant context is initialized before use
  DRAKE_DEMAND(plant_context_ != nullptr);
  // Retrieve the state input (q, v) from the input port
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input != nullptr);
  const Eigen::VectorXd& x = input->value();  // Extract full state vector
  // Update plant's internal state representation (positions + velocities)
  plant_.SetPositionsAndVelocities(plant_context_.get(), x);
  // Compute damping torque: τ = -D * v, where D is a diagonal damping matrix
  torque->get_mutable_value() = my_controller_->CalcTorque(desired_position_);
  TimingLogger::GetInstance().StopTimer("RunController"); // timer stopped
}

// Explicit template instantiation for double-precision floating point
// operations
template class UnitreeG1Controller<double>;

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
