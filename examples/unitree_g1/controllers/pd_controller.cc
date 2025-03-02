#include "examples/unitree_g1/includes/pd_controller.h"

#include <memory>
#include <iostream>
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace {

using drake::systems::Adder;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::kVectorValued;
using drake::systems::LeafSystem;
using drake::systems::controllers::PidController;

template <typename T>
class StateDependentDamper : public LeafSystem<T> {
 public:
  StateDependentDamper(const multibody::MultibodyPlant<T>& plant, const VectorX<double>& stiffness, const VectorX<double>& damping_ratio)
      : plant_(plant), stiffness_(stiffness), damping_ratio_(damping_ratio) {
    const int num_q = plant_.num_actuated_dofs();
    const int num_v = plant_.num_actuated_dofs();
    const int num_x = num_q + num_v;
    
    DRAKE_DEMAND(stiffness.size() == num_v);
    DRAKE_DEMAND(damping_ratio.size() == num_v);

    this->DeclareInputPort(systems::kUseDefaultName, kVectorValued, num_x);
    this->DeclareVectorOutputPort(systems::kUseDefaultName, num_v, &StateDependentDamper<T>::CalcTorque);
    // Make context with default parameters.
    plant_context_ = plant_.CreateDefaultContext();
    
  }

 private:
  const multibody::MultibodyPlant<T>& plant_;
  const VectorX<double> stiffness_;
  const VectorX<double> damping_ratio_;

  // This context is used solely for setting generalized positions and
  // velocities in multibody_plant_.
  std::unique_ptr<Context<T>> plant_context_;

  /**
   * Computes joint level damping forces by computing the damping ratio for each
   * joint independently as if all other joints were fixed. Note that the
   * effective inertia of a joint, when all of the other joints are fixed, is
   * given by the corresponding diagonal entry of the mass matrix. The critical
   * damping gain for the i-th joint is given by 2*sqrt(M(i,i)*stiffness(i)).
   */
  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
    
// ✅ Retrieve input state vector (actuated joints only)
const Eigen::VectorXd& x = this->EvalVectorInput(context, 0)->value();

// ✅ Create a full state vector (59 elements) initialized to zero
Eigen::VectorXd full_state = Eigen::VectorXd::Zero(plant_.num_positions() + plant_.num_velocities());

// ✅ Ensure the quaternion is valid (set it to identity)
full_state.segment<4>(3) << 1.0, 0.0, 0.0, 0.0;  // Identity quaternion (w, x, y, z)

// ✅ Copy actuated joint states (last 23 elements) into `full_state`
full_state.tail(2 * plant_.num_actuated_dofs()) = x;  // Fix: `x` was missing

// ✅ Set full state in plant context
plant_.SetPositionsAndVelocities(plant_context_.get(), full_state);

    
    
    const int num_v_full = plant_.num_velocities();  // Includes floating base
    const int num_v_actuated = plant_.num_actuated_dofs();  // Only actuated DOFs
    
    Eigen::MatrixXd H_full(num_v_full, num_v_full);
    plant_.CalcMassMatrixViaInverseDynamics(*plant_context_, &H_full);
    
    // ✅ Extract bottom-right part (actuated joints only)
    Eigen::MatrixXd H_actuated = H_full.bottomRightCorner(num_v_actuated, num_v_actuated);
    

    // Compute critical damping gains and scale by damping ratio. Use Eigen
    // arrays (rather than matrices) for elementwise multiplication.
    Eigen::ArrayXd temp = H_actuated.diagonal().array() * stiffness_.array();
    Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
    damping_gains *= damping_ratio_.array();

    // Compute damping torque.
    Eigen::VectorXd v = x.tail(plant_.num_actuated_dofs());
    torque->get_mutable_value() = -(damping_gains * v.array()).matrix();
  }
};

}  // namespace

template <typename T>
PD_Controller<T>::PD_Controller(
    const multibody::MultibodyPlant<T>& plant, const VectorX<double>& stiffness, const VectorX<double>& damping)
    : plant_(plant) {
  DiagramBuilder<T> builder;
  DRAKE_DEMAND(plant_.num_actuated_dofs() == stiffness.size());
  DRAKE_DEMAND(plant_.num_actuated_dofs() == damping.size());
  
  const int dim = plant_.num_actuated_dofs();
  
  /*
  torque_in ----------------------------
                                       |
  (q, v)    ------>|Gravity Comp|------+---> torque_out
               |                      /|
               --->|Damping|---------- |
               |                       |
               --->| Virtual Spring |---
  (q*, v*)  ------>|PID with kd=ki=0|
  */


  // Adds virtual springs.
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd ki = Eigen::VectorXd::Zero(dim);

  auto spring = builder.template AddSystem<PidController<T>>(stiffness, kd, ki);
  
  // Adds virtual damper.
  auto damper = builder.template AddSystem<StateDependentDamper<T>>(plant_, stiffness, damping);

  // Adds an adder to sum the gravity compensation, spring, damper, and
  // feedforward torque.
  auto adder = builder.template AddSystem<Adder<T>>(2, dim);
  
  // Connects the gravity compensation, spring, and damper torques to the adder.
  builder.Connect(spring->get_output_port(0), adder->get_input_port(0));
  builder.Connect(damper->get_output_port(0), adder->get_input_port(1));

  input_port_index_estimated_state_ = builder.ExportInput(spring->get_input_port_estimated_state(), "estimated_state");
  
  // Connects the estimated state to the damper.
  builder.ConnectInput(input_port_index_estimated_state_, damper->get_input_port(0));

  // Exposes the desired state port.
  input_port_index_desired_state_ = builder.ExportInput(spring->get_input_port_desired_state(), "desired_state");

  

  // Exposes the commanded torque port.
//   input_port_index_commanded_torque_ = builder.ExportInput(adder->get_input_port(0), "commanded_torque");

  // Exposes controller output.
  output_port_index_control_ = builder.ExportOutput(adder->get_output_port(), "control");

  builder.BuildInto(this);
}

template class PD_Controller<double>;

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake