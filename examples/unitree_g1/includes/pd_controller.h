#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

// N.B. Inheritance order must remain fixed for pydrake (#9243).
/**
 * @system
 * name: PD_Controller
 * input_ports:
 * - estimated_state
 * - desired_state
 * - commanded_torque
 * output_ports:
 * - control
 * @endsystem
 *
 * @tparam_double_only
 */
template <typename T>
class PD_Controller
    : public systems::Diagram<T>,
      public systems::controllers::StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PD_Controller);

  /// @p plant is aliased and must remain valid for the lifetime of the
  /// controller.
  PD_Controller(const multibody::MultibodyPlant<T>& plant,
                       const VectorX<double>& stiffness,
                       const VectorX<double>& damping);

  // const systems::InputPort<T>& get_input_port_commanded_torque() const {
  //   return systems::Diagram<T>::get_input_port(
  //       input_port_index_commanded_torque_);
  // }

  const systems::InputPort<T>& get_input_port_estimated_state() const override {
    return systems::Diagram<T>::get_input_port(
        input_port_index_estimated_state_);
  }

  const systems::InputPort<T>& get_input_port_desired_state() const override {
    return systems::Diagram<T>::get_input_port(input_port_index_desired_state_);
  }

  const systems::OutputPort<T>& get_output_port_control() const override {
    return systems::Diagram<T>::get_output_port(output_port_index_control_);
  }

    const systems::InputPort<T>& get_input_port_full_state() const {
    return systems::Diagram<T>::get_input_port(
      input_port_index_full_state_);
  }
 private:
  const multibody::MultibodyPlant<T>& plant_;
  systems::InputPortIndex input_port_index_estimated_state_;
  systems::InputPortIndex input_port_index_desired_state_;
  systems::InputPortIndex input_port_index_full_state_;
  systems::InputPortIndex input_port_index_commanded_torque_;
  systems::OutputPortIndex output_port_index_control_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake