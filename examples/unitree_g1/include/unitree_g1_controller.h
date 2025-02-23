// unitree_g1_controller.h
#ifndef UNITREE_G1_CONTROLLER_H_
#define UNITREE_G1_CONTROLLER_H_

#include "controller_base.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

class UnitreeG1Controller : public ControllerBase {
 public:
  explicit UnitreeG1Controller(const multibody::MultibodyPlant<double>* plant);

  // Compute control torques (initially zero)
  void ComputeControl(const systems::Context<double>& context, Eigen::VectorXd* torques) const override;

  // Input/Output ports
  const systems::InputPort<double>& get_input_port() const { return LeafSystem<double>::get_input_port(0); }
  const systems::OutputPort<double>& get_output_port() const { return LeafSystem<double>::get_output_port(0); }

 private:
  void CalcTorques(const systems::Context<double>& context, systems::BasicVector<double>* output) const;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

#endif  // UNITREE_G1_CONTROLLER_H_