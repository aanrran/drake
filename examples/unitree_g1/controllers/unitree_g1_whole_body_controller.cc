// unitree_g1_whole_body_controller.cc
#include "examples/unitree_g1/includes/unitree_g1_whole_body_controller.h"

#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

UnitreeG1WholeBodyController::UnitreeG1WholeBodyController(
    const multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& kp,
    const Eigen::VectorXd& kd)
    : plant_(plant) {
  systems::DiagramBuilder<double> builder;

  // Instantiate the PD Controller with the provided gains
  pd_controller_system_ = builder.AddSystem<PD_Controller<double>>(plant_, kp, kd);
  // Export the input and output ports of the PD Controller
  builder.ExportInput(pd_controller_system_->get_input_port(0), "pd_controller_input");
  builder.ExportOutput(pd_controller_system_->get_output_port(0), "pd_controller_output");

  // Build the Diagram
  builder.BuildInto(this);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
