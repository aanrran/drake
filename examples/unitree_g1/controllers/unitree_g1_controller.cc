#include "examples/unitree_g1/includes/unitree_g1_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/unitree_g1/includes/timing_logger.h"


namespace drake {
namespace examples {
namespace unitree_g1 {

UnitreeG1Controller::UnitreeG1Controller(
    const multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& kp,
    const Eigen::VectorXd& kd,
    double kappa, double kd_approx)
    : plant_(plant) {
  systems::DiagramBuilder<double> builder;
  // Connect timing logger (example usage)
  TimingLogger::GetInstance().StartTimer("DiagramBuild");
  // 1. PD Controller
  pd_controller_ = builder.AddSystem<PD_Controller<double>>(plant_, kp, kd);
  
  // 2. Approximate Simulation (Placeholder)
  // To be implemented in the same file.

  // Export PD controller ports
  builder.ExportInput(pd_controller_->get_input_port(0), "pd_input");
  builder.ExportOutput(pd_controller_->get_output_port(0), "pd_output");
  TimingLogger::GetInstance().StopTimer("DiagramBuild");

  builder.BuildInto(this);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
