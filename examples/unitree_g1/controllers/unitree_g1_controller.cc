#include "examples/unitree_g1/includes/unitree_g1_controller.h"
#include "examples/unitree_g1/includes/timing_logger.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/matrix_gain.h"


namespace drake {
namespace examples {
namespace unitree_g1 {

UnitreeG1Controller::UnitreeG1Controller(
    const multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& kp,
    const Eigen::VectorXd& kd,
    double kappa, double kd_approx)
    : plant_(plant), kappa_(kappa), kd_approx_(kd_approx) {
  systems::DiagramBuilder<double> builder;
  TimingLogger::GetInstance().StartTimer("DiagramBuild");
  // 1. PD Controller
  pd_controller_ = builder.AddSystem<PD_Controller<double>>(plant_, kp, kd);

  // 2. Declare Input Port for Whole-Body State
  auto whole_body_state_input = builder.AddSystem<drake::systems::PassThrough<double>>(plant.num_positions());
  
  // 3. Compute CoM Jacobian
  Eigen::MatrixXd Jcom(3, plant.num_positions());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
    *plant.CreateDefaultContext(),  // Context reference
    multibody::JacobianWrtVariable::kQDot,  
    plant.world_frame(),   // Base frame (World frame)
    plant.world_frame(),   // Expressed in World frame
    &Jcom  // Output matrix
);

  
  // 4. Define Control Law
  auto approx_simulation_controller = builder.AddSystem<drake::systems::MatrixGain<double>>(Jcom);
  
  // 5. Connect Whole-Body State Input → Approximate Simulation Controller
  builder.Connect(whole_body_state_input->get_output_port(), approx_simulation_controller->get_input_port());

  // 6. Export Ports
  builder.ExportInput(whole_body_state_input->get_input_port(), "whole_body_state");
  builder.ExportOutput(approx_simulation_controller->get_output_port(), "approx_sim_output");
  TimingLogger::GetInstance().StopTimer("DiagramBuild");
  builder.BuildInto(this);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
