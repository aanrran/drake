#include "examples/unitree_g1/includes/unitree_g1_controller.h"

#include "examples/unitree_g1/includes/plant_helpers.h"
#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

using drake::systems::MatrixGain;

/**
 * @brief Google Test to verify the integration of UnitreeG1Controller with
 * Drake's MultibodyPlant.
 *
 * This test sets up a MultibodyPlant for the Unitree G1, initializes the
 * controller, and verifies correct port connections and actuation.
 */
GTEST_TEST(UnitreeG1ControllerTest, PDSubsystemIntegration) {
  // Create a DiagramBuilder for managing the system connections.
  drake::systems::DiagramBuilder<double> builder;

  // Create and add a MultibodyPlant to the diagram builder.
  auto plant =
      builder.AddSystem<drake::multibody::MultibodyPlant<double>>(0.001);

  // Load the Unitree G1 URDF model into the MultibodyPlant.
  drake::multibody::Parser(plant).AddModels(
      "examples/unitree_g1/robots/g1_description/g1_23dof.urdf");

  // Manually add actuators to joints (ensures actuation).
  drake::examples::unitree_g1::helper::AddActuatorsToPlant(*plant);

  // Finalize the plant to allow usage in the simulation.
  plant->Finalize();

  // Instantiate the UnitreeG1Controller and add it to the diagram.
  auto controller = builder.AddSystem<UnitreeG1Controller<double>>(*plant);

  // Verify that the controller has the expected number of input and output
  // ports.
  EXPECT_EQ(controller->num_input_ports(), 1);
  EXPECT_EQ(controller->num_output_ports(), 1);

  // Check that the controller's input and output ports are accessible.
  EXPECT_NO_THROW(controller->get_input_port());
  EXPECT_NO_THROW(controller->get_output_port());

  // Connect the MultibodyPlant's state output to the controller's input.
  builder.Connect(plant->get_state_output_port(), controller->get_input_port());

  // Create a default context for the plant.
  auto plant_context = plant->CreateDefaultContext();

  // Extract the actuation matrix from the plant (B matrix).
  Eigen::MatrixXd B_full = plant->MakeActuationMatrix();

  // Create a MatrixGain system to apply the actuation matrix to the controller
  // output.
  auto B_gain = builder.AddSystem<MatrixGain>(B_full.transpose());

  // Connect the controller's output to the actuation gain input.
  builder.Connect(controller->get_output_port(), B_gain->get_input_port());

  // Connect the actuation gain output to the MultibodyPlant's actuation input.
  builder.Connect(B_gain->get_output_port(), plant->get_actuation_input_port());

  // Build the diagram to validate the system connections.
  auto diagram = builder.Build();

  // Create and validate a context for the diagram.
  auto context_ptr = diagram->CreateDefaultContext();
  ASSERT_NE(context_ptr, nullptr);

  // Create simulator to advance system
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();

  // Advance simulation time to ensure CalcTorque() is triggered
  double simulation_time = 1.0;  // Simulate for 1 second
  simulator.AdvanceTo(simulation_time);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
