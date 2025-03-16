#include <gtest/gtest.h>
#include "examples/unitree_g1/includes/unitree_g1_controller.h"
#include "examples/unitree_g1/includes/plant_helpers.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

using drake::systems::MatrixGain;

GTEST_TEST(UnitreeG1ControllerTest, PDSubsystemIntegration) {
  // Setup Drake Multibody Plant
  drake::systems::DiagramBuilder<double> builder;
  
  // Add plant to builder (Fixes the issue)
  auto plant = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(0.001);

  // Load the robot model before finalizing the plant
  drake::multibody::Parser(plant).AddModels("examples/unitree_g1/robots/g1_description/g1_23dof.urdf");

    //Manually Add Actuators to Joints (Ensures Actuation)
    drake::examples::unitree_g1::helper::AddActuatorsToPlant(*plant);

  plant->Finalize();

  // Instantiate controller diagram with PD subsystem
  auto controller = builder.AddSystem<UnitreeG1Controller<double>>(*plant);

  // Verify PD controller ports exist
  EXPECT_EQ(controller->num_input_ports(), 1);
  EXPECT_EQ(controller->num_output_ports(), 1);

  EXPECT_NO_THROW(controller->get_input_port());
  EXPECT_NO_THROW(controller->get_output_port());

  // Test existence and correct connection of input/output ports
  builder.Connect(plant->get_state_output_port(), controller->get_input_port());

  auto plant_context = plant->CreateDefaultContext();
  Eigen::MatrixXd B_full = plant->MakeActuationMatrix();
  auto B_gain = builder.AddSystem<MatrixGain>(B_full.transpose());

  builder.Connect(controller->get_output_port(), B_gain->get_input_port());

  builder.Connect(B_gain->get_output_port(), plant->get_actuation_input_port());

  // Build diagram to check correctness
  auto diagram = builder.Build();

  // Create and test context
  auto context_ptr = diagram->CreateDefaultContext();
  ASSERT_NE(context_ptr, nullptr);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
