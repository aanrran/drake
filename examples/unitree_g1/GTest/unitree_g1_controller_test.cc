#include <gtest/gtest.h>
#include "examples/unitree_g1/includes/unitree_g1_controller.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

GTEST_TEST(UnitreeG1ControllerTest, PDSubsystemIntegration) {
  // Setup Drake Multibody Plant
  drake::multibody::MultibodyPlant<double> plant(0.001);
  plant.Finalize();

  // Define arbitrary PD gains for testing
  const Eigen::VectorXd kp = Eigen::VectorXd::Constant(plant.num_positions(), 100.0);
  const Eigen::VectorXd kd = Eigen::VectorXd::Constant(plant.num_velocities(), 10.0);

  // Instantiate controller diagram with PD subsystem
  UnitreeG1Controller controller(plant, kp, kd, 0.1, 10.0);

  // Create a context to simulate the controller
  auto context = controller.CreateDefaultContext();

  // Test existence and correct connection of input/output ports
  EXPECT_NO_THROW(controller.GetInputPort("pd_input"));
  EXPECT_NO_THROW(controller.GetOutputPort("pd_output"));

  // Verify PD controller ports exist
  EXPECT_EQ(controller.num_input_ports(), 1);
  EXPECT_EQ(controller.num_output_ports(), 1);

  // Create and test context
  auto context_ptr = controller.CreateDefaultContext();
  ASSERT_NE(context_ptr, nullptr);
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake