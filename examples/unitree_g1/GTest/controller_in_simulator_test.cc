#include "examples/unitree_g1/includes/plant_helpers.h"
#include "examples/unitree_g1/includes/unitree_g1_controller.h"
#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/visualization/visualization_config_functions.h"
// Define simulation time as a command-line argument
DEFINE_double(simulation_time, 5.0, "Simulation duration in seconds");
namespace drake {
namespace examples {
namespace unitree_g1 {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::MatrixGain;
using helper::AddActuatorsToPlant;
using helper::AddGroundPlaneToPlant;

int do_main() {
  DiagramBuilder<double> builder;

  // ✅ 1. Create MultibodyPlant and SceneGraph
  const double plant_frequency = 1000.0;  // Run at 100 Hz
  const double time_step = 1.0 / plant_frequency;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, time_step);

  // ✅ 2. Load the Unitree G1 model from URDF
  const std::string urdf_path =
      "examples/unitree_g1/robots/g1_description/g1_23dof.urdf";
  auto model_instance = Parser(&plant).AddModels(urdf_path).at(0);

  // ✅ 3. Manually Add Actuators to Joints (Ensures Actuation)
  AddActuatorsToPlant(plant);

  // ✅ 4. Set Initial Robot Pose
  const double initial_z_offset = 0.75;
  plant.SetDefaultFreeBodyPose(
      plant.GetBodyByName("pelvis", model_instance),
      RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));

  // ✅ Weld pelvis at an offset of (0, 0, 0.8) in world frame
  // plant.WeldFrames(
  //     plant.world_frame(), plant.GetFrameByName("pelvis"),
  //     RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));

  // ✅ 5. Add Ground Plane for Simulation
  AddGroundPlaneToPlant(plant);

  // ✅ 6. add gravity adjustment feature
  plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d(
      0, 0, -0.81));  // Default -9.81 for Earth gravity, -1.625 for moon

  // ✅ 7. Finalize Plant Before Using Actuated DOFs
  plant.Finalize();

  // Instantiate the UnitreeG1Controller and add it to the diagram.
  auto controller = builder.AddSystem<UnitreeG1Controller<double>>(plant);

  // Connect the MultibodyPlant's state output to the controller's input.
  builder.Connect(plant.get_state_output_port(), controller->get_input_port());

  // Extract the actuation matrix from the plant (B matrix).
  Eigen::MatrixXd B_full = plant.MakeActuationMatrix();

  // Create a MatrixGain system to apply the actuation matrix to the controller
  // output.
  auto B_gain = builder.AddSystem<MatrixGain>(B_full.transpose());

  // Connect the controller's output to the actuation gain input.
  builder.Connect(controller->get_output_port(), B_gain->get_input_port());

  // Connect the actuation gain output to the MultibodyPlant's actuation input.
  // builder.Connect(B_gain->get_output_port(),
  // plant.get_actuation_input_port());

  //   // ✅ 14. Apply Torque Limits
  auto torque_saturation =
      builder.AddSystem<drake::systems::Saturation<double>>(
          Eigen::VectorXd::Constant(plant.num_actuators(), -30.0),
          Eigen::VectorXd::Constant(plant.num_actuators(), 30.0));

  builder.Connect(B_gain->get_output_port(),
                  torque_saturation->get_input_port());
  builder.Connect(torque_saturation->get_output_port(),
                  plant.get_actuation_input_port());
  // Visualization Setup
  drake::visualization::AddDefaultVisualization(&builder);
  // Build the diagram to validate the system connections.
  auto diagram = builder.Build();

  // Create simulator to advance system
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();

  // Advance simulation time to ensure CalcTorque() is triggered
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;  // ✅ Normal program exit
}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::unitree_g1::do_main();
}
