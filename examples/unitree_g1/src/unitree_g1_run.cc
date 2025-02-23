// unitree_g1_run.cc
#include <memory>

#include "unitree_g1_controller.h"
#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using Eigen::Translation3d;
using Eigen::VectorXd;

int do_main() {
  systems::DiagramBuilder<double> builder;

  // Configure multibody plant
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.001;  // Small time step for better accuracy
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  // Load Unitree G1 URDF and set initial pose
  const std::string urdf_path =
      "examples/unitree_g1/robots/g1_description/g1_23dof.urdf";
  multibody::Parser parser(&plant);
  auto model_instance = parser.AddModels(urdf_path).at(0);

  // Set the initial pose slightly above the ground
  const double initial_z_offset = 0.8;
  plant.SetDefaultFreeBodyPose(
      plant.GetBodyByName("pelvis", model_instance),
      RigidTransformd(Translation3d(0, 0, initial_z_offset)));

  // Add the ground plane
  const double ground_size = 10.0;
  const double ground_thickness = 0.1;

  plant.RegisterVisualGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundVisualGeometry", Vector4<double>(0.2, 0.2, 0.2, 1.0));

  const double static_friction = 0.8;
  const double dynamic_friction = 0.6;
  plant.RegisterCollisionGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundCollision",
      multibody::CoulombFriction<double>(static_friction, dynamic_friction));

  plant.mutable_gravity_field().set_gravity_vector(
      Eigen::Vector3d(0, 0, -9.81));
  plant.Finalize();

  visualization::AddDefaultVisualization(&builder);

  // Add controller
  auto controller = builder.AddSystem<UnitreeG1Controller>(&plant);
  // Connect state output of the plant to controller input
  builder.Connect(plant.get_state_output_port(), controller->get_input_port());

  // Connect controller output to the actuation input of the plant
  builder.Connect(controller->get_output_port(), plant.get_actuation_input_port());

  auto diagram = builder.Build();
  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);

  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::unitree_g1::do_main();
}
