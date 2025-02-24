#include <memory>
#include <iostream>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 5.0, "Simulation duration in seconds");

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Adder;
using drake::systems::ConstantVectorSource;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::controllers::PidController;
using drake::systems::MatrixGain;
using drake::systems::Saturation;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int do_main() {
  DiagramBuilder<double> builder;

  // ✅ Add MultibodyPlant and SceneGraph
  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  // ✅ Load the Unitree G1 model
  const std::string urdf_path = "examples/unitree_g1/robots/g1_description/g1_23dof.urdf";
  auto model_instance = Parser(&plant).AddModels(urdf_path).at(0);

  // ✅ Manually Add Actuators
  for (int i = 0; i < plant.num_joints(); ++i) {
    const auto& joint = plant.get_joint(drake::multibody::JointIndex(i));
    if (joint.num_positions() == 1) {
      plant.AddJointActuator(joint.name() + "_actuator", joint);
      std::cout << "Added actuator: " << joint.name() << std::endl;
    }
  }
  // Set the initial pose slightly above the ground
  const double initial_z_offset = 0.8;  // Adjust as needed unit(m)
  plant.SetDefaultFreeBodyPose(
      plant.GetBodyByName("pelvis", model_instance),
      RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));
      
    // ✅ Add the ground plane
  const double ground_size = 10.0;  // Large enough to cover the scene
  const double ground_thickness = 0.1;  // Small thickness to approximate HalfSpace

  plant.RegisterVisualGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundVisualGeometry",
      Vector4<double>(0.2, 0.2, 0.2, 1.0));  // Dark grey color (R, G, B, Alpha)

  // ✅ Add collision geometry with friction
  const double static_friction = 0.8;
  const double dynamic_friction = 0.6;
  plant.RegisterCollisionGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundCollision",
      multibody::CoulombFriction<double>(static_friction, dynamic_friction));

  // ✅ Finalize the plant
  plant.Finalize();

  // ✅ Print number of actuated DOFs
  const int num_actuated_dofs = plant.num_actuated_dofs();
  std::cout << "Number of actuated DOFs: " << num_actuated_dofs << std::endl;

  // ✅ Prevent error if no actuators exist
  if (num_actuated_dofs == 0) {
    throw std::runtime_error("No actuators found in model! Check the URDF.");
  }

  // ✅ Add PID controller
  VectorXd Kp = VectorXd::Constant(num_actuated_dofs, 20.0);
  VectorXd Ki = VectorXd::Zero(num_actuated_dofs);
  VectorXd Kd = VectorXd::Constant(num_actuated_dofs, 2.0);
  auto pid_controller = builder.AddSystem<PidController<double>>(Kp, Ki, Kd);

  // ✅ Create constant desired state
  VectorXd desired_state = VectorXd::Zero(2 * num_actuated_dofs);
  auto desired_state_source = builder.AddSystem<ConstantVectorSource<double>>(desired_state);
  builder.Connect(desired_state_source->get_output_port(), pid_controller->get_input_port_desired_state());

  // ✅ Create state selection matrix
  const int num_positions = plant.num_positions();
  const int num_velocities = plant.num_velocities();
  Eigen::MatrixXd selection_matrix = Eigen::MatrixXd::Zero(num_actuated_dofs * 2, num_positions + num_velocities);

  for (int i = 0; i < num_actuated_dofs; ++i) {
    selection_matrix(i, i) = 1.0;  // Position
    selection_matrix(num_actuated_dofs + i, num_positions + i) = 1.0;  // Velocity
  }

  auto state_selector = builder.AddSystem<MatrixGain>(selection_matrix);

  // ✅ Connect plant state output to state selector
  builder.Connect(plant.get_state_output_port(), state_selector->get_input_port());

  // ✅ Connect state selector output to PID controller (fixing missing connection!)
  builder.Connect(state_selector->get_output_port(), pid_controller->get_input_port_estimated_state());

  // ✅ Compute gravity compensation torques
  auto plant_context = plant.CreateDefaultContext();
  VectorXd tau_g_full = plant.CalcGravityGeneralizedForces(*plant_context);
  Eigen::MatrixXd B = plant.MakeActuationMatrix();
  VectorXd tau_g = B.transpose() * tau_g_full;
  auto gravity_compensation = builder.AddSystem<ConstantVectorSource<double>>(tau_g);

  // ✅ Sum PD control and gravity compensation using an Adder system
  std::cout << "Creating Adder with " << num_actuated_dofs << " DOFs." << std::endl;
  auto torque_adder = builder.AddSystem<Adder<double>>(2, num_actuated_dofs);

  // ✅ Clamp torques to safe range
  auto torque_saturation = builder.AddSystem<Saturation<double>>(VectorXd::Constant(num_actuated_dofs, -50.0),
                                                                 VectorXd::Constant(num_actuated_dofs, 50.0));

  // ✅ Connect components
  builder.Connect(pid_controller->get_output_port_control(), torque_adder->get_input_port(0));
  builder.Connect(gravity_compensation->get_output_port(), torque_adder->get_input_port(1));
  builder.Connect(torque_adder->get_output_port(), torque_saturation->get_input_port());
  builder.Connect(torque_saturation->get_output_port(), plant.get_actuation_input_port());

  // ✅ Visualization
  drake::visualization::AddDefaultVisualization(&builder);

  // ✅ Build diagram
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  // ✅ Run simulation
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

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
