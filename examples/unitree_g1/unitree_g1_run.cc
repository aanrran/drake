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

// Define simulation time as a command-line argument
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

  // ✅ 1. Create MultibodyPlant and SceneGraph
  auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  // ✅ 2. Load the Unitree G1 model from URDF
  const std::string urdf_path = "examples/unitree_g1/robots/g1_description/g1_23dof.urdf";
  auto model_instance = Parser(&plant).AddModels(urdf_path).at(0);

  // ✅ 3. Manually Add Actuators to Joints (Ensures Actuation)
  for (int i = 0; i < plant.num_joints(); ++i) {
    const auto& joint = plant.get_joint(drake::multibody::JointIndex(i));
    if (joint.num_positions() == 1) {  // Only actuate single DOF joints
      plant.AddJointActuator(joint.name() + "_actuator", joint);
      std::cout << "Added actuator: " << joint.name() << std::endl;
    }
  }

  // ✅ 4. Set Initial Robot Pose
  const double initial_z_offset = 0.8;
  plant.SetDefaultFreeBodyPose(
      plant.GetBodyByName("pelvis", model_instance),
      RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));

  // ✅ 5. Add Ground Plane for Simulation
  const double ground_size = 10.0;
  const double ground_thickness = 0.1;
  
  // Visual representation of the ground
  plant.RegisterVisualGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundVisualGeometry",
      Vector4<double>(0.2, 0.2, 0.2, 1.0));

  // Collision properties for the ground
  const double static_friction = 0.8;
  const double dynamic_friction = 0.6;
  plant.RegisterCollisionGeometry(
      plant.world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
      geometry::Box(ground_size, ground_size, ground_thickness),
      "GroundCollision",
      multibody::CoulombFriction<double>(static_friction, dynamic_friction));

  // ✅ 6. Finalize Plant Before Using Actuated DOFs
  plant.Finalize();

  // ✅ 7. Get Number of Actuated DOFs
  const int num_actuated_dofs = plant.num_actuated_dofs();
  std::cout << "Number of actuated DOFs: " << num_actuated_dofs << std::endl;

  if (num_actuated_dofs == 0) {
    throw std::runtime_error("No actuators found in model! Check the URDF.");
  }

  // ✅ 8. Setup PID Controller
  VectorXd Kp = VectorXd::Constant(num_actuated_dofs, 20.0);
  VectorXd Ki = VectorXd::Zero(num_actuated_dofs);
  VectorXd Kd = VectorXd::Constant(num_actuated_dofs, 2.0);
  auto pid_controller = builder.AddSystem<PidController<double>>(Kp, Ki, Kd);

  // ✅ 9. Setup Desired Joint States (Hold at Zero)
  VectorXd desired_state = VectorXd::Zero(2 * num_actuated_dofs);
  auto desired_state_source = builder.AddSystem<ConstantVectorSource<double>>(desired_state);
  builder.Connect(desired_state_source->get_output_port(), pid_controller->get_input_port_desired_state());

  // ✅ 10. Setup State Selection Matrix for Actuated Joints
  const int num_positions = plant.num_positions();
  const int num_velocities = plant.num_velocities();
  Eigen::MatrixXd selection_matrix = Eigen::MatrixXd::Zero(num_actuated_dofs * 2, num_positions + num_velocities);

  for (int i = 0; i < num_actuated_dofs; ++i) {
    selection_matrix(i, i) = 1.0;
    selection_matrix(num_actuated_dofs + i, num_positions + i) = 1.0;
  }
  auto state_selector = builder.AddSystem<MatrixGain>(selection_matrix);

  // ✅ 11. Connect Plant State to PID Controller
  builder.Connect(plant.get_state_output_port(), state_selector->get_input_port());
  builder.Connect(state_selector->get_output_port(), pid_controller->get_input_port_estimated_state());

  // ✅ 12. Compute Gravity Compensation Torques
  auto plant_context = plant.CreateDefaultContext();
  VectorXd tau_g_full = plant.CalcGravityGeneralizedForces(*plant_context);
  Eigen::MatrixXd B = plant.MakeActuationMatrix();
  VectorXd tau_g = B.transpose() * tau_g_full;
  auto gravity_compensation = builder.AddSystem<ConstantVectorSource<double>>(tau_g);

  // ✅ 13. Sum PD Controller Output and Gravity Compensation
  auto torque_adder = builder.AddSystem<Adder<double>>(2, num_actuated_dofs);

  // ✅ 14. Apply Torque Limits
  auto torque_saturation = builder.AddSystem<Saturation<double>>(VectorXd::Constant(num_actuated_dofs, -50.0),
                                                                 VectorXd::Constant(num_actuated_dofs, 50.0));

  // ✅ 15. Connect Components in Diagram
  builder.Connect(pid_controller->get_output_port_control(), torque_adder->get_input_port(0));
  builder.Connect(gravity_compensation->get_output_port(), torque_adder->get_input_port(1));
  builder.Connect(torque_adder->get_output_port(), torque_saturation->get_input_port());
  builder.Connect(torque_saturation->get_output_port(), plant.get_actuation_input_port());

  // ✅ 16. Visualization Setup
  drake::visualization::AddDefaultVisualization(&builder);

  // ✅ 17. Build and Simulate
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
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
