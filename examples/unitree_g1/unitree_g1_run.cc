


#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "./includes/pd_controller.h"
#include "./includes/plant_helpers.h"
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
  using drake::systems::MatrixGain;
  using drake::systems::Saturation;
  using drake::systems::Simulator;
  using drake::systems::controllers::PidController;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  
  using helper::AddActuatorsToPlant;
  using helper::AddGroundPlaneToPlant;

  using systems::controllers::StateFeedbackControllerInterface;

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
  const double initial_z_offset = 0.8;
  plant.SetDefaultFreeBodyPose(
      plant.GetBodyByName("pelvis", model_instance),
      RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));

  // ✅ Weld pelvis at an offset of (0, 0, 0.8) in world frame
  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("pelvis"),
      RigidTransformd(Eigen::Translation3d(0, 0, initial_z_offset)));

  // ✅ 5. Add Ground Plane for Simulation
  AddGroundPlaneToPlant(plant);

  // ✅ 6. add gravity adjustment feature
  plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d(
      0, 0, 0.0));  // Default -9.81 for Earth gravity, -1.625 for moon

  // ✅ 7. Finalize Plant Before Using Actuated DOFs
  plant.Finalize();

  // ✅ 8. Setup PID Controller
  const int num_q = plant.num_positions();
  ;
  const int num_v = plant.num_velocities();
  ;
  const int num_x = num_q + num_v;
  const int num_actuated_dofs = plant.num_actuated_dofs();

  std::cout << "number of q:" << num_q << ", number of v:" << num_v
            << ", number of x:" << num_x << std::endl;

  VectorXd Kp = VectorXd::Constant(num_actuated_dofs, 5.2);
  VectorXd Ki = VectorXd::Zero(num_actuated_dofs);
  VectorXd Kd = VectorXd::Constant(num_actuated_dofs, 0.7);
//   auto pid_controller = builder.AddSystem<PidController<double>>(Kp, Ki, Kd);
    StateFeedbackControllerInterface<double>* pid_controller = builder.AddSystem<PD_Controller<double>>(plant, Kp, Kd);

  // ✅ 9. Setup Desired Joint States (Hold at Zero)
  VectorXd desired_state = VectorXd::Zero(2 * num_actuated_dofs);
  desired_state << 0.0,  // left_hip_pitch_joint
      0.0,               // left_hip_roll_joint
      0.0,               // left_hip_yaw_joint
      0.5,               // left_knee_joint
      0.0,               // left_ankle_pitch_joint
      0.0,               // left_ankle_roll_joint
      0.0,               // right_hip_pitch_joint
      0.0,               // right_hip_roll_joint
      0.0,               // right_hip_yaw_joint
      0.5,               // right_knee_joint
      0.0,               // right_ankle_pitch_joint
      0.0,               // right_ankle_roll_joint
      0.0,               // waist_yaw_joint
      0.0,               // left_shoulder_pitch_joint
      0.0,               // left_shoulder_roll_joint
      0.0,               // left_shoulder_yaw_joint
      0.0,               // left_elbow_joint
      -0.1,               // left_wrist_roll_joint
      0.0,               // right_shoulder_pitch_joint
      0.0,               // right_shoulder_roll_joint
      0.0,               // right_shoulder_yaw_joint
      0.0,               // right_elbow_joint
      0.0,               // right_wrist_roll_joint

      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto desired_state_source =
      builder.AddSystem<ConstantVectorSource<double>>(desired_state);

  builder.Connect(desired_state_source->get_output_port(),
                  pid_controller->get_input_port_desired_state());

  // ✅ 10. Setup State Selection Matrix for Actuated Joints
  Eigen::MatrixXd selection_matrix =
      Eigen::MatrixXd::Zero(2 * num_actuated_dofs, num_x);
  // ✅ Extract actuated positions & velocities (skipping floating base)
  for (int i = 0; i < num_actuated_dofs; ++i) {
    selection_matrix(i, num_q - num_actuated_dofs + i) = 1.0;  // Positions
    selection_matrix(num_actuated_dofs + i, num_q + num_v - num_actuated_dofs +
                                                i) = 1.0;  // Velocities
  }

  auto state_selector = builder.AddSystem<MatrixGain>(selection_matrix);

  // ✅ 11. Connect Plant State to PID Controller
  builder.Connect(plant.get_state_output_port(),
                  state_selector->get_input_port());
  builder.Connect(state_selector->get_output_port(),
                  pid_controller->get_input_port_estimated_state());

// VectorXd feedforward_torque = VectorXd::Zero(num_actuated_dofs);
// auto feedforward_source = builder.AddSystem<ConstantVectorSource<double>>(feedforward_torque);
// builder.Connect(feedforward_source->get_output_port(), pid_controller->get_input_port_commanded_torque());

  // ✅ Compute Actuation Matrix
  auto plant_context = plant.CreateDefaultContext();
  Eigen::MatrixXd B_full = plant.MakeActuationMatrix();
  Eigen::MatrixXd B = B_full.transpose().rightCols(
      num_actuated_dofs);  // ✅ Extract last 23 columns
  auto B_gain = builder.AddSystem<MatrixGain>(B);

  builder.Connect(pid_controller->get_output_port_control(),
                  B_gain->get_input_port());

  //   // ✅ 14. Apply Torque Limits
  auto torque_saturation = builder.AddSystem<Saturation<double>>(
      VectorXd::Constant(num_actuated_dofs, -5.0),
      VectorXd::Constant(num_actuated_dofs, 5.0));

  builder.Connect(B_gain->get_output_port(),
                  torque_saturation->get_input_port());
  builder.Connect(torque_saturation->get_output_port(),
                  plant.get_actuation_input_port());

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
