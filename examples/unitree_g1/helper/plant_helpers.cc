#include "examples/unitree_g1/includes/plant_helpers.h"
#include "drake/math/rigid_transform.h"  // ✅ Ensure the math namespace is included
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace helper {

// ✅ Function to add actuators
void AddActuatorsToPlant(multibody::MultibodyPlant<double>& plant) {
    if (plant.num_actuated_dofs() == 0) { // Get Number of Actuated DOFs
        std::cout << std::fixed << std::setprecision(4);  // Set fixed-point notation with 4 decimal places
        std::cout << "------------------------------------------------------------------------------------------------------------------------\n";
        std::cout << "                                 Software Added Joint Actuator Information Table \n";
        std::cout << "------------------------------------------------------------------------------------------------------------------------\n";
        std::cout << std::setw(5) << std::left << "Idx"
                  << std::setw(35) << "Joint Name"
                  << std::setw(20) << "Effort Limit (Nm)"
                  << std::setw(25) << "Velocity Limit (rad/s)"
                  << std::setw(15) << "Pos Min (rad)"
                  << std::setw(15) << "Pos Max (rad)"
                  << std::setw(15) << "Roll (rad)"
                  << std::setw(15) << "Pitch (rad)"
                  << std::setw(15) << "Yaw (rad)"
                  << "\n";
        std::cout << "-----------------------------------------------------------------------------------------------------------------------\n";
        
        
          for (int i = 0; i < plant.num_joints(); ++i) {
              const auto& joint = plant.get_joint(drake::multibody::JointIndex(i));
      
              if (joint.num_positions() == 1 && joint.num_velocities() == 1) { // Only consider 1-DOF joints
                const auto& actuator = plant.AddJointActuator(joint.name() + "_actuator", joint);
          
                  // ✅ Extract RPY from parent frame in URDF
                  math::RigidTransform<double> X_PC = joint.frame_on_parent().GetFixedPoseInBodyFrame();
                  math::RollPitchYaw<double> rpy_PC(X_PC.rotation());
      
                  // ✅ Extract effort limit from URDF
                  double effort_limit = actuator.effort_limit();
      
                  // ✅ Extract effort limit from URDF
                  double velocity_limit = joint.velocity_upper_limits()[0];
      
                  // ✅ Extract position (angle) limits
                  double pos_min = joint.position_lower_limits()[0];
                  double pos_max = joint.position_upper_limits()[0];
          
                  // ✅ Print joint information with all limits
                  std::cout 
                    << std::setw(5) << std::left << i
                    << std::setw(35) << actuator.name()  // Print actuator name
                    << std::setw(20) << effort_limit
                    << std::setw(25) << velocity_limit
                    << std::setw(15) << pos_min
                    << std::setw(15) << pos_max
                    << std::setw(15) << rpy_PC.roll_angle()
                    << std::setw(15) << rpy_PC.pitch_angle()
                    << std::setw(15) << rpy_PC.yaw_angle()
                    << "\n";
              }
          }
      
          std::cout << "------------------------------------------------------------------------------------------------------------\n";
          std::cout << plant.num_actuated_dofs() << " actuators added to the system." << std::endl;
          std::cout << "------------------------------------------------------------------------------------------------------------\n";
      }
      
}

// ✅ Function to add a ground plane
void AddGroundPlaneToPlant(multibody::MultibodyPlant<double>& plant) {
    const double ground_size = 10.0;
    const double ground_thickness = 0.1;
    
    // Visual representation of the ground
    plant.RegisterVisualGeometry(
        plant.world_body(),
        drake::math::RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
        geometry::Box(ground_size, ground_size, ground_thickness),
        "GroundVisualGeometry",
        Vector4<double>(0.2, 0.2, 0.2, 1.0));
  
    // Collision properties for the ground
    const double static_friction = 0.8;
    const double dynamic_friction = 0.6;
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        drake::math::RigidTransformd(Eigen::Vector3d(0, 0, -ground_thickness)),
        geometry::Box(ground_size, ground_size, ground_thickness),
        "GroundCollision",
        multibody::CoulombFriction<double>(static_friction, dynamic_friction));
}

}  // namespace helper
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
